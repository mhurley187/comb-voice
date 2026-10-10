// comb_core.h - stereo comb-filter synth voice DSP (no disting NT API dependencies)
//
// Signal flow
//   exciter (noise osc and/or external audio) --[freq shifter, "Input" position]-->
//   two comb ENGINES (A and B), each a left and a right delay loop. Inside every loop:
//     delay -> stereo rotation -> comb-specific filter -> swirl allpasses -> damping ->
//     [freq shifter, "Loop" position] -> feedback gain -> mutation -> DC block -> soft limit
//   morph (A->B) -> width -> wet level, plus dry external audio -> [freq shifter, "Output" position]
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "nt_math.h"

namespace comb {

static const int kMaxDelay = 8192;            // power of two; 8192 @ 48k -> ~5.9 Hz lowest pitch
static const int kMask = kMaxDelay - 1;
static const int kRegenLen = 4096;            // shifter regen delay line (power of two, ~85 ms @ 48k)
static const int kRegenMask = kRegenLen - 1;
static const int kNumCombTypes = 6;
static const int kNumMutations = 7;
static const float kBaseHz = 261.6256f;       // 0 V = C4
static const float kDcR = 0.9995f;            // loop DC-blocker pole (~4 Hz corner at 48k)
static const float kTwoPi = 6.2831853f;
static const size_t kDramFloats = 4 * (size_t)kMaxDelay + 2 * (size_t)kRegenLen;   // 4 loops + 2 regen lines

// Cheap, bounded, unity-slope-at-zero tanh approximation.
inline float ftanh(float x)
{
	if (x > 3.f) x = 3.f; else if (x < -3.f) x = -3.f;
	float x2 = x * x;
	return x * (27.f + x2) / (27.f + 9.f * x2);
}

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

struct Params
{
	// pitch
	float semis = 0.f;          // coarse + fine, in semitones
	float spreadCents = 10.f;   // L/R detune (each side moves +/- this)
	// loop
	float feedback = 0.9f;      // 0..1 (>= ~0.98 self-oscillates)
	float damping = 0.5f;       // 0..1 (1 = dark)
	float dispersion = 0.5f;    // 0..1, used by the "Metal" comb
	// engines
	int   typeA = 0, mutA = 0, typeB = 2, mutB = 1;
	float morph = 0.f;          // 0 = A, 1 = B
	float mutAmount = 0.5f;     // 0..1
	// modulation
	float chaos = 0.f;          // 0..1
	float chaosHz = 0.5f;
	float swirl = 0.5f;         // 0..1: depth of the in-loop swirl (delay wobble + swept allpasses + stereo rotation)
	float swirlHz = 0.34f;      // swirl LFO rate
	// oscillator / input
	bool  pluckMode = false;    // false = drone (continuous noise), true = gate-triggered burst
	float excLevel = 0.7f;      // 0..1 internal oscillator (noise exciter) level; 0 = off
	float excDecayMs = 120.f;
	float extLevel = 1.f;       // 0..2, external audio sent into the loops
	float dry = 0.f;            // 0..1, external audio passed straight to the output
	float wet = 1.f;            // 0..1, comb output level
	// frequency shifter
	int   shPos = 0;            // 0 off, 1 input (before the loops), 2 in the loop, 3 output
	int   shMode = 0;           // 0 up, 1 down, 2 split (L up / R down), 3 ring mod (up + down)
	float shMix = 1.f;          // 0..1 shifted vs unshifted
	float shiftHz = 0.f;        // signed base shift
	float linHzPerV = 0.f;      // linear (through-zero) FM: Hz per volt
	float expOctPerV = 0.f;     // exponential FM: octaves of shift scaling per volt
	float regen = 0.f;          // 0..0.95 shifter regeneration (input/output positions)
	int   regenRoute = 0;       // 0 down, 1 up, 2 both
	float regenMs = 10.f;       // regeneration delay
	// output
	bool  freeze = false;
	float width = 1.f;          // 0..2
	float gain = 1.f;           // linear
};

struct Inputs
{
	const float* pitch = nullptr;   // 1 V/oct CV, volts
	const float* gate = nullptr;    // volts
	const float* extL = nullptr;    // external audio, +/-1 == full scale
	const float* extR = nullptr;    // if null, extL is used for both sides
	const float* linFM = nullptr;   // shifter linear FM CV, volts
	const float* expFM = nullptr;   // shifter exponential FM CV, volts
};

struct Wander
{
	float cur = 0.f, target = 0.f, phase = 0.f;
	uint32_t rng;
	explicit Wander(uint32_t seed = 1u) : rng(seed) {}
	inline float next(float hz, float sr)
	{
		phase += hz / sr;
		if (phase >= 1.f)
		{
			phase -= 1.f;
			rng = rng * 1664525u + 1013904223u;
			target = ((rng >> 8) * (1.f / 8388608.f)) - 1.f;   // -1..1
		}
		float slew = clampf(6.f * hz / sr, 0.f, 0.5f);
		cur += slew * (target - cur);
		return cur;
	}
};

// ---------------------------------------------------------------------------------------------
// Frequency shifter: Hilbert transform (two IIR allpass chains, 90 degrees apart to within 0.7
// degrees from ~20 Hz to ~24 kHz) + quadrature oscillator = single-sideband modulation.
// Coefficients are Olli Niemitalo's 12-pole design (squared).
// ---------------------------------------------------------------------------------------------
static const float kHilA2[4] = { 0.479400866f, 0.876218494f, 0.976597590f, 0.997499256f };   // path I (+1 sample delay)
static const float kHilB2[4] = { 0.161758498f, 0.733028932f, 0.945349700f, 0.990599157f };   // path Q

struct Allpass2     // H(z) = (c - z^-2) / (1 - c z^-2)
{
	float x1 = 0.f, x2 = 0.f, y1 = 0.f, y2 = 0.f;
	inline float run(float x, float c)
	{
		float y = c * (x + y2) - x2;
		x2 = x1; x1 = x; y2 = y1; y1 = y;
		return y;
	}
};

struct Hilbert
{
	Allpass2 a[4], b[4];
	float dly = 0.f;
	inline void run(float x, float& I, float& Q)
	{
		float ya = x, yb = x;
		for (int k = 0; k < 4; ++k) { ya = a[k].run(ya, kHilA2[k]); yb = b[k].run(yb, kHilB2[k]); }
		I = dly; dly = ya;
		Q = yb;
	}
	// Up = Q cos(p) - I sin(p),  Down = Q cos(p) + I sin(p)   (p = oscillator phase)
	inline void shift(float x, float c, float s, float& up, float& dn)
	{
		float I, Q;
		run(x, I, Q);
		up = Q * c - I * s;
		dn = Q * c + I * s;
	}
};

// Phase delay (samples) of the Q path at angular frequency w. Used to keep the comb in tune
// when the shifter sits inside the loop.
inline float pdHilbertQ(float w)
{
	float c2w = ntm::cos(2.f * w), s2w = ntm::sin(2.f * w);
	float tot = 0.f;
	for (int k = 0; k < 4; ++k)
	{
		float c = kHilB2[k];
		float p = ntm::atan2(s2w, c - c2w) - ntm::atan2(c * s2w, 1.f - c * c2w) - 3.14159265f;   // minus the DC polarity flip
		if (p > 0.f) p -= kTwoPi; else if (p < -kTwoPi) p += kTwoPi;
		tot += p;
	}
	return -tot / w;
}

struct ShiftCtl
{
	bool  loop = false;     // shifter active inside the loops
	float c = 1.f, s = 0.f; // quadrature oscillator
	float mix = 0.f;
	int   mode = 0;
};

inline float pickMode(int mode, int ch, float up, float dn)
{
	switch (mode)
	{
	default:
	case 0: return up;
	case 1: return dn;
	case 2: return ch == 0 ? up : dn;
	case 3: return 0.5f * (up + dn);
	}
}

// Stereo shifter with Koszalin-style regeneration (used at the Input and Output positions).
struct StereoShifter
{
	Hilbert h[2];
	float* rg[2] = { nullptr, nullptr };
	int w = 0;

	void init(float* mem)
	{
		rg[0] = mem; rg[1] = mem + kRegenLen; w = 0;
		for (int i = 0; i < 2 * kRegenLen; ++i) mem[i] = 0.f;
	}

	inline void run(const float in[2], float c, float s, int mode, float mix, float regen, int route,
	                float regenSamples, float out[2])
	{
		float d = clampf(regenSamples, 4.f, (float)(kRegenLen - 4));
		int di = (int)d; float t = d - (float)di;
		float wr[2];
		for (int ch = 0; ch < 2; ++ch)
		{
			float a = rg[ch][(w - di) & kRegenMask], b = rg[ch][(w - di - 1) & kRegenMask];
			float fb = a + t * (b - a);
			float x = in[ch] + regen * fb;
			float up, dn;
			h[ch].shift(x, c, s, up, dn);
			float y = pickMode(mode, ch, up, dn);
			out[ch] = in[ch] + mix * (y - in[ch]);
			wr[ch] = ftanh(route == 0 ? dn : (route == 1 ? up : 0.5f * (up + dn)));
		}
		rg[0][w] = wr[0]; rg[1][w] = wr[1];
		w = (w + 1) & kRegenMask;
	}
};

struct DelayLine
{
	float* buf = nullptr;
	int w = 0;
	void init(float* b) { buf = b; w = 0; for (int i = 0; i < kMaxDelay; ++i) buf[i] = 0.f; }
	inline void write(float x) { buf[w] = x; w = (w + 1) & kMask; }
	// d = delay in samples measured from the most recent write (d >= 3). Hermite interpolation.
	inline float read(float d) const
	{
		int di = (int)d;
		float t = d - (float)di;
		float xm1 = buf[(w - (di - 1)) & kMask];
		float x0  = buf[(w - di) & kMask];
		float x1  = buf[(w - (di + 1)) & kMask];
		float x2  = buf[(w - (di + 2)) & kMask];
		float c1 = 0.5f * (x1 - xm1);
		float c2 = xm1 - 2.5f * x0 + 2.f * x1 - 0.5f * x2;
		float c3 = 0.5f * (x2 - xm1) + 1.5f * (x0 - x1);
		return ((c3 * t + c2) * t + c1) * t + x0;
	}
};

struct Loop
{
	DelayLine dl;
	float lp = 0.f, ksPrev = 0.f;
	float dcx = 0.f, dcy = 0.f;
	float apx[4] = { 0, 0, 0, 0 }, apy[4] = { 0, 0, 0, 0 };      // "Metal" dispersion allpasses
	float sapx[4] = { 0, 0, 0, 0 }, sapy[4] = { 0, 0, 0, 0 };    // swirl allpasses
	Hilbert hil;                                                  // in-loop frequency shifter
	float hold = 0.f; int holdCnt = 0;
	float y = 0.f;
	void reset()
	{
		lp = ksPrev = dcx = dcy = hold = y = 0.f; holdCnt = 0;
		for (int i = 0; i < 4; ++i) apx[i] = apy[i] = sapx[i] = sapy[i] = 0.f;
		hil = Hilbert();
	}
};

struct SwirlCtl
{
	bool  on = false;
	float ap[2] = { 0.f, 0.f };     // per-channel swirl allpass coefficient
	float rc = 1.f, rs = 0.f;       // stereo rotation (cos, sin)
};

struct Engine
{
	Loop ch[2];

	// Mutation applied to the (already gain-scaled) feedback signal.
	static inline float mutate(Loop& L, float s, int mut, float amt)
	{
		switch (mut)
		{
		default:
		case 0: return s;
		case 1: return ftanh(s * (1.f + 6.f * amt));                       // saturate
		case 2: return ntm::sin(s * (1.f + 4.f * amt) * 1.5707963f);          // wavefold
		case 3: {                                                           // bit crush
			float levels = ntm::exp2(11.f - 9.f * amt);
			return ntm::floor(s * levels + 0.5f) / levels;
		}
		case 4: return clampf(s * (1.f + 4.f * amt) + 0.25f * amt, -1.f, 1.f);   // asymmetric hard clip
		case 5: {                                                           // sample-rate decimate
			int period = 1 + (int)(amt * 40.f);
			if (--L.holdCnt <= 0) { L.hold = s; L.holdCnt = period; }
			return L.hold;
		}
		case 6: return s + amt * ntm::fabs(s) * 1.2f;                          // asymmetric rectify (keeps the fundamental)
		}
	}

	// Phase delay (in samples) at angular frequency w of the loop's filters. Using phase delay (not
	// group delay) is what makes the loop's fundamental land exactly on the requested pitch.
	static inline float pdLowpass(float a, float w)
	{
		float b = 1.f - a;
		return ntm::atan2(b * ntm::sin(w), 1.f - b * ntm::cos(w)) / w;
	}
	static inline float pdDcBlock(float w)   // negative = phase lead
	{
		const float R = kDcR;
		float phi = 1.5707963f - 0.5f * w - ntm::atan2(R * ntm::sin(w), 1.f - R * ntm::cos(w));
		return -phi / w;
	}
	static inline float pdAllpass(float g, float w)
	{
		float phi = ntm::atan2(-ntm::sin(w), g + ntm::cos(w)) - ntm::atan2(-g * ntm::sin(w), 1.f + g * ntm::cos(w));
		return -phi / w;
	}

	float comp[2] = { 0.f, 0.f };
	int compCnt = 0;

	// One sample. f[2] = per-channel pitch in Hz, exc[2] = excitation, out[2] = comb outputs.
	void tick(int type, int mut, float sr, const float f[2], const Params& P,
	          const float g[2], float damp, float fr, const float exc[2],
	          const SwirlCtl& sw, const ShiftCtl& sh, float out[2])
	{
		const float gd = 0.25f + 0.5f * P.dispersion;           // allpass coefficient
		const bool refresh = (--compCnt <= 0);
		if (refresh) compCnt = 8;

		float a[2];
		for (int c = 0; c < 2; ++c)
		{
			// one-pole damping: cutoff tracks pitch so decay character is consistent across notes
			float fc = f[c] * (1.f + (1.f - damp) * 40.f);
			fc = clampf(fc, 20.f, 0.45f * sr);
			a[c] = 1.f - ntm::exp(-kTwoPi * fc / sr);
			a[c] = a[c] + (1.f - a[c]) * fr;                   // freeze: bypass damping
			a[c] = clampf(a[c], 0.05f, 1.f);

			if (refresh)
			{
				float w = kTwoPi * f[c] / sr;
				float C = pdDcBlock(w);
				if (type == 2) C += 0.5f;                       // 2-point average = half a sample
				else           C += pdLowpass(a[c], w);
				if (type == 3) C += 4.f * pdAllpass(gd, w);
				if (sw.on)     C += 4.f * pdAllpass(sw.ap[c], w);
				if (sh.loop)   C += sh.mix * pdHilbertQ(w);
				comp[c] = C;
			}

			float T = (type == 1) ? 0.5f * sr / f[c] : sr / f[c];   // hollow comb: odd harmonics of f
			float D = clampf(T - comp[c], 3.f, (float)(kMaxDelay - 8));
			if (type == 4)                                           // thick: two taps +/- ~6 cents
			{
				float d1 = clampf(D * 0.9967f, 3.f, (float)(kMaxDelay - 8));
				float d2 = clampf(D * 1.0033f, 3.f, (float)(kMaxDelay - 8));
				ch[c].y = 0.5f * (ch[c].dl.read(d1) + ch[c].dl.read(d2));
			}
			else ch[c].y = ch[c].dl.read(D);
		}

		// stereo rotation of the feedback (energy preserving)
		float yr[2] = { ch[0].y, ch[1].y };
		if (sw.on)
		{
			yr[0] = sw.rc * ch[0].y - sw.rs * ch[1].y;
			yr[1] = sw.rs * ch[0].y + sw.rc * ch[1].y;
		}

		for (int c = 0; c < 2; ++c)
		{
			Loop& L = ch[c];
			float y = yr[c];
			float s;
			switch (type)
			{
			default:
			case 0: case 4: s = y; break;
			case 1: s = -y; break;
			case 2: {
				float avg = 0.5f * (y + L.ksPrev);
				L.ksPrev = y;
				s = avg + (y - avg) * fr;
				break;
			}
			case 3: {
				s = y;
				for (int k = 0; k < 4; ++k)
				{
					float o = gd * s + L.apx[k] - gd * L.apy[k];
					L.apx[k] = s; L.apy[k] = o; s = o;
				}
				break;
			}
			case 5: s = 0.5f * (y + yr[1 - c]); break;                  // stereo cross-coupled
			}

			if (sw.on)                                                  // swirl: swept allpass chain
			{
				const float ga = sw.ap[c];
				for (int k = 0; k < 4; ++k)
				{
					float o = ga * s + L.sapx[k] - ga * L.sapy[k];
					L.sapx[k] = s; L.sapy[k] = o; s = o;
				}
			}

			if (type != 2) { L.lp += a[c] * (s - L.lp); s = L.lp; }

			if (sh.loop)                                                // frequency shifter inside the loop
			{
				float up, dn;
				L.hil.shift(s, sh.c, sh.s, up, dn);
				s += sh.mix * (pickMode(sh.mode, c, up, dn) - s);
			}

			s *= g[c];
			s = mutate(L, s, mut, P.mutAmount);

			float dc = s - L.dcx + kDcR * L.dcy;                      // DC blocker
			L.dcx = s; L.dcy = dc;
			s = ftanh(dc);                                              // keeps self-oscillation bounded

			L.dl.write(exc[c] * (1.f - fr) + s);
			out[c] = ch[c].y;
		}
	}
};

struct Voice
{
	float sr = 48000.f;
	Engine eng[2];
	StereoShifter ss;                       // used at the Input or Output position
	Wander wl{ 11u }, wr{ 22u }, wm{ 33u };
	uint32_t noiseState = 0x12345678u;
	float env = 0.f;
	bool gateHigh = false;
	float frAmt = 0.f;
	float semisSlew = 0.f;
	float ph1 = 0.f, ph2 = 0.f, ph3 = 0.f;  // swirl LFO phases (turns)
	float shPh = 0.f;                       // shifter oscillator phase (turns)
	float shMixS = 0.f;                     // smoothed shifter mix
	bool primed = false;

	void init(float sampleRate, float* dram)
	{
		sr = sampleRate;
		for (int e = 0; e < 2; ++e)
			for (int c = 0; c < 2; ++c)
			{
				eng[e].ch[c].dl.init(dram + (size_t)(e * 2 + c) * kMaxDelay);
				eng[e].ch[c].reset();
			}
		ss.init(dram + (size_t)4 * kMaxDelay);
		env = 0.f; gateHigh = false; frAmt = 0.f; primed = false;
		ph1 = ph2 = ph3 = shPh = shMixS = 0.f;
	}

	inline float noise()
	{
		noiseState ^= noiseState << 13; noiseState ^= noiseState >> 17; noiseState ^= noiseState << 5;
		return ((int32_t)noiseState) * (1.f / 2147483648.f);
	}

	static inline float wrap01(float x) { return x - ntm::floor(x); }

	void process(const Params& P, const Inputs& in, float* outL, float* outR, int n)
	{
		if (!primed) { semisSlew = P.semis; shMixS = (P.shPos != 0) ? P.shMix : 0.f; primed = true; }

		const float invSr = 1.f / sr;
		const float decayCoef = ntm::exp(-1.f / (0.001f * clampf(P.excDecayMs, 1.f, 5000.f) * sr));
		const float fMin = sr / (float)(kMaxDelay - 16);
		const float fMax = 0.2f * sr;
		// Feedback is a decay time: 5 ms .. ~17 s (T60), identical at every pitch. >= 98% self-oscillates.
		const float fbx = clampf(P.feedback, 0.f, 1.f);
		const float t60 = 0.005f * ntm::exp2(11.965784f * fbx);
		const bool selfOsc = fbx >= 0.98f;
		const float frTarget = P.freeze ? 1.f : 0.f;
		const bool swirlOn = P.swirl > 0.0005f;
		const float regenSamples = P.regenMs * 0.001f * sr;
		const float shLimit = 0.45f * sr;

		for (int i = 0; i < n; ++i)
		{
			semisSlew += 0.002f * (P.semis - semisSlew);
			frAmt += 0.002f * (frTarget - frAmt);
			shMixS += 0.0015f * (((P.shPos != 0) ? P.shMix : 0.f) - shMixS);

			float wL = wl.next(P.chaosHz, sr), wR = wr.next(P.chaosHz, sr), wM = wm.next(P.chaosHz * 0.7f, sr);

			float semi = semisSlew + (in.pitch ? in.pitch[i] * 12.f : 0.f);
			float centsL = 100.f * semi - P.spreadCents + wL * P.chaos * 25.f;
			float centsR = 100.f * semi + P.spreadCents + wR * P.chaos * 25.f;
			float f[2] = { clampf(kBaseHz * ntm::exp2(centsL / 1200.f), fMin, fMax),
			               clampf(kBaseHz * ntm::exp2(centsR / 1200.f), fMin, fMax) };

			// ---- swirl LFOs: per engine/channel wobble of the loop pitch, allpass sweep, stereo rotation
			float fE[2][2] = { { f[0], f[1] }, { f[0], f[1] } };
			SwirlCtl sw[2];
			if (swirlOn)
			{
				ph1 = wrap01(ph1 + P.swirlHz * invSr);
				ph2 = wrap01(ph2 + P.swirlHz * 1.37f * invSr);
				ph3 = wrap01(ph3 + P.swirlHz * 0.61f * invSr);
				float s1 = ntm::sin(kTwoPi * ph1), c1 = ntm::cos(kTwoPi * ph1);
				float s2 = ntm::sin(kTwoPi * ph2), c2 = ntm::cos(kTwoPi * ph2);
				float s3 = ntm::sin(kTwoPi * ph3), c3 = ntm::cos(kTwoPi * ph3);
				const float cc = P.swirl * 45.f;                       // cents of wobble
				const float aa = P.swirl * 0.7f;                       // allpass sweep depth
				const float cents[2][2] = { { cc * s1, cc * c1 }, { -cc * s1, -cc * c1 } };
				const float lfo2[2][2]  = { { s2, c2 }, { -s2, -c2 } };
				const float th[2] = { aa * s3, aa * c3 };              // rotation angle, radians
				for (int e = 0; e < 2; ++e)
				{
					sw[e].on = true;
					for (int c = 0; c < 2; ++c)
					{
						fE[e][c] = clampf(f[c] * ntm::exp2(cents[e][c] * (1.f / 1200.f)), fMin, fMax);
						sw[e].ap[c] = clampf(0.2f + aa * lfo2[e][c], -0.92f, 0.92f);
					}
					sw[e].rc = ntm::cos(th[e]); sw[e].rs = ntm::sin(th[e]);
				}
			}

			// ---- shifter oscillator
			ShiftCtl sh;
			const bool shActive = (P.shPos != 0) && (shMixS > 0.0005f);
			if (shActive)
			{
				float hz = P.shiftHz;
				if (in.linFM) hz += in.linFM[i] * P.linHzPerV;
				if (in.expFM) hz *= ntm::exp2(in.expFM[i] * P.expOctPerV);
				hz = clampf(hz, -shLimit, shLimit);
				shPh = wrap01(shPh + hz * invSr);
				sh.c = ntm::cos(kTwoPi * shPh); sh.s = ntm::sin(kTwoPi * shPh);
				sh.mix = shMixS; sh.mode = P.shMode;
				sh.loop = (P.shPos == 2);
			}

			if (in.gate)
			{
				bool hi = in.gate[i] > (gateHigh ? 0.5f : 1.5f);
				if (hi && !gateHigh) env = 1.f;
				gateHigh = hi;
			}
			env *= decayCoef;

			float t60m = t60 * ntm::exp2(wM * P.chaos * 0.6f);
			float g[2];
			for (int c = 0; c < 2; ++c)
			{
				float gc = selfOsc ? 1.02f : ntm::exp(-6.9078f / (f[c] * t60m));
				g[c] = gc + (1.f - gc) * frAmt;
			}
			float damp = clampf(P.damping + wM * P.chaos * 0.3f, 0.f, 1.f);

			// ---- excitation: internal oscillator (noise) + external audio into the loops
			float n0 = noise(), n1 = noise();
			float level = P.excLevel * P.excLevel;
			float e[2];
			for (int c = 0; c < 2; ++c)
			{
				float nz = c ? n1 : n0;
				if (P.pluckMode) e[c] = nz * env * level * 1.6f;
				else
				{
					// Level compensation so drone loudness stays roughly constant as decay time changes.
					// (Fitted by measurement: damping removes much of the noise energy at long decays,
					// so the ideal 1/sqrt(1-g^2) correction over-corrects.)
					float gc = clampf(g[c], 0.f, 0.999f);
					e[c] = nz * level * (0.5f + 0.6f * ntm::sqrt(1.f - gc * gc));
				}
			}
			const float xl = in.extL ? in.extL[i] : 0.f;
			const float xr = in.extR ? in.extR[i] : xl;
			float exc[2] = { e[0] + xl * P.extLevel, e[1] + xr * P.extLevel };

			if (shActive && P.shPos == 1)
			{
				float o[2];
				ss.run(exc, sh.c, sh.s, sh.mode, sh.mix, P.regen, P.regenRoute, regenSamples, o);
				exc[0] = o[0]; exc[1] = o[1];
			}

			float oa[2], ob[2];
			eng[0].tick(P.typeA, P.mutA, sr, fE[0], P, g, damp, frAmt, exc, sw[0], sh, oa);
			eng[1].tick(P.typeB, P.mutB, sr, fE[1], P, g, damp, frAmt, exc, sw[1], sh, ob);

			float l = oa[0] + P.morph * (ob[0] - oa[0]);
			float r = oa[1] + P.morph * (ob[1] - oa[1]);
			float mid = 0.5f * (l + r), side = 0.5f * (l - r) * P.width;
			float o[2] = { P.wet * ftanh((mid + side) * P.gain) + xl * P.dry * P.gain,
			               P.wet * ftanh((mid - side) * P.gain) + xr * P.dry * P.gain };

			if (shActive && P.shPos == 3)
			{
				float p[2];
				ss.run(o, sh.c, sh.s, sh.mode, sh.mix, P.regen, P.regenRoute, regenSamples, p);
				o[0] = p[0]; o[1] = p[1];
			}
			outL[i] = clampf(o[0], -2.f, 2.f);
			outR[i] = clampf(o[1], -2.f, 2.f);
		}
	}
};

} // namespace comb
