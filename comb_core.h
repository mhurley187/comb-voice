// comb_core.h - stereo comb-filter synth voice DSP (no disting NT API dependencies)
//
// Two "engines" (A and B) each pair a comb algorithm with a mutation that sits
// INSIDE the feedback loop. Each engine runs a left and a right delay loop,
// detuned against each other. A morph control crossfades A -> B.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

namespace comb {

static const int kMaxDelay = 8192;            // power of two; 8192 @ 48k -> ~5.9 Hz lowest pitch
static const int kMask = kMaxDelay - 1;
static const int kNumCombTypes = 6;
static const int kNumMutations = 7;
static const float kBaseHz = 261.6256f;       // 0 V = C4
static const float kDcR = 0.9995f;            // loop DC-blocker pole (~4 Hz corner at 48k)
static const size_t kDramFloats = 4 * kMaxDelay;   // 2 engines x 2 channels

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
	// exciter
	bool  pluckMode = false;    // false = drone (continuous noise), true = gate-triggered burst
	float excLevel = 0.7f;      // 0..1
	float excDecayMs = 120.f;
	float extLevel = 0.f;       // 0..1, external audio into the loops
	// output
	bool  freeze = false;
	float width = 1.f;          // 0..2
	float gain = 1.f;           // linear
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

struct DelayLine
{
	float* buf = nullptr;
	int w = 0;
	void init(float* b) { buf = b; w = 0; memset(buf, 0, sizeof(float) * kMaxDelay); }
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
	float apx[4] = { 0, 0, 0, 0 }, apy[4] = { 0, 0, 0, 0 };
	float hold = 0.f; int holdCnt = 0;
	float y = 0.f;
	void reset()
	{
		lp = ksPrev = dcx = dcy = hold = y = 0.f; holdCnt = 0;
		for (int i = 0; i < 4; ++i) apx[i] = apy[i] = 0.f;
	}
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
		case 2: return sinf(s * (1.f + 4.f * amt) * 1.5707963f);          // wavefold
		case 3: {                                                           // bit crush
			float levels = exp2f(11.f - 9.f * amt);
			return floorf(s * levels + 0.5f) / levels;
		}
		case 4: return clampf(s * (1.f + 4.f * amt) + 0.25f * amt, -1.f, 1.f);   // asymmetric hard clip
		case 5: {                                                           // sample-rate decimate
			int period = 1 + (int)(amt * 40.f);
			if (--L.holdCnt <= 0) { L.hold = s; L.holdCnt = period; }
			return L.hold;
		}
		case 6: return s + amt * fabsf(s) * 1.2f;                          // asymmetric rectify (keeps the fundamental)
		}
	}

	// Phase delay (in samples) at angular frequency w of the loop's filters. Using phase delay (not
	// group delay) is what makes the loop's fundamental land exactly on the requested pitch.
	static inline float pdLowpass(float a, float w)
	{
		float b = 1.f - a;
		return atan2f(b * sinf(w), 1.f - b * cosf(w)) / w;
	}
	static inline float pdDcBlock(float w)   // negative = phase lead
	{
		const float R = kDcR;
		float phi = 1.5707963f - 0.5f * w - atan2f(R * sinf(w), 1.f - R * cosf(w));
		return -phi / w;
	}
	static inline float pdAllpass(float g, float w)
	{
		float phi = atan2f(-sinf(w), g + cosf(w)) - atan2f(-g * sinf(w), 1.f + g * cosf(w));
		return -phi / w;
	}

	float comp[2] = { 0.f, 0.f };
	int compCnt = 0;

	// One sample. f[2] = per-channel pitch in Hz, exc[2] = excitation, out[2] = comb outputs.
	void tick(int type, int mut, float sr, const float f[2], const Params& P,
	          const float g[2], float damp, float fr, const float exc[2], float out[2])
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
			a[c] = 1.f - expf(-6.2831853f * fc / sr);
			a[c] = a[c] + (1.f - a[c]) * fr;                   // freeze: bypass damping
			a[c] = clampf(a[c], 0.05f, 1.f);

			if (refresh)
			{
				float w = 6.2831853f * f[c] / sr;
				float C = pdDcBlock(w);
				if (type == 2) C += 0.5f;                       // 2-point average = half a sample
				else           C += pdLowpass(a[c], w);
				if (type == 3) C += 4.f * pdAllpass(gd, w);
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

		for (int c = 0; c < 2; ++c)
		{
			Loop& L = ch[c];
			float y = L.y;
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
			case 5: s = 0.5f * (y + ch[1 - c].y); break;                // stereo cross-coupled
			}

			if (type != 2) { L.lp += a[c] * (s - L.lp); s = L.lp; }

			s *= g[c];
			s = mutate(L, s, mut, P.mutAmount);

			float dc = s - L.dcx + kDcR * L.dcy;                      // DC blocker
			L.dcx = s; L.dcy = dc;
			s = ftanh(dc);                                              // keeps self-oscillation bounded

			L.dl.write(exc[c] * (1.f - fr) + s);
			out[c] = y;
		}
	}
};

struct Voice
{
	float sr = 48000.f;
	Engine eng[2];
	Wander wl{ 11u }, wr{ 22u }, wm{ 33u };
	uint32_t noiseState = 0x12345678u;
	float env = 0.f;
	bool gateHigh = false;
	float frAmt = 0.f;
	float semisSlew = 0.f;
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
		env = 0.f; gateHigh = false; frAmt = 0.f; primed = false;
	}

	inline float noise()
	{
		noiseState ^= noiseState << 13; noiseState ^= noiseState >> 17; noiseState ^= noiseState << 5;
		return ((int32_t)noiseState) * (1.f / 2147483648.f);
	}

	// pitchV: 1 V/oct CV in volts (or null). gate: volts (or null). ext: audio, +/-1 (or null).
	void process(const Params& P, const float* pitchV, const float* gate, const float* ext,
	             float* outL, float* outR, int n)
	{
		if (!primed) { semisSlew = P.semis; primed = true; }

		const float decayCoef = expf(-1.f / (0.001f * clampf(P.excDecayMs, 1.f, 5000.f) * sr));
		const float fMin = sr / (float)(kMaxDelay - 16);
		const float fMax = 0.2f * sr;
		// Feedback is a decay time: 5 ms .. ~17 s (T60), identical at every pitch. >= 98% self-oscillates.
		const float fbx = clampf(P.feedback, 0.f, 1.f);
		const float t60 = 0.005f * powf(4000.f, fbx);
		const bool selfOsc = fbx >= 0.98f;
		const float frTarget = P.freeze ? 1.f : 0.f;

		for (int i = 0; i < n; ++i)
		{
			semisSlew += 0.002f * (P.semis - semisSlew);
			frAmt += 0.002f * (frTarget - frAmt);

			float wL = wl.next(P.chaosHz, sr), wR = wr.next(P.chaosHz, sr), wM = wm.next(P.chaosHz * 0.7f, sr);

			float semi = semisSlew + (pitchV ? pitchV[i] * 12.f : 0.f);
			float centsL = 100.f * semi - P.spreadCents + wL * P.chaos * 25.f;
			float centsR = 100.f * semi + P.spreadCents + wR * P.chaos * 25.f;
			float f[2] = { clampf(kBaseHz * exp2f(centsL / 1200.f), fMin, fMax),
			               clampf(kBaseHz * exp2f(centsR / 1200.f), fMin, fMax) };

			if (gate)
			{
				bool hi = gate[i] > (gateHigh ? 0.5f : 1.5f);
				if (hi && !gateHigh) env = 1.f;
				gateHigh = hi;
			}
			env *= decayCoef;

			float t60m = t60 * exp2f(wM * P.chaos * 0.6f);
			float g[2];
			for (int c = 0; c < 2; ++c)
			{
				float gc = selfOsc ? 1.02f : expf(-6.9078f / (f[c] * t60m));
				g[c] = gc + (1.f - gc) * frAmt;
			}
			float damp = clampf(P.damping + wM * P.chaos * 0.3f, 0.f, 1.f);

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
					e[c] = nz * level * (0.5f + 0.6f * sqrtf(1.f - gc * gc));
				}
			}
			float x = (ext ? ext[i] : 0.f) * P.extLevel;
			float exc[2] = { e[0] + x, e[1] + x };

			float oa[2], ob[2];
			eng[0].tick(P.typeA, P.mutA, sr, f, P, g, damp, frAmt, exc, oa);
			eng[1].tick(P.typeB, P.mutB, sr, f, P, g, damp, frAmt, exc, ob);

			float l = oa[0] + P.morph * (ob[0] - oa[0]);
			float r = oa[1] + P.morph * (ob[1] - oa[1]);
			float mid = 0.5f * (l + r), side = 0.5f * (l - r) * P.width;
			outL[i] = ftanh((mid + side) * P.gain);
			outR[i] = ftanh((mid - side) * P.gain);
		}
	}
};

} // namespace comb
