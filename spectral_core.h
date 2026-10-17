// spectral_core.h - multitap delay with a real-time spectral resynthesis stage in its feedback loop.
// Pure DSP, no API dependencies.
//
//   input --+--> [stereo delay line] --> N taps (positions bunched by "Grouping") --> tap mix
//           ^                                                                         |
//           |                         +-----------------------------------------------+
//           |                         v
//           |        mono --> FFT analysis --> strongest partials --> sine oscillator bank (shift / stretch / scatter /
//           |                                                          smear / tilt / freeze) --> stereo resynthesis
//           |                         |
//           +-- damping <-- feedback <-+ (blend of plain taps and resynthesis)
#pragma once
#include <stdint.h>
#include "nt_math.h"

namespace sd {

static const int kDelayLen    = 131072;      // samples per channel (power of two): 2.7 s at 48 kHz
static const int kDelayMask   = kDelayLen - 1;
static const int kMaxN        = 2048;        // largest FFT
static const int kMaxPartials = 32;
static const int kMaxTaps     = 8;

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// deterministic pseudo-random in [-1, 1) from an integer
inline float hashf(uint32_t n)
{
	uint32_t h = n * 2654435761u;
	h ^= h >> 15; h *= 2246822519u; h ^= h >> 13; h *= 3266489917u; h ^= h >> 16;
	return (float)(h >> 8) * (1.f / 8388608.f) - 1.f;
}

// tanh-like soft clip (Pade), bounded to +/-1
inline float softclip(float x)
{
	if (x > 3.f) return 1.f;
	if (x < -3.f) return -1.f;
	float x2 = x * x;
	return x * ( 27.f + x2 ) / ( 27.f + 9.f * x2 );
}

// all big buffers live in DRAM
struct Big
{
	float dl[2][kDelayLen];
	float ring[kMaxN];                  // mono analysis input history
	float re[kMaxN], im[kMaxN];         // FFT work
	float pre[kMaxN / 2 + 1], pim[kMaxN / 2 + 1];   // previous frame (for phase-vocoder frequency estimates)
	float m2[kMaxN / 2 + 1];
	float win[2][kMaxN];                // Hann: [0] for N = 1024, [1] for N = 2048
	float twc[kMaxN / 2], tws[kMaxN / 2];
	uint32_t rev[kMaxN];
};

struct Params
{
	// delay
	float timeMs = 250.f;       // total tap span
	int   taps = 4;             // 1..8
	float group = 0.f;          // -1 (taps bunch near the END) .. +1 (taps bunch near the START)
	float slope = 0.3f;         // 0..1: level fall-off across the taps
	float pan = 0.5f;           // 0..1: taps alternate left/right
	float feedback = 0.5f;      // 0..1.1
	float dampHz = 12000.f;
	// spectral
	int   quality = 1;          // 0: 1024-point (fast), 1: 2048-point (fine)
	int   partials = 16;        // 1..32
	float shiftSemis = 0.f;
	float stretch = 0.f;        // -1..1 inharmonic stretch
	float scatterCents = 0.f;   // 0..100 random detune per partial
	float smear = 0.f;          // 0..0.95 amplitude smoothing across frames (blur / sustain)
	float tiltDbOct = 0.f;      // spectral tilt around 1 kHz
	bool  freeze = false;
	float resFb = 1.f;          // 0..1 resynthesis share of the feedback signal
	float resOut = 0.5f;        // 0..1 resynthesis share of the wet output
	float width = 0.6f;         // 0..1 stereo spread of the partials
	// output
	float mix = 0.5f;
	float gain = 1.f;
};

struct Inputs
{
	const float* l = nullptr;        // +/-1 == full scale
	const float* r = nullptr;        // if null, l is used
	const float* timeCV = nullptr;   // 1 V/oct (time doubles per volt)
	const float* groupCV = nullptr;  // 5 V == +100%
	const float* shiftCV = nullptr;  // 1 V == 12 semitones
	const float* freezeGate = nullptr;
};

// ---------------------------------------------------------------------------------------------
// Spectral analysis / resynthesis
// ---------------------------------------------------------------------------------------------
struct Spectral
{
	Big* B = nullptr;
	float sr = 48000.f, invSr = 1.f / 48000.f;

	int wr = 0;
	int N = 2048, logN = 11, hop = 512;
	int hopCount = 0, stage = -1, sliceTimer = 0, sliceEvery = 30;
	bool havePrev = false;
	int quality = -1;

	// oscillator bank
	float vf[kMaxPartials], va[kMaxPartials], vtf[kMaxPartials], vta[kMaxPartials];
	float vdf[kMaxPartials], vda[kMaxPartials], vph[kMaxPartials];
	float vscat[kMaxPartials], gl[kMaxPartials], gr[kMaxPartials];
	int rampLeft = 0;
	float shiftS = 0.f, lastWidth = -1.f, lastScatter = -1.f;

	void init(Big* big, float sampleRate)
	{
		B = big; sr = sampleRate; invSr = 1.f / sr;
		for (int i = 0; i < kMaxN; ++i) { B->ring[i] = 0.f; B->re[i] = B->im[i] = 0.f; }
		for (int i = 0; i <= kMaxN / 2; ++i) { B->pre[i] = B->pim[i] = B->m2[i] = 0.f; }
		for (int i = 0; i < kMaxN / 2; ++i)
		{
			float a = ntm::kTwoPi * (float)i / (float)kMaxN;
			B->twc[i] = ntm::cos(a); B->tws[i] = -ntm::sin(a);
		}
		for (int w = 0; w < 2; ++w)
		{
			int n = w == 0 ? 1024 : 2048;
			for (int i = 0; i < kMaxN; ++i) B->win[w][i] = 0.f;
			for (int i = 0; i < n; ++i) B->win[w][i] = 0.5f - 0.5f * ntm::cos(ntm::kTwoPi * (float)i / (float)n);
		}
		for (int i = 0; i < kMaxN; ++i)
		{
			uint32_t r = 0, x = (uint32_t)i;
			for (int b = 0; b < 11; ++b) { r = (r << 1) | (x & 1u); x >>= 1; }
			B->rev[i] = r;
		}
		for (int v = 0; v < kMaxPartials; ++v)
		{
			vf[v] = va[v] = vtf[v] = vta[v] = vdf[v] = vda[v] = 0.f;
			vph[v] = 0.5f * ( hashf(100u + (uint32_t)v) + 1.f );
			vscat[v] = 1.f; gl[v] = gr[v] = 1.f;
		}
		rampLeft = 0; shiftS = 0.f; lastWidth = -1.f; lastScatter = -1.f; wr = 0; hopCount = 0; stage = -1; havePrev = false; quality = -1;
	}

	void setQuality(int q)
	{
		quality = q;
		// both sizes update every 256 samples (187.5 Hz): the longer window gives finer frequency resolution
		// without slowing the frame rate (a slower frame rate sounds like a ~94 Hz buzz)
		N = q == 0 ? 1024 : 2048; logN = q == 0 ? 10 : 11; hop = 256;
		sliceEvery = ( hop - 8 ) / ( logN + 2 ); if (sliceEvery < 1) sliceEvery = 1;
		hopCount = 0; stage = -1; havePrev = false;
	}

	void startFrame()
	{
		const float* w = B->win[N == 1024 ? 0 : 1];
		const int shiftRev = ( N == 1024 ) ? 1 : 0;
		for (int n = 0; n < N; ++n)
		{
			uint32_t idx = (uint32_t)(wr - N + n) & (uint32_t)(kMaxN - 1);
			uint32_t r = B->rev[n] >> shiftRev;
			B->re[r] = B->ring[idx] * w[n];
			B->im[r] = 0.f;
		}
		stage = 0; sliceTimer = 0;
	}

	void fftStage(int s)
	{
		const int len = 2 << s, half = len >> 1, stride = kMaxN / len;
		float* re = B->re; float* im = B->im;
		for (int i = 0; i < N; i += len)
			for (int j = 0; j < half; ++j)
			{
				const float wc = B->twc[j * stride], ws = B->tws[j * stride];
				const int a = i + j, b = a + half;
				const float tr = re[b] * wc - im[b] * ws;
				const float ti = re[b] * ws + im[b] * wc;
				re[b] = re[a] - tr; im[b] = im[a] - ti;
				re[a] += tr;        im[a] += ti;
			}
	}

	// pick the strongest spectral peaks and retarget the oscillator bank
	void finishFrame(const Params& P)
	{
		const int K = P.partials < 1 ? 1 : (P.partials > kMaxPartials ? kMaxPartials : P.partials);
		const int half = N / 2;
		const float binHz = sr / (float)N;
		float* re = B->re; float* im = B->im; float* m2 = B->m2;
		for (int k = 0; k <= half; ++k) m2[k] = re[k] * re[k] + im[k] * im[k];

		const float thr = 1e-4f * 0.25f * (float)N;           // amplitude 1e-4 (-80 dB)
		const float thr2 = thr * thr;
		int pk[kMaxPartials]; float pm[kMaxPartials]; int cnt = 0;
		const int kHi = (int)(0.45f * (float)N);
		for (int k = 2; k < kHi && k < half - 1; ++k)
		{
			const float m = m2[k];
			if (m > thr2 && m > m2[k - 1] && m >= m2[k + 1])
			{
				if (cnt < K || m > pm[cnt - 1])
				{
					int p = cnt < K ? cnt : K - 1;
					while (p > 0 && pm[p - 1] < m) { pk[p] = pk[p - 1]; pm[p] = pm[p - 1]; --p; }
					pk[p] = k; pm[p] = m;
					if (cnt < K) ++cnt;
				}
			}
		}

		bool claimed[kMaxPartials];
		for (int v = 0; v < kMaxPartials; ++v) claimed[v] = false;
		const float tol = 1.2f * binHz;
		const float sm = P.smear;
		for (int p = 0; p < cnt; ++p)
		{
			const int k = pk[p];
			float delta = 0.f;
			if (havePrev)
			{
				float ph = ntm::atan2(im[k], re[k]);
				float pp = ntm::atan2(B->pim[k], B->pre[k]);
				float d = ph - pp - ntm::kTwoPi * (float)k * (float)hop / (float)N;   // expected phase advance over one hop
				d -= ntm::kTwoPi * ntm::floor(d * 0.15915494f + 0.5f);
				delta = clampf(d * (float)N / ( ntm::kTwoPi * (float)hop ), -0.75f, 0.75f);   // deviation in bins
			}
			const float f = ( (float)k + delta ) * binHz;
			float amp = ntm::sqrt(pm[p]) * 4.f / (float)N;
			if (P.tiltDbOct != 0.f) amp *= ntm::exp2(P.tiltDbOct * 0.16609640f * ntm::log2(f * 0.001f));

			int best = -1; float bd = tol;
			for (int v = 0; v < K; ++v)
			{
				if (claimed[v] || ( vta[v] <= 0.f && va[v] <= 1e-5f )) continue;
				float d = ntm::fabs(vtf[v] - f);
				if (d < bd) { bd = d; best = v; }
			}
			if (best < 0)
			{
				for (int v = 0; v < K; ++v)
					if (!claimed[v] && vta[v] <= 1e-6f && va[v] <= 1e-5f) { best = v; vf[v] = vtf[v] = f; va[v] = 0.f; vta[v] = 0.f;
						break; }
			}
			if (best < 0) continue;
			claimed[best] = true;
			vtf[best] = f;
			vta[best] = vta[best] * sm + amp * ( 1.f - sm );
		}
		for (int v = 0; v < kMaxPartials; ++v)
		{
			if (v >= K) { vta[v] = 0.f; continue; }
			if (!claimed[v]) { vta[v] *= sm; if (vta[v] < 1e-5f) vta[v] = 0.f; }
		}
		// ramp every voice to its new targets across one hop
		rampLeft = hop;
		const float inv = 1.f / (float)hop;
		for (int v = 0; v < kMaxPartials; ++v) { vda[v] = ( vta[v] - va[v] ) * inv; vdf[v] = ( vtf[v] - vf[v] ) * inv; }

		for (int k = 0; k <= half; ++k) { B->pre[k] = re[k]; B->pim[k] = im[k]; }
		havePrev = true;
	}

	void updateStatic(const Params& P)
	{
		if (P.width != lastWidth)
		{
			lastWidth = P.width;
			for (int v = 0; v < kMaxPartials; ++v)
			{
				float pan = hashf(500u + (uint32_t)v * 7u) * P.width;               // -width .. +width
				float a = ( pan + 1.f ) * 0.78539816f;                              // 0 .. pi/2
				gl[v] = 1.41421356f * ntm::cos(a); gr[v] = 1.41421356f * ntm::sin(a);
			}
		}
		if (P.scatterCents != lastScatter)
		{
			lastScatter = P.scatterCents;
			for (int v = 0; v < kMaxPartials; ++v) vscat[v] = ntm::exp2(hashf(900u + (uint32_t)v * 13u) * P.scatterCents * ( 1.f / 1200.f ));
		}
	}

	// one sample in, stereo resynthesis out
	void tick(float in, const Params& P, float shiftTarget, bool frozen, float& outL, float& outR)
	{
		if (P.quality != quality) setQuality(P.quality);
		B->ring[wr] = in;
		wr = ( wr + 1 ) & ( kMaxN - 1 );

		if (!frozen)
		{
			if (++hopCount >= hop) { hopCount = 0; if (stage < 0) startFrame(); }
			if (stage >= 0 && ++sliceTimer >= sliceEvery)
			{
				sliceTimer = 0;
				if (stage < logN) { fftStage(stage); ++stage; }
				else { finishFrame(P); stage = -1; }
			}
		}
		else { havePrev = false; stage = -1; hopCount = 0; }

		// ramp amplitudes / frequencies
		if (rampLeft > 0)
		{
			for (int v = 0; v < kMaxPartials; ++v) { va[v] += vda[v]; vf[v] += vdf[v]; }
			if (--rampLeft == 0) for (int v = 0; v < kMaxPartials; ++v) { va[v] = vta[v]; vf[v] = vtf[v]; }
		}

		shiftS += ( shiftTarget - shiftS ) * 0.002f;
		const float ratio = ntm::exp2(shiftS * ( 1.f / 12.f ));
		const float nyq = 0.47f * sr;
		float oL = 0.f, oR = 0.f;
		for (int v = 0; v < kMaxPartials; ++v)
		{
			const float a = va[v];
			if (a <= 1e-7f) continue;
			const float f0 = vf[v];
			const float f = f0 * ratio * ( 1.f + P.stretch * f0 * 0.0005f ) * vscat[v];
			if (f <= 0.f || f > nyq) continue;
			float ph = vph[v] + f * invSr;
			ph -= ntm::floor(ph);
			vph[v] = ph;
			const float s = a * ntm::sin(ntm::kTwoPi * ph);
			oL += s * gl[v]; oR += s * gr[v];
		}
		outL = oL; outR = oR;
	}
};

// ---------------------------------------------------------------------------------------------
// The whole effect
// ---------------------------------------------------------------------------------------------
struct Effect
{
	Big* B = nullptr;
	Spectral spec;
	float sr = 48000.f;
	int wr = 0;
	float timeS = 12000.f, groupS = 0.f, groupS2 = 0.f, shapeTaps[kMaxTaps], level[kMaxTaps], tl[kMaxTaps], tr_[kMaxTaps];
	float frac[kMaxTaps], logu[kMaxTaps];
	float outNorm = 1.f, fbNorm = 1.f, mixS = 0.5f, fbS = 0.f, resFbS = 1.f, resOutS = 0.5f;
	float lpL = 0.f, lpR = 0.f, dcxL = 0.f, dcyL = 0.f, dcxR = 0.f, dcyR = 0.f;
	int blockCount = 0, lastTaps = -1;
	float lastSlope = -1.f, lastPan = -1.f;
	bool primed = false;

	void init(Big* big, float sampleRate)
	{
		B = big; sr = sampleRate;
		for (int c = 0; c < 2; ++c) for (int i = 0; i < kDelayLen; ++i) B->dl[c][i] = 0.f;
		spec.init(big, sampleRate);
		wr = 0; primed = false; lastTaps = -1; lastSlope = -1.f; lastPan = -1.f;
		lpL = lpR = dcxL = dcyL = dcxR = dcyR = 0.f; blockCount = 0;
	}

	static inline float hermite(const float* b, float idx)
	{
		float fl = ntm::floor(idx);
		int i1 = (int)fl;
		float mu = idx - fl;
		float y0 = b[( i1 - 1 ) & kDelayMask], y1 = b[i1 & kDelayMask], y2 = b[( i1 + 1 ) & kDelayMask], y3 = b[( i1 + 2 ) & kDelayMask];
		float c1 = 0.5f * ( y2 - y0 );
		float c2 = y0 - 2.5f * y1 + 2.f * y2 - 0.5f * y3;
		float c3 = 0.5f * ( y3 - y0 ) + 1.5f * ( y1 - y2 );
		return ( ( c3 * mu + c2 ) * mu + c1 ) * mu + y1;
	}

	void updateTaps(const Params& P)
	{
		const int n = P.taps < 1 ? 1 : (P.taps > kMaxTaps ? kMaxTaps : P.taps);
		if (n != lastTaps || P.slope != lastSlope || P.pan != lastPan)
		{
			lastTaps = n; lastSlope = P.slope; lastPan = P.pan;
			float s1 = 0.f, s2 = 0.f;
			for (int t = 0; t < kMaxTaps; ++t)
			{
				if (t >= n) { level[t] = 0.f; tl[t] = tr_[t] = 0.f; logu[t] = 0.f; continue; }
				const float u = (float)( t + 1 ) / (float)n;
				logu[t] = ntm::log2(u);
				level[t] = ntm::exp2(-P.slope * 4.f * u);
				s1 += level[t]; s2 += level[t] * level[t];
				const float p = ( ( t & 1 ) ? 1.f : -1.f ) * P.pan;
				const float a = ( p + 1.f ) * 0.78539816f;
				tl[t] = 1.41421356f * ntm::cos(a); tr_[t] = 1.41421356f * ntm::sin(a);
			}
			outNorm = 1.f / ntm::sqrt(s2);
			fbNorm = 1.f / s1;
		}
	}

	void process(const Params& P, const Inputs& in, float* outL, float* outR, int n)
	{
		if (!primed)
		{
			timeS = clampf(P.timeMs * ( in.timeCV ? ntm::exp2(clampf(in.timeCV[0], -8.f, 8.f)) : 1.f ) * 0.001f * sr, 8.f, (float)( kDelayLen - 8 ));
			groupS = groupS2 = clampf(P.group + ( in.groupCV ? in.groupCV[0] * 0.2f : 0.f ), -1.f, 1.f); 			mixS = P.mix; fbS = P.feedback; resFbS = P.resFb; resOutS = P.resOut; primed = true;
		}
		spec.updateStatic(P);
		updateTaps(P);
		const int nt = lastTaps;
		const float smT = 1.f - ntm::exp(-1.f / ( 0.05f * sr ));
		const float smP = 1.f - ntm::exp(-1.f / ( 0.02f * sr ));
		const float smG = 1.f - ntm::exp(-1.f / ( 0.04f * sr ));
		const float lpA = 1.f - ntm::exp(-ntm::kTwoPi * clampf(P.dampHz, 50.f, 0.45f * sr) / sr);
		const float maxSamples = (float)( kDelayLen - 8 );

		for (int i = 0; i < n; ++i)
		{
			const float inL = in.l ? in.l[i] : 0.f;
			const float inR = in.r ? in.r[i] : inL;

			// ---- parameters with CV
			float timeMs = P.timeMs;
			if (in.timeCV) timeMs *= ntm::exp2(clampf(in.timeCV[i], -8.f, 8.f));
			float tTarget = clampf(timeMs * 0.001f * sr, 8.f, maxSamples);
			timeS += ( tTarget - timeS ) * smT;
			float gT = P.group + ( in.groupCV ? in.groupCV[i] * 0.2f : 0.f );
			groupS += ( clampf(gT, -1.f, 1.f) - groupS ) * smG;       // two cascaded smoothers: continuous velocity, no Doppler clicks
			groupS2 += ( groupS - groupS2 ) * smG;
			mixS += ( P.mix - mixS ) * smP; fbS += ( P.feedback - fbS ) * smP;
			resFbS += ( P.resFb - resFbS ) * smP; resOutS += ( P.resOut - resOutS ) * smP;

			// tap positions are recomputed every sample: stepping them in blocks makes the moving read heads zipper
			const float g = ntm::exp2(groupS2 * 2.f);     // exponent 0.25 .. 4
			for (int t = 0; t < nt; ++t) frac[t] = ntm::exp2(g * logu[t]);

			// ---- read taps
			float tapL = 0.f, tapR = 0.f;
			for (int t = 0; t < nt; ++t)
			{
				float d = timeS * frac[t];
				if (d < 3.f) d = 3.f;
				const float idx = (float)wr - d;
				const float sL = hermite(B->dl[0], idx), sR = hermite(B->dl[1], idx);
				tapL += level[t] * tl[t] * sL;
				tapR += level[t] * tr_[t] * sR;
			}

			// ---- spectral resynthesis of the (feedback-normalised) tap mix
			const float fbTapL = tapL * fbNorm, fbTapR = tapR * fbNorm;
			float shiftT = P.shiftSemis + ( in.shiftCV ? in.shiftCV[i] * 12.f : 0.f );
			bool frozen = P.freeze || ( in.freezeGate && in.freezeGate[i] > 1.f );
			float rL, rR;
			spec.tick(0.5f * ( fbTapL + fbTapR ), P, clampf(shiftT, -48.f, 48.f), frozen, rL, rR);

			// ---- feedback path
			float fL = fbTapL * ( 1.f - resFbS ) + rL * resFbS;
			float fR = fbTapR * ( 1.f - resFbS ) + rR * resFbS;
			lpL += ( fL - lpL ) * lpA; lpR += ( fR - lpR ) * lpA;
			float xL = inL + fbS * lpL, xR = inR + fbS * lpR;
			float yL = xL - dcxL + 0.995f * dcyL; dcxL = xL; dcyL = yL;      // DC blocker
			float yR = xR - dcxR + 0.995f * dcyR; dcxR = xR; dcyR = yR;
			B->dl[0][wr] = 2.f * softclip(yL * 0.5f);      // gentle saturation keeps the loop bounded (+/-2)
			B->dl[1][wr] = 2.f * softclip(yR * 0.5f);
			wr = ( wr + 1 ) & kDelayMask;

			// ---- output
			const float k = outNorm / fbNorm;
			const float wL = tapL * outNorm * ( 1.f - resOutS ) + rL * k * resOutS;
			const float wR = tapR * outNorm * ( 1.f - resOutS ) + rR * k * resOutS;
			// output limiter: transparent for normal levels, never beyond +/-2 (== +/-10 V on the bus)
			outL[i] = 2.f * softclip(( inL * ( 1.f - mixS ) + wL * mixS ) * P.gain * 0.5f);
			outR[i] = 2.f * softclip(( inR * ( 1.f - mixS ) + wR * mixS ) * P.gain * 0.5f);
		}
	}
};

}
