// lfo_core.h - six phase-shiftable, time-stretchable LFOs. Pure DSP, no API dependencies.
#pragma once
#include <stdint.h>
#include "nt_math.h"

namespace pl {

static const int kChannels = 6;
static const int kNumShapes = 20;

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// deterministic random value in [-1, 1) from an integer (so phase-shifted channels see the same random sequence)
inline float hashf(int32_t n)
{
	uint32_t h = (uint32_t)n * 2654435761u;
	h ^= h >> 15; h *= 2246822519u; h ^= h >> 13; h *= 3266489917u; h ^= h >> 16;
	return (float)(h >> 8) * (1.f / 8388608.f) - 1.f;
}

inline float smoothstep(float t) { return t * t * (3.f - 2.f * t); }
inline float frac(float x) { return x - ntm::floor(x); }
inline float tri(float x)   // starts at 0, rises: 0 -> 1 (x=.25) -> -1 (x=.75) -> 0
{
	return x < 0.25f ? 4.f * x : (x < 0.75f ? 2.f - 4.f * x : 4.f * x - 4.f);
}

// x in [0,1) (already skew-warped), c = cycle number.  Returns about [-1, 1].
inline float shape(int s, float x, int32_t c)
{
	const float kTwoPi = ntm::kTwoPi;
	switch (s)
	{
	default:
	case 0:  return ntm::sin(kTwoPi * x);                                    // Sine
	case 1:  return tri(x);                                                  // Triangle
	case 2:  return 2.f * x - 1.f;                                           // Ramp up
	case 3:  return 1.f - 2.f * x;                                           // Ramp down
	case 4:  return x < 0.5f ? 1.f : -1.f;                                   // Square (Skew = duty)
	case 5:  return hashf(c) + ( hashf(c + 1) - hashf(c) ) * smoothstep(x);  // Smooth random
	case 6:  return hashf(c);                                                // Stepped random
	case 7:  { float g = x * 4.f; g = g > 1.f ? 1.f : g;                      // Glide random (S&H with slew)
	           return hashf(c - 1) + ( hashf(c) - hashf(c - 1) ) * smoothstep(g); }
	case 8:  return ntm::floor(x * 8.f) * (2.f / 7.f) - 1.f;                 // Staircase (8 steps up)
	case 9:  return ntm::sin(1.5f * ntm::kPi * ntm::sin(kTwoPi * x));        // Folded sine
	case 10: return 2.f * ntm::fabs(ntm::sin(ntm::kPi * x)) - 1.f;           // Rectified sine
	case 11: { const float e5 = 0.0067379f;                                  // Pump (fast exp fall)
	           return 2.f * ( ntm::exp(-5.f * x) - e5 ) / ( 1.f - e5 ) - 1.f; }
	case 12: {                                                               // Bounce (ball: arcs shrink x0.6)
	           const float r = 0.6f;
	           float total = 0.f, d = 1.f;
	           for (int k = 0; k < 8; ++k) { total += d; d *= r; }
	           float pos = x * total, dk = 1.f, hk = 1.f;
	           for (int k = 0; k < 8; ++k)
	           {
	               if (pos < dk || k == 7) { float u = clampf(pos / dk, 0.f, 1.f); return 2.f * hk * 4.f * u * (1.f - u) - 1.f; }
	               pos -= dk; dk *= r; hk *= r * r;
	           }
	           return -1.f; }
	case 13: return ntm::sin(kTwoPi * ( 2.f * x + 6.f * x * x ));            // Chirp (2 -> 14 cycles)
	case 14: return ntm::sin(kTwoPi * x + 3.f * ntm::sin(3.f * kTwoPi * x)); // FM wobble
	case 15: { float v = ntm::sin(kTwoPi * x) + 0.6f * ntm::sin(kTwoPi * 3.f * x + 1.f)    // Wander (odd harmonics)
	                   + 0.4f * ntm::sin(kTwoPi * 5.f * x + 2.3f);
	           return clampf(v * 0.65f, -1.f, 1.f); }
	case 16: { float g = x * 4.f; float k = ntm::floor(g); float l = g - k;  // Ratchet (4 fading pulses)
	           float h = 1.f - 0.25f * k; return l < 0.5f ? 2.f * h - 1.f : -1.f; }
	case 17: return 2.f * frac(3.f * x) - 1.f;                               // Triple ramp
	case 18: { float g = x * 4.f; int32_t k = (int32_t)ntm::floor(g); float l = g - (float)k;   // Jitter (4 random glides)
	           float a = hashf(c * 4 + k - 1), b = hashf(c * 4 + k);
	           return a + ( b - a ) * smoothstep(l); }
	case 19: return tri(frac(5.f * x));                                      // Zigzag (5 quick triangles)
	}
}

// skew warp: moves the half-way point of the cycle to 'sk' (0.5 = no change)
inline float warp(float x, float sk)
{
	return x < sk ? 0.5f * x / sk : 0.5f + 0.5f * ( x - sk ) / ( 1.f - sk );
}

struct Params
{
	int   shape = 0;
	float rateHz = 1.f;          // base rate (before CV)
	float rateSpread = 0.f;      // -1..1: octaves of rate difference across the six (channel 6 = +spread octaves)
	float phase = 0.f;           // -1..1: 1 = channels evenly spaced around one full cycle
	float skew = 0.5f;           // 0.05..0.95
	float depth = 1.f;           // 0..1
	float offsetV = 0.f;
	int   polarity[kChannels] = { 0, 0, 0, 0, 0, 0 };   // per LFO: 0 bipolar, 1 unipolar +, 2 unipolar -
	float level[kChannels] = { 1.f, 1.f, 1.f, 1.f, 1.f, 1.f };   // per LFO output attenuation 0..1
	float voltScale = 5.f;       // full-scale volts
	int   clockIdx = 10;         // clock ratio step (0..20, see kClockNum/kClockDen) used while a clock is present
};

// clock ratios: output cycles per clock tick = num / den.  index 10 = x1
static const int kNumClockRatios = 21;
static const int kClockNum[kNumClockRatios] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,   1,   2, 3, 4, 5, 6, 8, 12, 16, 24, 32 };
static const int kClockDen[kNumClockRatios] = { 32, 24, 16, 12, 8, 6, 5, 4, 3, 2,   1,   1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

struct Inputs
{
	const float* rateCV = nullptr;    // 1 V/oct
	const float* phaseCV = nullptr;   // +5 V == +100% phase
	const float* reset = nullptr;     // rising edge through 1 V resets all phases
	const float* spreadCV = nullptr;  // added to Rate spread: 5 V == +100%
	const float* skewCV = nullptr;    // added to Skew: 5 V == +45% (so -5..+5 V sweeps the whole range)
	const float* clock = nullptr;     // clock: rising edges through 1 V; while present the Rate knob picks a ratio
	const float* shapeGate = nullptr; // each rising edge through 1 V steps to the next shape (wraps)
};

struct Bank
{
	float sr = 48000.f;
	float ph[kChannels];
	int32_t cyc[kChannels];
	float levelS[kChannels] = { 1.f, 1.f, 1.f, 1.f, 1.f, 1.f };
	float phaseS = 0.f, skewS = 0.5f, depthS = 1.f, offS = 0.f, spreadS = 0.f;
	bool primed = false, resetHigh = false, gateHigh = false;
	bool clkHigh = false;
	float sinceEdge = 0.f, period = 0.f;   // samples
	int edges = 0, clkN = 0, lastClockIdx = -1;
	int shapeOff = 0, lastShapeParam = -1, curShape = 0, prevShape = 0;
	float xf = 0.f;                  // shape crossfade (1 -> 0 over ~5 ms) after a shape change

	void init(float sampleRate)
	{
		sr = sampleRate;
		for (int i = 0; i < kChannels; ++i) { ph[i] = 0.f; cyc[i] = 0; }
		primed = false; resetHigh = false; gateHigh = false; clkHigh = false; sinceEdge = 0.f; period = 0.f; edges = 0; clkN = 0; lastClockIdx = -1; shapeOff = 0; lastShapeParam = -1; xf = 0.f;
	}

	// out: kChannels pointers, each n samples (volts)
	void process(const Params& P, const Inputs& in, float* const* out, int n)
	{
		if (!primed) { phaseS = P.phase; skewS = P.skew; depthS = P.depth; offS = P.offsetV; for (int c = 0; c < kChannels; ++c) levelS[c] = P.level[c]; spreadS = P.rateSpread; primed = true; }
		const float sm = 1.f - ntm::exp(-1.f / (0.01f * sr));
		const float invSr = 1.f / sr;
		const float xfStep = 1.f / (0.005f * sr);
		if (P.shape != lastShapeParam)                      // moving the Shape parameter makes it the new starting point
		{
			if (lastShapeParam >= 0) { prevShape = curShape; xf = 1.f; }
			lastShapeParam = P.shape; shapeOff = 0;
			curShape = ((P.shape % kNumShapes) + kNumShapes) % kNumShapes;
			if (!primed || xf == 0.f) prevShape = curShape;
		}
		for (int i = 0; i < n; ++i)
		{
			if (in.shapeGate)
			{
				bool hi = in.shapeGate[i] > ( gateHigh ? 0.5f : 1.f );
				if (hi && !gateHigh)
				{
					shapeOff = ( shapeOff + 1 ) % kNumShapes;
					prevShape = curShape; xf = 1.f;
					curShape = ( P.shape + shapeOff ) % kNumShapes;
				}
				gateHigh = hi;
			}
			const float sCV = in.spreadCV ? in.spreadCV[i] * 0.2f : 0.f;
			const float kCV = in.skewCV ? in.skewCV[i] * 0.09f : 0.f;
			phaseS  += ( P.phase - phaseS ) * sm;
			skewS   += ( clampf(P.skew + kCV, 0.02f, 0.98f) - skewS ) * sm;
			depthS  += ( P.depth - depthS ) * sm;
			offS    += ( P.offsetV - offS ) * sm;
			spreadS += ( clampf(P.rateSpread + sCV, -1.5f, 1.5f) - spreadS ) * sm;

			if (in.reset)
			{
				bool hi = in.reset[i] > 1.f;
				if (hi && !resetHigh) { for (int c = 0; c < kChannels; ++c) { ph[c] = 0.f; cyc[c] = 0; } clkN = 0; }
				resetHigh = hi;
			}
			// ---- clock follower: while a steady clock is present the Rate knob selects a multiplication / division
			bool clocked = false;
			const int ci = P.clockIdx < 0 ? 0 : (P.clockIdx >= kNumClockRatios ? kNumClockRatios - 1 : P.clockIdx);
			if (in.clock)
			{
				sinceEdge += 1.f;
				bool hi = in.clock[i] > ( clkHigh ? 0.5f : 1.f );
				if (hi && !clkHigh && sinceEdge > 0.0005f * sr)           // ignore edges closer than 0.5 ms
				{
					if (edges > 0 && sinceEdge < 10.f * sr)
					{
						float np = sinceEdge;
						period = ( period > 0.f && ntm::fabs(np - period) < 0.5f * period ) ? period + ( np - period ) * 0.5f : np;
					}
					bool fresh = edges < 2 || sinceEdge > 2.5f * period;
					edges = ( sinceEdge < 10.f * sr ) ? edges + 1 : 1;
					sinceEdge = 0.f;
					const int den = kClockDen[ci], num = kClockNum[ci];
					clkN = ( clkN + 1 ) % den;
					if (edges >= 2)
					{
						float T = (float)( ( clkN * num ) % den ) / (float)den;
						float e = T - ph[0]; e -= ntm::floor(e + 0.5f);        // shortest way round, in [-0.5, 0.5)
						float k = ( fresh || lastClockIdx != ci ) ? 1.f : 0.25f;
						if (fresh) k = 1.f;
						for (int c = 0; c < kChannels; ++c)
						{
							ph[c] += e * k;
							if (ph[c] >= 1.f) { ph[c] -= 1.f; cyc[c]++; } else if (ph[c] < 0.f) { ph[c] += 1.f; cyc[c]--; }
						}
					}
					lastClockIdx = ci;
				}
				clkHigh = hi;
				clocked = edges >= 2 && period > 0.f && sinceEdge < 2.5f * period;
				if (!clocked && sinceEdge > 2.5f * period + 0.5f * sr) edges = 0;
			}
			float cv = in.rateCV ? clampf(in.rateCV[i], -10.f, 10.f) : 0.f;
			float hz = P.rateHz * ntm::exp2(cv);
			if (clocked) hz = (float)kClockNum[ci] / (float)kClockDen[ci] * sr / period;
			float pcv = in.phaseCV ? in.phaseCV[i] * 0.2f : 0.f;       // 5 V -> 1.0
			float spread = clampf(phaseS + pcv, -4.f, 4.f);

			for (int c = 0; c < kChannels; ++c)
			{
				float mul = ntm::exp2(spreadS * ( (float)c - 2.5f ) * 0.4f);   // channels span +/-1 octave * spread across 6 (centered)
				float inc = clampf(hz * mul * invSr, 0.f, 0.25f);
				ph[c] += inc;
				if (ph[c] >= 1.f) { ph[c] -= 1.f; cyc[c]++; }
				float x = ph[c] + spread * (float)c * ( 1.f / kChannels );
				float fl = ntm::floor(x);
				int32_t cy = cyc[c] + (int32_t)fl;
				x -= fl;
				const float wx = warp(x, skewS);
				float s = shape(curShape, wx, cy);
				if (xf > 0.f) s = shape(prevShape, wx, cy) * xf + s * ( 1.f - xf );
				float v;
				if (P.polarity[c] == 0)      v = s * P.voltScale * depthS;
				else if (P.polarity[c] == 1) v = ( s + 1.f ) * P.voltScale * depthS;
				else                         v = -( s + 1.f ) * P.voltScale * depthS;
				levelS[c] += ( P.level[c] - levelS[c] ) * sm;
				out[c][i] = clampf(( v + offS ) * levelS[c], -10.f, 10.f);
			}
			if (xf > 0.f) { xf -= xfStep; if (xf < 0.f) xf = 0.f; }
		}
	}
};

}
