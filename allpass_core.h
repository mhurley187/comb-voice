// allpass_core.h - stereo allpass "smear" filter core: up to 49 biquad allpass sections (98 poles) per channel.
#pragma once
#include <stdint.h>
#include "nt_math.h"

namespace ap {

static const int kMaxSections = 49;
static const int kTick = 16;               // control-rate interval (samples); coefficients ramp across it

struct Params
{
	float poles;        // 2..98 (fractional allowed; base value, CV added in core)
	float freqOct;      // octaves above 20 Hz (0..10)
	float res;          // -0.99..0.99 feedback
	float spreadOct;    // 0..6 octaves covered by the stages
	float bias;         // -1..1  (-1 peaks low, +1 peaks high)
	float q;            // 0.5..10 section sharpness
	float mix;          // 0..1
	float lfoHz;
	float lfoDepth;     // octaves
	float lfoPhase;     // 0..0.5 turns offset of R vs L
	float envAmt;       // octaves at full envelope (can be negative)
	float stereoOct;    // total L/R frequency offset in octaves
	float gain;         // linear
	// CV (volts), per-sample arrays or NULL
	float cvScale[7];
};

struct Inputs
{
	const float* l; const float* r;                                   // +/-1.0 internal audio (r may be NULL)
	const float* cvPoles; const float* cvFreq; const float* cvRes; const float* cvSpread;
	const float* cvBias; const float* cvQ; const float* cvMix;
};

inline float softclip( float x )   // Pade tanh
{
	if ( x > 3.f ) x = 3.f; else if ( x < -3.f ) x = -3.f;
	float x2 = x * x;
	return x * ( 27.f + x2 ) / ( 27.f + 9.f * x2 );
}

struct Section { float s1, s2, b0, b1, d0, d1; };

struct Filter
{
	float sr, invSr;
	Section sec[2][kMaxSections];
	float w[kMaxSections], dw[kMaxSections];     // blend weight per section (shared by channels)
	int act;                                     // sections currently processed
	// smoothed control values
	float sPoles, sFreq, sRes, sSpread, sBias, sQ, sMix;
	float lfoPh, env;
	float fbPrev[2], fbHpX[2], fbHpY[2];
	int tickPos;
	bool first;

	void init( float sampleRate )
	{
		sr = sampleRate; invSr = 1.f / sampleRate;
		for ( int c = 0; c < 2; ++c ) for ( int i = 0; i < kMaxSections; ++i ) { Section& s = sec[c][i]; s.s1 = s.s2 = 0; s.b0 = s.b1 = s.d0 = s.d1 = 0; }
		for ( int i = 0; i < kMaxSections; ++i ) w[i] = dw[i] = 0;
		act = 0; sPoles = 2; sFreq = 5; sRes = 0; sSpread = 2; sBias = 0; sQ = 0.7f; sMix = 0.5f;
		lfoPh = 0; env = 0;
		for ( int c = 0; c < 2; ++c ) fbPrev[c] = fbHpX[c] = fbHpY[c] = 0;
		tickPos = 0; first = true;
	}

	// per-tick: compute section targets and ramps
	void retarget( const Params& P, float poles, float freqOct, float spread, float bias, float q, float lfoVal[2], float envVal )
	{
		{ int a = 0; for ( int k = 0; k < kMaxSections; ++k ) if ( w[k] > 0.f ) a = k + 1; act = a; }
		float N = poles * 0.5f;
		if ( N < 1.f ) N = 1.f; if ( N > (float)kMaxSections ) N = (float)kMaxSections;
		int K = (int)N; if ( (float)K < N ) ++K;          // ceil
		float pl = ntm::exp2( -2.f * bias );               // exponent warp: bias>0 pushes stages high
		const float inv = 1.f / (float)kTick;
		const float maxF = 0.45f * sr;
		for ( int c = 0; c < 2; ++c )
		{
			float off = ( c == 0 ? -0.5f : 0.5f ) * P.stereoOct;
			float base = ntm::log2( 20.f ) + freqOct + off + lfoVal[c] * P.lfoDepth + envVal * P.envAmt;
			for ( int i = 0; i < K; ++i )
			{
				float u = ( (float)i + 0.5f ) / N; if ( u > 1.f ) u = 1.f;
				float up = ntm::exp2( pl * ntm::log2( u ) );
				float lf = base + spread * ( up - 0.5f );
				float f = ntm::exp2( lf );
				if ( f < 8.f ) f = 8.f; if ( f > maxF ) f = maxF;
				float w0 = ntm::kTwoPi * f * invSr;
				float sn = ntm::sin( w0 ), cs = ntm::cos( w0 );
				float alpha = sn / ( 2.f * q );
				float ia = 1.f / ( 1.f + alpha );
				float b0 = ( 1.f - alpha ) * ia, b1 = -2.f * cs * ia;
				Section& s = sec[c][i];
				if ( i >= act ) { s.b0 = b0; s.b1 = b1; s.s1 = s.s2 = 0; }
				s.d0 = ( b0 - s.b0 ) * inv; s.d1 = ( b1 - s.b1 ) * inv;
			}
		}
		for ( int i = 0; i < kMaxSections; ++i )
		{
			float tw = ( i + 1 <= (int)N ) ? 1.f : ( i < K ? N - (float)i : 0.f );
			if ( i >= act && tw > 0.f ) w[i] = 0.f;
			dw[i] = ( tw - w[i] ) * inv;
		}
		act = K > act ? K : act;
	}

	void process( const Params& P, const Inputs& in, float* outL, float* outR, int n )
	{
		const float lfoInc = P.lfoHz * invSr;
		const float envA = 1.f - ntm::exp( -1.f / ( 0.002f * sr ) ), envR = 1.f - ntm::exp( -1.f / ( 0.15f * sr ) );
		const float fbHpC = 1.f - 20.f * ntm::kTwoPi * invSr;
		static const float kMaxCv = 10.f;
		for ( int i = 0; i < n; ++i )
		{
			float xl = in.l ? in.l[i] : 0.f;
			float xr = in.r ? in.r[i] : xl;
			float rect = 0.5f * ( ntm::fabs( xl ) + ntm::fabs( xr ) );
			env += ( rect > env ? envA : envR ) * ( rect - env );
			lfoPh += lfoInc; if ( lfoPh >= 1.f ) lfoPh -= 1.f;

			if ( tickPos == 0 )
			{
				auto cv = []( const float* a, int idx ) { return a ? a[idx] : 0.f; };
				float cp = cv( in.cvPoles, i ), cf = cv( in.cvFreq, i ), cr = cv( in.cvRes, i ), cs = cv( in.cvSpread, i );
				float cb = cv( in.cvBias, i ), cq = cv( in.cvQ, i ), cm = cv( in.cvMix, i );
				(void)kMaxCv;
				float tPoles = P.poles + cp * P.cvScale[0];
				float tFreq  = P.freqOct + cf;
				float tRes   = P.res + cr * P.cvScale[2];
				float tSpread = P.spreadOct + cs * P.cvScale[3];
				float tBias  = P.bias + cb * P.cvScale[4];
				float tQ     = P.q + cq * P.cvScale[5];
				float tMix   = P.mix + cm * P.cvScale[6];
				if ( tPoles < 2.f ) tPoles = 2.f; if ( tPoles > 98.f ) tPoles = 98.f;
				if ( tRes < -0.99f ) tRes = -0.99f; if ( tRes > 0.99f ) tRes = 0.99f;
				if ( tSpread < 0.f ) tSpread = 0.f; if ( tSpread > 8.f ) tSpread = 8.f;
				if ( tBias < -1.f ) tBias = -1.f; if ( tBias > 1.f ) tBias = 1.f;
				if ( tQ < 0.4f ) tQ = 0.4f; if ( tQ > 12.f ) tQ = 12.f;
				if ( tMix < 0.f ) tMix = 0.f; if ( tMix > 1.f ) tMix = 1.f;
				if ( first ) { sPoles = tPoles; sFreq = tFreq; sRes = tRes; sSpread = tSpread; sBias = tBias; sQ = tQ; sMix = tMix; first = false; }
				const float kc = 0.25f;                       // ~ 5 ms smoothing at 16-sample tick
				sPoles += 0.12f * ( tPoles - sPoles );        // slower for stage count: ~ 20 ms
				sFreq += kc * ( tFreq - sFreq ); sRes += kc * ( tRes - sRes ); sSpread += kc * ( tSpread - sSpread );
				sBias += kc * ( tBias - sBias ); sQ += kc * ( tQ - sQ ); sMix += kc * ( tMix - sMix );
				float lv[2];
				lv[0] = ntm::sin( ntm::kTwoPi * lfoPh );
				float p2 = lfoPh + P.lfoPhase; if ( p2 >= 1.f ) p2 -= 1.f;
				lv[1] = ntm::sin( ntm::kTwoPi * p2 );
				float ev = env > 1.5f ? 1.5f : env;
				retarget( P, sPoles, sFreq, sSpread, sBias, sQ, lv, ev );
			}
			if ( ++tickPos >= kTick ) tickPos = 0;

			float xin[2] = { xl, xr };
			float yout[2];
			const float fb = sRes;
			for ( int c = 0; c < 2; ++c )
			{
				// feedback path: DC-blocked, soft-clipped previous output
				float hp = fbPrev[c] - fbHpX[c] + fbHpC * fbHpY[c];
				fbHpX[c] = fbPrev[c]; fbHpY[c] = hp;
				float x = xin[c] + fb * softclip( hp );
				for ( int k = 0; k < act; ++k )
				{
					Section& s = sec[c][k];
					float wk = w[k];
					s.b0 += s.d0; s.b1 += s.d1;
					if ( wk <= 0.f && dw[k] <= 0.f ) continue;
					float y = s.b0 * x + s.s1;
					s.s1 = s.b1 * ( x - y ) + s.s2;
					s.s2 = x - s.b0 * y;
					x = wk >= 1.f ? y : x + wk * ( y - x );
				}
				fbPrev[c] = x;
				yout[c] = x;
			}
			for ( int k = 0; k < act; ++k ) { w[k] += dw[k]; if ( w[k] > 1.f ) w[k] = 1.f; if ( w[k] < 0.f ) w[k] = 0.f; }

			float comp = 1.f - 0.6f * ntm::fabs( fb );
			float dry[2] = { xl, xr };
			float o[2];
			for ( int c = 0; c < 2; ++c )
			{
				float v = ( dry[c] * ( 1.f - sMix ) + yout[c] * comp * sMix ) * P.gain;
				o[c] = 2.f * softclip( v * 0.5f );
			}
			outL[i] = o[0]; outR[i] = o[1];
		}
	}
};

} // namespace ap
