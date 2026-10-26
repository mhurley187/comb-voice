// jumbler_core.h - 6x6 rotating router core. The routing matrix is circulant: M[o][i] = c[(o - i) mod 6],
// where c = shift(rotIn) * radiate-kernel * shift(rotOut) (circular convolution). Signals pass straight through
// (volts in, volts out), so it works for audio and CV alike.
#pragma once
#include <stdint.h>
#include "nt_math.h"

namespace jb {

static const int kN = 6;
static const int kTick = 16;

struct Params
{
	float rotIn, rotOut;      // channels (0..6)
	float radiate;            // 0..1
	bool  quantIn, quantOut;
	float slewMs;
	float spinIn, spinOut;    // channels per second (signed)
	float cvRotIn, cvRotOut, cvRad;   // channels / volt, channels / volt, units / volt
};

struct Inputs
{
	const float* in[kN];                          // NULL = silence
	const float* cvRotIn; const float* cvRotOut; const float* cvRad;
};

inline float wrap6( float x ) { x -= 6.f * ntm::floor( x * ( 1.f / 6.f ) ); if ( x >= 6.f ) x -= 6.f; return x; }

struct Router
{
	float sr;
	float cur[kN], delta[kN];
	float accIn, accOut;
	int tickPos;
	bool first;

	void init( float sampleRate )
	{
		sr = sampleRate; accIn = accOut = 0.f; tickPos = 0; first = true;
		for ( int i = 0; i < kN; ++i ) { cur[i] = i == 0 ? 1.f : 0.f; delta[i] = 0.f; }
	}

	static void shiftKernel( float pos, bool quant, float* s )
	{
		for ( int i = 0; i < kN; ++i ) s[i] = 0.f;
		if ( quant ) pos = ntm::floor( pos + 0.5f );
		pos = wrap6( pos );
		int i0 = (int)pos; float f = pos - (float)i0;
		if ( i0 >= kN ) { i0 -= kN; }
		s[i0] += 1.f - f; s[( i0 + 1 ) % kN] += f;
	}

	static void radiateKernel( float r, float* k )
	{
		float w = 3.f * r, sum = 0.f;
		for ( int d = 0; d < kN; ++d )
		{
			int dist = d < kN - d ? d : kN - d;
			float g = 1.f - (float)dist / ( w + 1.f );
			k[d] = g > 0.f ? g : 0.f; sum += k[d];
		}
		float inv = 1.f / sum;
		for ( int d = 0; d < kN; ++d ) k[d] *= inv;
	}

	void retarget( const Params& P, float cvIn, float cvOut, float cvRad )
	{
		float r = P.radiate + cvRad * P.cvRad; r = r < 0.f ? 0.f : ( r > 1.f ? 1.f : r );
		float sIn[kN], sOut[kN], k[kN], t[kN], c[kN];
		shiftKernel( P.rotIn + cvIn * P.cvRotIn + accIn, P.quantIn, sIn );
		shiftKernel( P.rotOut + cvOut * P.cvRotOut + accOut, P.quantOut, sOut );
		radiateKernel( r, k );
		for ( int i = 0; i < kN; ++i ) t[i] = c[i] = 0.f;
		for ( int a = 0; a < kN; ++a ) for ( int b = 0; b < kN; ++b ) t[( a + b ) % kN] += sIn[a] * k[b];
		for ( int a = 0; a < kN; ++a ) for ( int b = 0; b < kN; ++b ) c[( a + b ) % kN] += t[a] * sOut[b];
		float ms = P.slewMs < 0.5f ? 0.5f : P.slewMs;
		float alpha = 1.f - ntm::exp( -(float)kTick / ( ms * 0.001f * sr ) );
		const float inv = 1.f / (float)kTick;
		for ( int i = 0; i < kN; ++i )
		{
			if ( first ) cur[i] = c[i];
			float nxt = cur[i] + alpha * ( c[i] - cur[i] );
			delta[i] = ( nxt - cur[i] ) * inv;
		}
		first = false;
	}

	void process( const Params& P, const Inputs& in, float* const* out, int n )
	{
		const float dtIn = P.spinIn / sr, dtOut = P.spinOut / sr;
		for ( int s = 0; s < n; ++s )
		{
			accIn = wrap6( accIn + dtIn ); accOut = wrap6( accOut + dtOut );
			if ( tickPos == 0 )
				retarget( P, in.cvRotIn ? in.cvRotIn[s] : 0.f, in.cvRotOut ? in.cvRotOut[s] : 0.f, in.cvRad ? in.cvRad[s] : 0.f );
			if ( ++tickPos >= kTick ) tickPos = 0;
			float x[kN];
			for ( int i = 0; i < kN; ++i ) x[i] = in.in[i] ? in.in[i][s] : 0.f;
			for ( int o = 0; o < kN; ++o )
			{
				float y = 0.f;
				for ( int i = 0; i < kN; ++i ) y += cur[( o - i + kN ) % kN] * x[i];
				out[o][s] = y;
			}
			for ( int i = 0; i < kN; ++i ) cur[i] += delta[i];
		}
	}
};

} // namespace jb
