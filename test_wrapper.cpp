// Host test: drive the plugin through its factory with mock buses and check parameter/bus wiring.
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <distingnt/api.h>
_NT_globals NT_globals = { 48000.f, 24, nullptr };

int main()
{
	const _NT_factory* f = (const _NT_factory*)pluginEntry( kNT_selector_factoryInfo, 0 );
	if ( !f || pluginEntry( kNT_selector_numFactories, 0 ) != 1 ) { puts("FAIL: factory"); return 1; }

	_NT_algorithmRequirements req = {};
	f->calculateRequirements( req, nullptr );
	std::vector<uint8_t> sram( req.sram + 64 ), dram( req.dram + 64 );
	_NT_algorithmMemoryPtrs ptrs = { sram.data(), dram.data(), nullptr, nullptr };
	_NT_algorithm* alg = f->construct( ptrs, req, nullptr );

	std::vector<int16_t> v( req.numParameters );
	for ( unsigned i = 0; i < req.numParameters; ++i ) v[i] = alg->parameters[i].def;
	alg->v = v.data();
	printf( "params=%u sram=%u dram=%u\n", req.numParameters, req.sram, req.dram );

	// bus layout: 28 buses x 24 frames. pitch CV on bus 1 (default), gate on bus 2 (set below), outs 13/14 (defaults)
	const int N = 24, blocks = 48000 / N;
	std::vector<float> bus( 28 * N );
	v[1] = 2;                          // gate in = bus 2
	v[24 - 24 + 21] = 1;               // (index 21 = Exciter mode) -> Pluck
	std::vector<float> outL, outR;
	for ( int b = 0; b < blocks; ++b )
	{
		std::fill( bus.begin(), bus.end(), 0.f );
		for ( int i = 0; i < N; ++i ) { bus[0 * N + i] = 1.0f;                        // 1 V pitch CV -> C5 (523.25 Hz)
		                                bus[1 * N + i] = ( b == 0 && i < 12 ) ? 5.f : 0.f; }   // trigger at start
		f->step( alg, bus.data(), N / 4 );
		for ( int i = 0; i < N; ++i ) { outL.push_back( bus[12 * N + i] ); outR.push_back( bus[13 * N + i] ); }
	}

	// find period of left output by autocorrelation between 0.1 s and 0.9 s
	int s0 = 4800, len = 38400; double best = -1; int bestLag = 0;
	for ( int lag = 40; lag < 400; ++lag )
	{
		double acc = 0; for ( int i = 0; i < len - 400; ++i ) acc += outL[s0 + i] * outL[s0 + i + lag];
		if ( acc > best ) { best = acc; bestLag = lag; }
	}
	float peak = 0, rms = 0; for ( int i = s0; i < s0 + len; ++i ) { peak = fmaxf( peak, fabsf( outL[i] ) ); rms += outL[i] * outL[i]; }
	rms = sqrtf( rms / len );
	printf( "period lag=%d samples -> %.1f Hz (expect ~523.3)  peak=%.2f V rms=%.3f V\n", bestLag, 48000.0 / bestLag, peak, rms );
	bool diff = false; for ( int i = s0; i < s0 + len; ++i ) if ( outL[i] != outR[i] ) diff = true;
	printf( "L/R differ (spread works): %s\n", diff ? "yes" : "NO" );
	bool finite = true; for ( float x : outL ) if ( !std::isfinite( x ) ) finite = false;
	puts( finite && peak > 0.05f && peak < 5.01f && fabs( 48000.0 / bestLag - 523.25 ) < 15 && diff ? "WRAPPER TEST: PASS" : "WRAPPER TEST: FAIL" );
	return 0;
}
