// jumbler.cpp - "Jumbler": 6-in / 6-out router with CV-controlled input rotation, output rotation and radiate,
// for the Expert Sleepers disting NT. DSP lives in jumbler_core.h.
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <new>
#include <distingnt/api.h>
#include "jumbler_core.h"

#if defined(__arm__) || defined(CV_PROVIDE_LIBC)
extern "C" {
__attribute__((used, optimize("no-tree-loop-distribute-patterns")))
void* memset( void* dst, int c, size_t n ) { uint8_t* d = (uint8_t*)dst; while ( n-- ) *d++ = (uint8_t)c; return dst; }
__attribute__((used, optimize("no-tree-loop-distribute-patterns")))
void* memcpy( void* dst, const void* src, size_t n ) { uint8_t* d = (uint8_t*)dst; const uint8_t* s = (const uint8_t*)src; while ( n-- ) *d++ = *s++; return dst; }
}
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) ((int)(sizeof(a) / sizeof((a)[0])))
#endif

static const int kChunk = 64;

enum
{
	kParamIn1, kParamIn2, kParamIn3, kParamIn4, kParamIn5, kParamIn6,
	kParamRotInCV, kParamRotOutCV, kParamRadCV,
	kParamOut1, kParamOut2, kParamOut3, kParamOut4, kParamOut5, kParamOut6, kParamOutMode,

	kParamRotIn, kParamRotOut, kParamRadiate, kParamQuantIn, kParamQuantOut, kParamSlew, kParamSpinIn, kParamSpinOut,

	kNumParams
};

static const char* const enumSmoothStep[] = { "Smooth", "Stepped" };

static const _NT_parameter parameters[] = {
	NT_PARAMETER_CV_INPUT( "Input 1", 1, 1 )
	NT_PARAMETER_CV_INPUT( "Input 2", 1, 2 )
	NT_PARAMETER_CV_INPUT( "Input 3", 1, 3 )
	NT_PARAMETER_CV_INPUT( "Input 4", 1, 4 )
	NT_PARAMETER_CV_INPUT( "Input 5", 1, 5 )
	NT_PARAMETER_CV_INPUT( "Input 6", 1, 6 )
	NT_PARAMETER_CV_INPUT( "Rotate in CV", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Rotate out CV", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Radiate CV", 0, 0 )
	NT_PARAMETER_CV_OUTPUT( "Output 1", 1, 13 )
	NT_PARAMETER_CV_OUTPUT( "Output 2", 1, 14 )
	NT_PARAMETER_CV_OUTPUT( "Output 3", 1, 15 )
	NT_PARAMETER_CV_OUTPUT( "Output 4", 1, 16 )
	NT_PARAMETER_CV_OUTPUT( "Output 5", 1, 17 )
	NT_PARAMETER_CV_OUTPUT( "Output 6", 1, 18 )
	NT_PARAMETER_OUTPUT_MODE( "Output mode" )

	{ .name = "Rotate in",  .min = 0,    .max = 60,   .def = 0,  .unit = kNT_unitNone,    .scaling = kNT_scaling10,  .enumStrings = NULL },
	{ .name = "Rotate out", .min = 0,    .max = 60,   .def = 0,  .unit = kNT_unitNone,    .scaling = kNT_scaling10,  .enumStrings = NULL },
	{ .name = "Radiate",    .min = 0,    .max = 100,  .def = 0,  .unit = kNT_unitPercent, .scaling = 0,              .enumStrings = NULL },
	{ .name = "In motion",  .min = 0,    .max = 1,    .def = 0,  .unit = kNT_unitEnum,    .scaling = 0,              .enumStrings = enumSmoothStep },
	{ .name = "Out motion", .min = 0,    .max = 1,    .def = 0,  .unit = kNT_unitEnum,    .scaling = 0,              .enumStrings = enumSmoothStep },
	{ .name = "Slew",       .min = 1,    .max = 500,  .def = 5,  .unit = kNT_unitMs,      .scaling = 0,              .enumStrings = NULL },
	{ .name = "Spin in",    .min = -500, .max = 500,  .def = 0,  .unit = kNT_unitNone,    .scaling = kNT_scaling100, .enumStrings = NULL },
	{ .name = "Spin out",   .min = -500, .max = 500,  .def = 0,  .unit = kNT_unitNone,    .scaling = kNT_scaling100, .enumStrings = NULL },
};
static_assert( ARRAY_SIZE(parameters) == kNumParams, "parameter table and enum are out of sync" );

static const uint8_t pageRotate[] = { kParamRotIn, kParamRotOut, kParamRadiate, kParamQuantIn, kParamQuantOut, kParamSlew, kParamSpinIn, kParamSpinOut };
static const uint8_t pageIn[]     = { kParamIn1, kParamIn2, kParamIn3, kParamIn4, kParamIn5, kParamIn6, kParamRotInCV, kParamRotOutCV, kParamRadCV };
static const uint8_t pageOut[]    = { kParamOut1, kParamOut2, kParamOut3, kParamOut4, kParamOut5, kParamOut6, kParamOutMode };

static const _NT_parameterPage pages[] = {
	{ .name = "Rotate",  .numParams = ARRAY_SIZE(pageRotate), .params = pageRotate },
	{ .name = "Inputs",  .numParams = ARRAY_SIZE(pageIn),     .params = pageIn },
	{ .name = "Outputs", .numParams = ARRAY_SIZE(pageOut),    .params = pageOut },
};
static const _NT_parameterPages parameterPages = { .numPages = ARRAY_SIZE(pages), .pages = pages };

struct _jumblerAlgorithm : public _NT_algorithm { jb::Router fx; };

static void calculateRequirements( _NT_algorithmRequirements& req, const int32_t* specifications )
{
	req.numParameters = ARRAY_SIZE(parameters);
	req.sram = sizeof(_jumblerAlgorithm);
	req.dram = 0; req.dtc = 0; req.itc = 0;
}

static _NT_algorithm* construct( const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements& req, const int32_t* specifications )
{
	_jumblerAlgorithm* alg = new (ptrs.sram) _jumblerAlgorithm();
	alg->parameters = parameters;
	alg->parameterPages = &parameterPages;
	alg->fx.init( NT_globals.sampleRate );
	return alg;
}

static void parameterChanged( _NT_algorithm* self, int p ) {}

static inline const float* busIn( const float* busFrames, int param, int numFrames )
{
	return param > 0 ? busFrames + ( param - 1 ) * numFrames : NULL;
}

static void step( _NT_algorithm* self, float* busFrames, int numFramesBy4 )
{
	_jumblerAlgorithm* a = (_jumblerAlgorithm*)self;
	const int16_t* v = a->v;
	const int numFrames = numFramesBy4 * 4;

	jb::Params P;
	P.rotIn    = 0.1f * v[kParamRotIn];
	P.rotOut   = 0.1f * v[kParamRotOut];
	P.radiate  = 0.01f * v[kParamRadiate];
	P.quantIn  = v[kParamQuantIn] != 0;
	P.quantOut = v[kParamQuantOut] != 0;
	P.slewMs   = (float)v[kParamSlew];
	P.spinIn   = 0.01f * v[kParamSpinIn];
	P.spinOut  = 0.01f * v[kParamSpinOut];
	P.cvRotIn  = 0.6f;       // 10 V = one full turn (6 channels)
	P.cvRotOut = 0.6f;
	P.cvRad    = 0.1f;       // 10 V = full radiate

	const float* ins[jb::kN];
	for ( int i = 0; i < jb::kN; ++i ) ins[i] = busIn( busFrames, v[kParamIn1 + i], numFrames );
	const float* cvI = busIn( busFrames, v[kParamRotInCV], numFrames );
	const float* cvO = busIn( busFrames, v[kParamRotOutCV], numFrames );
	const float* cvR = busIn( busFrames, v[kParamRadCV], numFrames );
	float* outs[jb::kN];
	for ( int o = 0; o < jb::kN; ++o ) outs[o] = v[kParamOut1 + o] > 0 ? busFrames + ( v[kParamOut1 + o] - 1 ) * numFrames : NULL;
	const bool replace = v[kParamOutMode] != 0;

	float tmp[jb::kN][kChunk];
	float* tp[jb::kN];
	for ( int o = 0; o < jb::kN; ++o ) tp[o] = tmp[o];

	for ( int off = 0; off < numFrames; off += kChunk )
	{
		int n = numFrames - off;
		if ( n > kChunk ) n = kChunk;
		jb::Inputs in;
		for ( int i = 0; i < jb::kN; ++i ) in.in[i] = ins[i] ? ins[i] + off : NULL;
		in.cvRotIn = cvI ? cvI + off : NULL;
		in.cvRotOut = cvO ? cvO + off : NULL;
		in.cvRad = cvR ? cvR + off : NULL;
		a->fx.process( P, in, tp, n );
		for ( int o = 0; o < jb::kN; ++o )
		{
			if ( !outs[o] ) continue;
			if ( replace ) for ( int s = 0; s < n; ++s ) outs[o][off + s] = tmp[o][s];
			else           for ( int s = 0; s < n; ++s ) outs[o][off + s] += tmp[o][s];
		}
	}
}

static const _NT_factory factory =
{
	.guid = NT_MULTICHAR( 'J', 'm', 'b', 'l' ),
	.name = "Jumbler",
	.description = "6x6 router with CV-controlled input and output rotation",
	.numSpecifications = 0,
	.calculateRequirements = calculateRequirements,
	.construct = construct,
	.parameterChanged = parameterChanged,
	.step = step,
	.draw = NULL,
	.midiRealtime = NULL,
	.midiMessage = NULL,
};

uintptr_t pluginEntry( _NT_selector selector, uint32_t data )
{
	switch ( selector )
	{
	case kNT_selector_version:      return kNT_apiVersionCurrent;
	case kNT_selector_numFactories: return 1;
	case kNT_selector_factoryInfo:  return (uintptr_t)( ( data == 0 ) ? &factory : NULL );
	}
	return 0;
}
