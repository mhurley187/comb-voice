// allpass_smear.cpp - "Allpass Smear": stereo allpass filter bank (2..98 poles) for the Expert Sleepers disting NT.
// DSP lives in allpass_core.h.
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <new>
#include <distingnt/api.h>
#include "allpass_core.h"

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

static const float kAudioVolts = 5.0f;
static const int   kChunk = 64;

enum
{
	kParamInL, kParamInR,
	kParamPolesCV, kParamFreqCV, kParamResCV, kParamSpreadCV, kParamBiasCV, kParamSharpCV, kParamMixCV,
	kParamOutL, kParamOutLMode, kParamOutR, kParamOutRMode,

	kParamPoles, kParamFreq, kParamRes, kParamSpread, kParamBias, kParamSharp,
	kParamLfoRate, kParamLfoDepth, kParamLfoPhase, kParamEnv, kParamStereo,
	kParamMix, kParamLevel,

	kNumParams
};

static const _NT_parameter parameters[] = {
	NT_PARAMETER_AUDIO_INPUT( "Audio in L", 1, 1 )
	NT_PARAMETER_AUDIO_INPUT( "Audio in R", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Poles CV", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Freq CV (1V/oct)", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Resonance CV", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Spread CV", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Bias CV", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Sharpness CV", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Mix CV", 0, 0 )
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Left output", 1, 13 )
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Right output", 1, 14 )

	{ .name = "Poles",      .min = 20,   .max = 980,  .def = 400, .unit = kNT_unitNone,    .scaling = kNT_scaling10,  .enumStrings = NULL },
	{ .name = "Frequency",  .min = 0,    .max = 1000, .def = 500, .unit = kNT_unitVolts,   .scaling = kNT_scaling100, .enumStrings = NULL },
	{ .name = "Resonance",  .min = -99,  .max = 99,   .def = 0,   .unit = kNT_unitPercent, .scaling = 0,              .enumStrings = NULL },
	{ .name = "Spread",     .min = 0,    .max = 600,  .def = 200, .unit = kNT_unitNone,    .scaling = kNT_scaling100, .enumStrings = NULL },
	{ .name = "Peak bias",  .min = -100, .max = 100,  .def = 0,   .unit = kNT_unitPercent, .scaling = 0,              .enumStrings = NULL },
	{ .name = "Sharpness",  .min = 5,    .max = 100,  .def = 7,   .unit = kNT_unitNone,    .scaling = kNT_scaling10,  .enumStrings = NULL },

	{ .name = "LFO rate",   .min = 1,    .max = 2000, .def = 50,  .unit = kNT_unitHz,      .scaling = kNT_scaling100, .enumStrings = NULL },
	{ .name = "LFO depth",  .min = 0,    .max = 400,  .def = 0,   .unit = kNT_unitNone,    .scaling = kNT_scaling100, .enumStrings = NULL },
	{ .name = "LFO phase",  .min = 0,    .max = 180,  .def = 90,  .unit = kNT_unitNone,    .scaling = 0,              .enumStrings = NULL },
	{ .name = "Env amount", .min = -400, .max = 400,  .def = 0,   .unit = kNT_unitNone,    .scaling = kNT_scaling100, .enumStrings = NULL },
	{ .name = "Stereo",     .min = -200, .max = 200,  .def = 0,   .unit = kNT_unitNone,    .scaling = kNT_scaling100, .enumStrings = NULL },

	{ .name = "Mix",        .min = 0,    .max = 100,  .def = 50,  .unit = kNT_unitPercent, .scaling = 0,              .enumStrings = NULL },
	{ .name = "Level",      .min = -40,  .max = 12,   .def = 0,   .unit = kNT_unitDb,      .scaling = 0,              .enumStrings = NULL },
};
static_assert( ARRAY_SIZE(parameters) == kNumParams, "parameter table and enum are out of sync" );

static const uint8_t pageFilter[]  = { kParamPoles, kParamFreq, kParamRes, kParamSpread, kParamBias, kParamSharp };
static const uint8_t pageMod[]     = { kParamLfoRate, kParamLfoDepth, kParamLfoPhase, kParamEnv, kParamStereo };
static const uint8_t pageOut[]     = { kParamMix, kParamLevel };
static const uint8_t pageRoute[]   = { kParamInL, kParamInR, kParamPolesCV, kParamFreqCV, kParamResCV, kParamSpreadCV, kParamBiasCV, kParamSharpCV, kParamMixCV,
                                       kParamOutL, kParamOutLMode, kParamOutR, kParamOutRMode };

static const _NT_parameterPage pages[] = {
	{ .name = "Filter",  .numParams = ARRAY_SIZE(pageFilter), .params = pageFilter },
	{ .name = "Mod",     .numParams = ARRAY_SIZE(pageMod),    .params = pageMod },
	{ .name = "Output",  .numParams = ARRAY_SIZE(pageOut),    .params = pageOut },
	{ .name = "Routing", .numParams = ARRAY_SIZE(pageRoute),  .params = pageRoute },
};
static const _NT_parameterPages parameterPages = { .numPages = ARRAY_SIZE(pages), .pages = pages };

struct _apAlgorithm : public _NT_algorithm { ap::Filter fx; };

static void calculateRequirements( _NT_algorithmRequirements& req, const int32_t* specifications )
{
	req.numParameters = ARRAY_SIZE(parameters);
	req.sram = sizeof(_apAlgorithm);
	req.dram = 0; req.dtc = 0; req.itc = 0;
}

static _NT_algorithm* construct( const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements& req, const int32_t* specifications )
{
	_apAlgorithm* alg = new (ptrs.sram) _apAlgorithm();
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
	_apAlgorithm* a = (_apAlgorithm*)self;
	const int16_t* v = a->v;
	const int numFrames = numFramesBy4 * 4;

	ap::Params P;
	P.poles     = 0.1f * v[kParamPoles];
	P.freqOct   = 0.01f * v[kParamFreq];
	P.res       = 0.01f * v[kParamRes];
	P.spreadOct = 0.01f * v[kParamSpread];
	P.bias      = 0.01f * v[kParamBias];
	P.q         = 0.1f * v[kParamSharp];
	P.mix       = 0.01f * v[kParamMix];
	P.lfoHz     = 0.01f * v[kParamLfoRate];
	P.lfoDepth  = 0.01f * v[kParamLfoDepth];
	P.lfoPhase  = (float)v[kParamLfoPhase] / 360.f;
	P.envAmt    = 0.01f * v[kParamEnv];
	P.stereoOct = 0.01f * v[kParamStereo];
	P.gain      = ntm::exp2( 3.321928f * (float)v[kParamLevel] / 20.f );
	P.cvScale[0] = 9.6f;  P.cvScale[1] = 0.f;  P.cvScale[2] = 0.2f; P.cvScale[3] = 0.6f;
	P.cvScale[4] = 0.2f;  P.cvScale[5] = 0.95f; P.cvScale[6] = 0.1f;

	const float* inL = busIn( busFrames, v[kParamInL], numFrames );
	const float* inR = busIn( busFrames, v[kParamInR], numFrames );
	const float* cvs[7] = {
		busIn( busFrames, v[kParamPolesCV],  numFrames ), busIn( busFrames, v[kParamFreqCV],  numFrames ),
		busIn( busFrames, v[kParamResCV],    numFrames ), busIn( busFrames, v[kParamSpreadCV], numFrames ),
		busIn( busFrames, v[kParamBiasCV],   numFrames ), busIn( busFrames, v[kParamSharpCV],  numFrames ),
		busIn( busFrames, v[kParamMixCV],    numFrames ) };
	float* outL = busFrames + ( v[kParamOutL] - 1 ) * numFrames;
	float* outR = busFrames + ( v[kParamOutR] - 1 ) * numFrames;
	const bool replaceL = v[kParamOutLMode] != 0;
	const bool replaceR = v[kParamOutRMode] != 0;

	float bufL[kChunk], bufR[kChunk], l[kChunk], r[kChunk];

	for ( int off = 0; off < numFrames; off += kChunk )
	{
		int n = numFrames - off;
		if ( n > kChunk ) n = kChunk;

		ap::Inputs in;
		in.l = in.r = NULL;
		if ( inL ) { for ( int i = 0; i < n; ++i ) bufL[i] = inL[off + i] * ( 1.f / kAudioVolts ); in.l = bufL; }
		if ( inR ) { for ( int i = 0; i < n; ++i ) bufR[i] = inR[off + i] * ( 1.f / kAudioVolts ); in.r = bufR; }
		else if ( inL ) in.r = bufL;                                  // mono input feeds both sides
		in.cvPoles  = cvs[0] ? cvs[0] + off : NULL;
		in.cvFreq   = cvs[1] ? cvs[1] + off : NULL;
		in.cvRes    = cvs[2] ? cvs[2] + off : NULL;
		in.cvSpread = cvs[3] ? cvs[3] + off : NULL;
		in.cvBias   = cvs[4] ? cvs[4] + off : NULL;
		in.cvQ      = cvs[5] ? cvs[5] + off : NULL;
		in.cvMix    = cvs[6] ? cvs[6] + off : NULL;
		a->fx.process( P, in, l, r, n );

		for ( int i = 0; i < n; ++i )
		{
			const float sl = l[i] * kAudioVolts, sr = r[i] * kAudioVolts;
			outL[off + i] = replaceL ? sl : outL[off + i] + sl;
			outR[off + i] = replaceR ? sr : outR[off + i] + sr;
		}
	}
}

static const _NT_factory factory =
{
	.guid = NT_MULTICHAR( 'A', 'p', 'S', 'm' ),
	.name = "Allpass Smear",
	.description = "Stereo allpass filter bank, 2 to 98 poles, with peak bias",
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
