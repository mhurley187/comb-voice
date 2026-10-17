// spectral_delay.cpp - "Spectral Delay": multitap delay with a spectral resynthesis stage in the feedback loop,
// for the Expert Sleepers disting NT. DSP lives in spectral_core.h.
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <new>
#include <distingnt/api.h>
#include "spectral_core.h"

// Plug-ins must not rely on libc symbols; provide memset/memcpy for the ARM build.
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

static const float kAudioVolts = 5.0f;      // +/-1.0 internal == +/-5 V on the bus
static const int   kChunk = 64;

enum
{
	kParamInL, kParamInR, kParamTimeCV, kParamGroupCV, kParamShiftCV, kParamFreezeGate,
	kParamOutL, kParamOutLMode, kParamOutR, kParamOutRMode,

	// delay
	kParamTime, kParamTaps, kParamGroup, kParamSlope, kParamPan, kParamFeedback, kParamDamping,
	// spectral
	kParamQuality, kParamPartials, kParamShift, kParamStretch, kParamScatter, kParamSmear, kParamTilt,
	kParamFreeze, kParamResFb, kParamResOut, kParamWidth,
	// output
	kParamMix, kParamLevel,

	kNumParams
};

static const char* const enumQuality[] = { "Fast", "Fine" };
static const char* const enumOffOn[]   = { "Off", "On" };

static const _NT_parameter parameters[] = {
	NT_PARAMETER_AUDIO_INPUT( "Audio in L", 1, 1 )
	NT_PARAMETER_AUDIO_INPUT( "Audio in R", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Time CV", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Grouping CV", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Shift CV", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Freeze gate", 0, 0 )
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Left output", 1, 13 )
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Right output", 1, 14 )

	{ .name = "Time",       .min = 0,    .max = 1000, .def = 500, .unit = kNT_unitPercent, .scaling = kNT_scaling10, .enumStrings = NULL },
	{ .name = "Taps",       .min = 1,    .max = 8,    .def = 4,   .unit = kNT_unitNone,    .scaling = 0, .enumStrings = NULL },
	{ .name = "Grouping",   .min = -100, .max = 100,  .def = 0,   .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Tap slope",  .min = 0,    .max = 100,  .def = 30,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Tap pan",    .min = 0,    .max = 100,  .def = 50,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Feedback",   .min = 0,    .max = 110,  .def = 50,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Damping",    .min = 0,    .max = 100,  .def = 25,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "Quality",    .min = 0,    .max = 1,    .def = 1,   .unit = kNT_unitEnum,    .scaling = 0, .enumStrings = enumQuality },
	{ .name = "Partials",   .min = 1,    .max = 32,   .def = 16,  .unit = kNT_unitNone,    .scaling = 0, .enumStrings = NULL },
	{ .name = "Shift",      .min = -240, .max = 240,  .def = 0,   .unit = kNT_unitSemitones, .scaling = kNT_scaling10, .enumStrings = NULL },
	{ .name = "Stretch",    .min = -100, .max = 100,  .def = 0,   .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Scatter",    .min = 0,    .max = 100,  .def = 0,   .unit = kNT_unitCents,   .scaling = 0, .enumStrings = NULL },
	{ .name = "Smear",      .min = 0,    .max = 100,  .def = 20,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Tilt",       .min = -100, .max = 100,  .def = 0,   .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Freeze",     .min = 0,    .max = 1,    .def = 0,   .unit = kNT_unitEnum,    .scaling = 0, .enumStrings = enumOffOn },
	{ .name = "Resynth FB", .min = 0,    .max = 100,  .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Resynth out", .min = 0,   .max = 100,  .def = 50,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Width",      .min = 0,    .max = 100,  .def = 60,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "Mix",        .min = 0,    .max = 100,  .def = 50,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Level",      .min = -40,  .max = 6,    .def = 0,   .unit = kNT_unitDb,      .scaling = 0, .enumStrings = NULL },
};
static_assert( ARRAY_SIZE(parameters) == kNumParams, "parameter table and enum are out of sync" );

static const uint8_t pageDelay[]    = { kParamTime, kParamTaps, kParamGroup, kParamSlope, kParamPan, kParamFeedback, kParamDamping };
static const uint8_t pageSpectral[] = { kParamResFb, kParamResOut, kParamShift, kParamPartials, kParamSmear, kParamStretch, kParamScatter, kParamTilt, kParamWidth, kParamFreeze, kParamQuality };
static const uint8_t pageOut[]      = { kParamMix, kParamLevel };
static const uint8_t pageRoute[]    = { kParamInL, kParamInR, kParamTimeCV, kParamGroupCV, kParamShiftCV, kParamFreezeGate, kParamOutL, kParamOutLMode, kParamOutR, kParamOutRMode };

static const _NT_parameterPage pages[] = {
	{ .name = "Delay",    .numParams = ARRAY_SIZE(pageDelay),    .params = pageDelay },
	{ .name = "Spectral", .numParams = ARRAY_SIZE(pageSpectral), .params = pageSpectral },
	{ .name = "Output",   .numParams = ARRAY_SIZE(pageOut),      .params = pageOut },
	{ .name = "Routing",  .numParams = ARRAY_SIZE(pageRoute),    .params = pageRoute },
};
static const _NT_parameterPages parameterPages = { .numPages = ARRAY_SIZE(pages), .pages = pages };

struct _spectralAlgorithm : public _NT_algorithm { sd::Effect fx; };

static void calculateRequirements( _NT_algorithmRequirements& req, const int32_t* specifications )
{
	req.numParameters = ARRAY_SIZE(parameters);
	req.sram = sizeof(_spectralAlgorithm);
	req.dram = sizeof(sd::Big);
	req.dtc = 0; req.itc = 0;
}

static _NT_algorithm* construct( const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements& req, const int32_t* specifications )
{
	_spectralAlgorithm* alg = new (ptrs.sram) _spectralAlgorithm();
	alg->parameters = parameters;
	alg->parameterPages = &parameterPages;
	alg->fx.init( (sd::Big*)ptrs.dram, NT_globals.sampleRate );
	return alg;
}

static void parameterChanged( _NT_algorithm* self, int p ) {}

static inline const float* busIn( const float* busFrames, int param, int numFrames )
{
	return param > 0 ? busFrames + ( param - 1 ) * numFrames : NULL;
}

static void step( _NT_algorithm* self, float* busFrames, int numFramesBy4 )
{
	_spectralAlgorithm* a = (_spectralAlgorithm*)self;
	const int16_t* v = a->v;
	const int numFrames = numFramesBy4 * 4;

	sd::Params P;
	P.timeMs       = 20.f * ntm::exp2( 6.965784f * 0.001f * v[kParamTime] );          // 20 ms .. 2.5 s
	P.taps         = v[kParamTaps];
	P.group        = 0.01f * v[kParamGroup];
	P.slope        = 0.01f * v[kParamSlope];
	P.pan          = 0.01f * v[kParamPan];
	P.feedback     = 0.01f * v[kParamFeedback];
	P.dampHz       = 20000.f * ntm::exp2( -5.321928f * 0.01f * v[kParamDamping] );    // 20 kHz .. 500 Hz
	P.quality      = v[kParamQuality];
	P.partials     = v[kParamPartials];
	P.shiftSemis   = 0.1f * v[kParamShift];
	P.stretch      = 0.01f * v[kParamStretch];
	P.scatterCents = (float)v[kParamScatter];
	P.smear        = 0.0095f * v[kParamSmear];
	P.tiltDbOct    = 0.06f * v[kParamTilt];
	P.freeze       = v[kParamFreeze] != 0;
	P.resFb        = 0.01f * v[kParamResFb];
	P.resOut       = 0.01f * v[kParamResOut];
	P.width        = 0.01f * v[kParamWidth];
	P.mix          = 0.01f * v[kParamMix];
	P.gain         = ntm::exp2( 3.321928f * (float)v[kParamLevel] / 20.f );

	const float* inL   = busIn( busFrames, v[kParamInL], numFrames );
	const float* inR   = busIn( busFrames, v[kParamInR], numFrames );
	const float* tcv   = busIn( busFrames, v[kParamTimeCV], numFrames );
	const float* gcv   = busIn( busFrames, v[kParamGroupCV], numFrames );
	const float* scv   = busIn( busFrames, v[kParamShiftCV], numFrames );
	const float* fgate = busIn( busFrames, v[kParamFreezeGate], numFrames );
	float* outL = busFrames + ( v[kParamOutL] - 1 ) * numFrames;
	float* outR = busFrames + ( v[kParamOutR] - 1 ) * numFrames;
	const bool replaceL = v[kParamOutLMode] != 0;
	const bool replaceR = v[kParamOutRMode] != 0;

	float bufL[kChunk], bufR[kChunk], l[kChunk], r[kChunk];

	for ( int off = 0; off < numFrames; off += kChunk )
	{
		int n = numFrames - off;
		if ( n > kChunk ) n = kChunk;

		sd::Inputs in;
		if ( inL ) { for ( int i = 0; i < n; ++i ) bufL[i] = inL[off + i] * ( 1.f / kAudioVolts ); in.l = bufL; }
		if ( inR ) { for ( int i = 0; i < n; ++i ) bufR[i] = inR[off + i] * ( 1.f / kAudioVolts ); in.r = bufR; }
		in.timeCV     = tcv ? tcv + off : NULL;
		in.groupCV    = gcv ? gcv + off : NULL;
		in.shiftCV    = scv ? scv + off : NULL;
		in.freezeGate = fgate ? fgate + off : NULL;
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
	.guid = NT_MULTICHAR( 'S', 'p', 'D', 'l' ),
	.name = "Spectral Delay",
	.description = "Multitap delay with spectral resynthesis in the feedback loop",
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
