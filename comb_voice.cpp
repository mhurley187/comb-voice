// comb_voice.cpp - "Comb Voice" plugin for the Expert Sleepers disting NT.
//
// Written against the distingNT plug-in API (https://github.com/expertsleepersltd/distingNT_API).
// The DSP lives in comb_core.h; this file only maps parameters and buses.
//
// ASSUMPTIONS TO VERIFY against your copy of the API headers (all flagged with "CHECK"):
//   - bus values are volts (so 1 V/oct CV is used directly and audio is scaled by kAudioVolts)
//   - the parameter-unit names and NT_PARAMETER_* macros used below exist as spelled
#include <new>
#include <distingnt/api.h>
#include "comb_core.h"

// ---------------------------------------------------------------------------------------------
// The ARM compiler may silently turn struct initialisation or copies into calls to memset/memcpy.
// A plug-in is loaded by the NT's own loader, so it must not rely on C library symbols being
// available: provide them here, so those calls resolve inside the plug-in itself.
// (Compiled on the ARM target only, or when CV_PROVIDE_LIBC is defined for host testing.)
// ---------------------------------------------------------------------------------------------
#if defined(__arm__) || defined(CV_PROVIDE_LIBC)
extern "C" {

__attribute__((used, optimize("no-tree-loop-distribute-patterns")))
void* memset( void* dst, int c, size_t n )
{
	uint8_t* d = (uint8_t*)dst;
	while ( n-- ) *d++ = (uint8_t)c;
	return dst;
}

__attribute__((used, optimize("no-tree-loop-distribute-patterns")))
void* memcpy( void* dst, const void* src, size_t n )
{
	uint8_t* d = (uint8_t*)dst;
	const uint8_t* s = (const uint8_t*)src;
	while ( n-- ) *d++ = *s++;
	return dst;
}

}
#endif

#define ARRAY_SIZE(a) ((int)(sizeof(a) / sizeof((a)[0])))

static const float kAudioVolts = 5.0f;      // CHECK: +/-1.0 internal == +/-5 V on the bus
static const int   kChunk = 64;

enum
{
	kParamPitchIn,
	kParamGateIn,
	kParamAudioIn,
	kParamOutL,
	kParamOutLMode,
	kParamOutR,
	kParamOutRMode,

	kParamCoarse,
	kParamFine,
	kParamSpread,
	kParamFeedback,
	kParamDamping,
	kParamDispersion,

	kParamCombA,
	kParamMutA,
	kParamCombB,
	kParamMutB,
	kParamMorph,
	kParamMutAmount,

	kParamExcMode,
	kParamExcLevel,
	kParamExcDecay,
	kParamInLevel,
	kParamFreeze,

	kParamChaos,
	kParamChaosRate,
	kParamWidth,
	kParamLevel,

	kNumParams
};

static const char* const enumCombs[] = { "Classic", "Hollow", "String", "Metal", "Thick", "Cross" };
static const char* const enumMuts[]  = { "Clean", "Saturate", "Fold", "Crush", "Clip", "Decimate", "Rectify" };
static const char* const enumExc[]   = { "Drone", "Pluck" };
static const char* const enumOffOn[] = { "Off", "On" };

static const _NT_parameter parameters[] = {
	NT_PARAMETER_CV_INPUT( "Pitch CV", 0, 1 )
	NT_PARAMETER_CV_INPUT( "Gate", 0, 0 )
	NT_PARAMETER_AUDIO_INPUT( "Audio in", 0, 0 )
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Left output", 1, 13 )
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Right output", 1, 14 )

	{ .name = "Coarse",     .min = -48, .max = 48,   .def = 0,   .unit = kNT_unitSemitones, .scaling = 0, .enumStrings = NULL },
	{ .name = "Fine",       .min = -100, .max = 100, .def = 0,   .unit = kNT_unitCents,     .scaling = 0, .enumStrings = NULL },
	{ .name = "Spread",     .min = 0,   .max = 100,  .def = 10,  .unit = kNT_unitCents,     .scaling = 0, .enumStrings = NULL },
	{ .name = "Feedback",   .min = 0,   .max = 100,  .def = 90,  .unit = kNT_unitPercent,   .scaling = 0, .enumStrings = NULL },
	{ .name = "Damping",    .min = 0,   .max = 100,  .def = 50,  .unit = kNT_unitPercent,   .scaling = 0, .enumStrings = NULL },
	{ .name = "Dispersion", .min = 0,   .max = 100,  .def = 50,  .unit = kNT_unitPercent,   .scaling = 0, .enumStrings = NULL },

	{ .name = "Comb A",     .min = 0, .max = 5, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = enumCombs },
	{ .name = "Mutation A", .min = 0, .max = 6, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = enumMuts },
	{ .name = "Comb B",     .min = 0, .max = 5, .def = 2, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = enumCombs },
	{ .name = "Mutation B", .min = 0, .max = 6, .def = 1, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = enumMuts },
	{ .name = "Morph A>B",  .min = 0, .max = 100, .def = 0,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Mutation amt", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "Exciter",    .min = 0, .max = 1,    .def = 0,   .unit = kNT_unitEnum,    .scaling = 0, .enumStrings = enumExc },
	{ .name = "Exciter lvl", .min = 0, .max = 100, .def = 70,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Pluck decay", .min = 5, .max = 2000, .def = 200, .unit = kNT_unitMs,     .scaling = 0, .enumStrings = NULL },
	{ .name = "Input level", .min = 0, .max = 100, .def = 0,   .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Freeze",     .min = 0, .max = 1,    .def = 0,   .unit = kNT_unitEnum,    .scaling = 0, .enumStrings = enumOffOn },

	{ .name = "Chaos",      .min = 0, .max = 100, .def = 0,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Chaos rate", .min = 0, .max = 100, .def = 30, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Width",      .min = 0, .max = 200, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Level",      .min = -40, .max = 6, .def = 0,  .unit = kNT_unitDb,      .scaling = 0, .enumStrings = NULL },
};

static_assert( ARRAY_SIZE(parameters) == kNumParams, "parameter table and enum are out of sync" );

static const uint8_t pageTone[]   = { kParamCoarse, kParamFine, kParamSpread, kParamFeedback, kParamDamping, kParamDispersion };
static const uint8_t pageEngine[] = { kParamCombA, kParamMutA, kParamCombB, kParamMutB, kParamMorph, kParamMutAmount };
static const uint8_t pageExc[]    = { kParamExcMode, kParamExcLevel, kParamExcDecay, kParamInLevel, kParamFreeze };
static const uint8_t pageMod[]    = { kParamChaos, kParamChaosRate, kParamWidth, kParamLevel };
static const uint8_t pageRoute[]  = { kParamPitchIn, kParamGateIn, kParamAudioIn, kParamOutL, kParamOutLMode, kParamOutR, kParamOutRMode };

static const _NT_parameterPage pages[] = {
	{ .name = "Tone",       .numParams = ARRAY_SIZE(pageTone),   .params = pageTone },
	{ .name = "Engines",    .numParams = ARRAY_SIZE(pageEngine), .params = pageEngine },
	{ .name = "Exciter",    .numParams = ARRAY_SIZE(pageExc),    .params = pageExc },
	{ .name = "Mod/Output", .numParams = ARRAY_SIZE(pageMod),    .params = pageMod },
	{ .name = "Routing",    .numParams = ARRAY_SIZE(pageRoute),  .params = pageRoute },
};

static const _NT_parameterPages parameterPages = {
	.numPages = ARRAY_SIZE(pages),
	.pages = pages,
};

struct _combAlgorithm : public _NT_algorithm
{
	comb::Voice voice;
};

static void calculateRequirements( _NT_algorithmRequirements& req, const int32_t* specifications )
{
	req.numParameters = ARRAY_SIZE(parameters);
	req.sram = sizeof(_combAlgorithm);
	req.dram = comb::kDramFloats * sizeof(float);
	req.dtc = 0;
	req.itc = 0;
}

static _NT_algorithm* construct( const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements& req, const int32_t* specifications )
{
	_combAlgorithm* alg = new (ptrs.sram) _combAlgorithm();
	alg->parameters = parameters;
	alg->parameterPages = &parameterPages;
	alg->voice.init( NT_globals.sampleRate, (float*)ptrs.dram );
	return alg;
}

static void parameterChanged( _NT_algorithm* self, int p )
{
	// parameters are read directly in step()
}

static inline const float* busIn( const float* busFrames, int param, int numFrames )
{
	return param > 0 ? busFrames + ( param - 1 ) * numFrames : NULL;
}

static void step( _NT_algorithm* self, float* busFrames, int numFramesBy4 )
{
	_combAlgorithm* a = (_combAlgorithm*)self;
	const int16_t* v = a->v;
	const int numFrames = numFramesBy4 * 4;

	comb::Params P;
	P.semis       = (float)v[kParamCoarse] + 0.01f * (float)v[kParamFine];
	P.spreadCents = (float)v[kParamSpread];
	P.feedback    = 0.01f * v[kParamFeedback];
	P.damping     = 0.01f * v[kParamDamping];
	P.dispersion  = 0.01f * v[kParamDispersion];
	P.typeA = v[kParamCombA]; P.mutA = v[kParamMutA];
	P.typeB = v[kParamCombB]; P.mutB = v[kParamMutB];
	P.morph       = 0.01f * v[kParamMorph];
	P.mutAmount   = 0.01f * v[kParamMutAmount];
	P.chaos       = 0.01f * v[kParamChaos];
	P.chaosHz     = 0.05f * ntm::exp2( 8.643856f * 0.01f * v[kParamChaosRate] );   // 0.05 .. 20 Hz
	P.pluckMode   = v[kParamExcMode] != 0;
	P.excLevel    = 0.01f * v[kParamExcLevel];
	P.excDecayMs  = (float)v[kParamExcDecay];
	P.extLevel    = 0.01f * v[kParamInLevel];
	P.freeze      = v[kParamFreeze] != 0;
	P.width       = 0.01f * v[kParamWidth];
	P.gain        = ntm::exp2( 3.321928f * (float)v[kParamLevel] / 20.f );

	const float* pitch = busIn( busFrames, v[kParamPitchIn], numFrames );
	const float* gate  = busIn( busFrames, v[kParamGateIn], numFrames );
	const float* audio = busIn( busFrames, v[kParamAudioIn], numFrames );
	float* outL = busFrames + ( v[kParamOutL] - 1 ) * numFrames;
	float* outR = busFrames + ( v[kParamOutR] - 1 ) * numFrames;
	const bool replaceL = v[kParamOutLMode] != 0;
	const bool replaceR = v[kParamOutRMode] != 0;

	float extBuf[kChunk], l[kChunk], r[kChunk];

	for ( int off = 0; off < numFrames; off += kChunk )
	{
		int n = numFrames - off;
		if ( n > kChunk ) n = kChunk;

		if ( audio )
			for ( int i = 0; i < n; ++i ) extBuf[i] = audio[off + i] * ( 1.f / kAudioVolts );

		a->voice.process( P, pitch ? pitch + off : NULL, gate ? gate + off : NULL,
		                  audio ? extBuf : NULL, l, r, n );

		for ( int i = 0; i < n; ++i )
		{
			float sl = l[i] * kAudioVolts, sr = r[i] * kAudioVolts;
			outL[off + i] = replaceL ? sl : outL[off + i] + sl;
			outR[off + i] = replaceR ? sr : outR[off + i] + sr;
		}
	}
}

static const _NT_factory factory =
{
	.guid = NT_MULTICHAR( 'C', 'm', 'b', 'V' ),      // pick your own unique 4 characters before sharing
	.name = "Comb Voice",
	.description = "Stereo comb-filter synth voice: 2 comb x mutation engines, chaos, freeze",
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
	case kNT_selector_version:
		return kNT_apiVersionCurrent;
	case kNT_selector_numFactories:
		return 1;
	case kNT_selector_factoryInfo:
		return (uintptr_t)( ( data == 0 ) ? &factory : NULL );
	}
	return 0;
}
