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

	// added in v2 (appended so older indices stay put)
	kParamAudioInR,
	kParamLinFMIn,
	kParamExpFMIn,
	kParamDry,
	kParamWet,
	kParamSwirl,
	kParamSwirlRate,
	kParamShPos,
	kParamShMode,
	kParamShRange,
	kParamShift,
	kParamShiftFine,
	kParamShMix,
	kParamLinFMDepth,
	kParamExpFMDepth,
	kParamRegen,
	kParamRegenRoute,
	kParamRegenTime,

	kNumParams
};

static const char* const enumCombs[] = { "Classic", "Hollow", "String", "Metal", "Thick", "Cross" };
static const char* const enumMuts[]  = { "Clean", "Saturate", "Fold", "Crush", "Clip", "Decimate", "Rectify" };
static const char* const enumExc[]   = { "Drone", "Pluck" };
static const char* const enumOffOn[] = { "Off", "On" };
static const char* const enumShPos[]   = { "Off", "Input", "Loop", "Output" };
static const char* const enumShMode[]  = { "Up", "Down", "Split", "Ring" };
static const char* const enumShRange[] = { "x1", "x10", "x100" };
static const char* const enumRegenRoute[] = { "Down", "Up", "Both" };

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
	{ .name = "Osc level", .min = 0, .max = 100, .def = 70,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Pluck decay", .min = 5, .max = 2000, .def = 200, .unit = kNT_unitMs,     .scaling = 0, .enumStrings = NULL },
	{ .name = "Input level", .min = 0, .max = 200, .def = 100,   .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Freeze",     .min = 0, .max = 1,    .def = 0,   .unit = kNT_unitEnum,    .scaling = 0, .enumStrings = enumOffOn },

	{ .name = "Chaos",      .min = 0, .max = 100, .def = 0,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Chaos rate", .min = 0, .max = 100, .def = 30, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Width",      .min = 0, .max = 200, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Level",      .min = -40, .max = 6, .def = 0,  .unit = kNT_unitDb,      .scaling = 0, .enumStrings = NULL },

	NT_PARAMETER_AUDIO_INPUT( "Audio in R", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Lin FM in", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Exp FM in", 0, 0 )
	{ .name = "Dry level",  .min = 0, .max = 100, .def = 0,   .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Wet level",  .min = 0, .max = 100, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Swirl",      .min = 0, .max = 100, .def = 50,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Swirl rate", .min = 0, .max = 100, .def = 50,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Shifter",    .min = 0, .max = 3, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = enumShPos },
	{ .name = "Shift mode", .min = 0, .max = 3, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = enumShMode },
	{ .name = "Shift range", .min = 0, .max = 2, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = enumShRange },
	{ .name = "Shift",      .min = -500, .max = 500, .def = 0, .unit = kNT_unitHz, .scaling = kNT_scaling10, .enumStrings = NULL },
	{ .name = "Shift fine", .min = -100, .max = 100, .def = 0, .unit = kNT_unitHz, .scaling = kNT_scaling100, .enumStrings = NULL },
	{ .name = "Shifter mix", .min = 0, .max = 100, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Lin FM depth", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Exp FM depth", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Regen",      .min = 0, .max = 95, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Regen route", .min = 0, .max = 2, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = enumRegenRoute },
	{ .name = "Regen time", .min = 1, .max = 80, .def = 10, .unit = kNT_unitMs, .scaling = 0, .enumStrings = NULL },
};

static_assert( ARRAY_SIZE(parameters) == kNumParams, "parameter table and enum are out of sync" );

static const uint8_t pageTone[]   = { kParamCoarse, kParamFine, kParamSpread, kParamFeedback, kParamDamping, kParamDispersion };
static const uint8_t pageEngine[] = { kParamCombA, kParamMutA, kParamCombB, kParamMutB, kParamMorph, kParamMutAmount };
static const uint8_t pageSwirl[]  = { kParamSwirl, kParamSwirlRate, kParamChaos, kParamChaosRate, kParamWidth };
static const uint8_t pageInput[]  = { kParamExcLevel, kParamExcMode, kParamExcDecay, kParamInLevel, kParamDry, kParamWet, kParamFreeze };
static const uint8_t pageShift[]  = { kParamShPos, kParamShMode, kParamShRange, kParamShift, kParamShiftFine, kParamShMix, kParamLinFMDepth, kParamExpFMDepth };
static const uint8_t pageRegen[]  = { kParamRegen, kParamRegenRoute, kParamRegenTime };
static const uint8_t pageOut[]    = { kParamLevel };
static const uint8_t pageRoute[]  = { kParamPitchIn, kParamGateIn, kParamAudioIn, kParamAudioInR, kParamLinFMIn, kParamExpFMIn, kParamOutL, kParamOutLMode, kParamOutR, kParamOutRMode };

static const _NT_parameterPage pages[] = {
	{ .name = "Tone",       .numParams = ARRAY_SIZE(pageTone),   .params = pageTone },
	{ .name = "Engines",    .numParams = ARRAY_SIZE(pageEngine), .params = pageEngine },
	{ .name = "Swirl/Mod",  .numParams = ARRAY_SIZE(pageSwirl),  .params = pageSwirl },
	{ .name = "Input/Osc",  .numParams = ARRAY_SIZE(pageInput),  .params = pageInput },
	{ .name = "Shifter",    .numParams = ARRAY_SIZE(pageShift),  .params = pageShift },
	{ .name = "Regen",      .numParams = ARRAY_SIZE(pageRegen),  .params = pageRegen },
	{ .name = "Output",     .numParams = ARRAY_SIZE(pageOut),    .params = pageOut },
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
	P.dry         = 0.01f * v[kParamDry];
	P.wet         = 0.01f * v[kParamWet];
	P.swirl       = 0.01f * v[kParamSwirl];
	P.swirlHz     = 0.03f * ntm::exp2( 7.f * 0.01f * v[kParamSwirlRate] );             // 0.03 .. 3.8 Hz
	P.shPos       = v[kParamShPos];
	P.shMode      = v[kParamShMode];
	P.shMix       = 0.01f * v[kParamShMix];
	const float rangeMul = v[kParamShRange] == 2 ? 100.f : ( v[kParamShRange] == 1 ? 10.f : 1.f );
	P.shiftHz     = ( 0.1f * v[kParamShift] + 0.01f * v[kParamShiftFine] ) * rangeMul;
	P.linHzPerV   = v[kParamLinFMDepth] * rangeMul;                                      // 100% = 100 Hz/V at x1
	P.expOctPerV  = 0.01f * v[kParamExpFMDepth];
	P.regen       = 0.01f * v[kParamRegen];
	P.regenRoute  = v[kParamRegenRoute];
	P.regenMs     = (float)v[kParamRegenTime];
	P.freeze      = v[kParamFreeze] != 0;
	P.width       = 0.01f * v[kParamWidth];
	P.gain        = ntm::exp2( 3.321928f * (float)v[kParamLevel] / 20.f );

	const float* pitch = busIn( busFrames, v[kParamPitchIn], numFrames );
	const float* gate  = busIn( busFrames, v[kParamGateIn], numFrames );
	const float* audio = busIn( busFrames, v[kParamAudioIn], numFrames );
	const float* audioR = busIn( busFrames, v[kParamAudioInR], numFrames );
	const float* linFM = busIn( busFrames, v[kParamLinFMIn], numFrames );
	const float* expFM = busIn( busFrames, v[kParamExpFMIn], numFrames );
	float* outL = busFrames + ( v[kParamOutL] - 1 ) * numFrames;
	float* outR = busFrames + ( v[kParamOutR] - 1 ) * numFrames;
	const bool replaceL = v[kParamOutLMode] != 0;
	const bool replaceR = v[kParamOutRMode] != 0;

	float extBuf[kChunk], extBufR[kChunk], l[kChunk], r[kChunk];

	for ( int off = 0; off < numFrames; off += kChunk )
	{
		int n = numFrames - off;
		if ( n > kChunk ) n = kChunk;

		if ( audio )
			for ( int i = 0; i < n; ++i ) extBuf[i] = audio[off + i] * ( 1.f / kAudioVolts );

		if ( audioR )
			for ( int i = 0; i < n; ++i ) extBufR[i] = audioR[off + i] * ( 1.f / kAudioVolts );

		comb::Inputs in;
		in.pitch = pitch ? pitch + off : NULL;
		in.gate  = gate ? gate + off : NULL;
		in.extL  = audio ? extBuf : NULL;
		in.extR  = audioR ? extBufR : NULL;
		in.linFM = linFM ? linFM + off : NULL;
		in.expFM = expFM ? expFM + off : NULL;
		a->voice.process( P, in, l, r, n );

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
	.description = "Stereo comb synth voice / effect with swirl, frequency shifter, regen",
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
