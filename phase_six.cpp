// phase_six.cpp - "Phase Six": six phase-shiftable, time-stretchable LFOs for the Expert Sleepers disting NT.
// LFO n is wired to output n (outputs default to buses 13..18). DSP lives in lfo_core.h.
#include <new>
#include <distingnt/api.h>
#include "lfo_core.h"

// Plug-ins must not rely on libc symbols; provide memset/memcpy for the ARM build (see the Comb Voice notes).
#if defined(__arm__) || defined(CV_PROVIDE_LIBC)
extern "C" {
__attribute__((used, optimize("no-tree-loop-distribute-patterns")))
void* memset( void* dst, int c, size_t n ) { uint8_t* d = (uint8_t*)dst; while ( n-- ) *d++ = (uint8_t)c; return dst; }
__attribute__((used, optimize("no-tree-loop-distribute-patterns")))
void* memcpy( void* dst, const void* src, size_t n ) { uint8_t* d = (uint8_t*)dst; const uint8_t* s = (const uint8_t*)src; while ( n-- ) *d++ = *s++; return dst; }
}
#endif

#define ARRAY_SIZE(a) ((int)(sizeof(a) / sizeof((a)[0])))

enum
{
	kParamRateCV, kParamPhaseCV, kParamReset,
	kParamOut1, kParamOut1Mode, kParamOut2, kParamOut2Mode, kParamOut3, kParamOut3Mode,
	kParamOut4, kParamOut4Mode, kParamOut5, kParamOut5Mode, kParamOut6, kParamOut6Mode,

	kParamRate, kParamRange, kParamRateSpread, kParamPhase,
	kParamShape, kParamSkew,
	kParamDepth, kParamPolarity, kParamOffset,

	kNumParams
};

static const char* const enumShapes[] = {
	"Sine", "Triangle", "Ramp up", "Ramp down", "Square", "Smooth rnd", "Stepped rnd", "Glide rnd", "Staircase", "Folded sine",
	"Rect sine", "Pump", "Bounce", "Chirp", "FM wobble", "Wander", "Ratchet", "Triple ramp", "Jitter", "Zigzag" };
static const char* const enumRange[] = { "Slow", "Medium", "Fast" };
static const char* const enumPolarity[] = { "Bipolar", "Unipolar +", "Unipolar -" };

static const _NT_parameter parameters[] = {
	NT_PARAMETER_CV_INPUT( "Rate CV", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Phase CV", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Reset", 0, 0 )
	NT_PARAMETER_CV_OUTPUT_WITH_MODE( "LFO 1 output", 1, 13 )
	NT_PARAMETER_CV_OUTPUT_WITH_MODE( "LFO 2 output", 1, 14 )
	NT_PARAMETER_CV_OUTPUT_WITH_MODE( "LFO 3 output", 1, 15 )
	NT_PARAMETER_CV_OUTPUT_WITH_MODE( "LFO 4 output", 1, 16 )
	NT_PARAMETER_CV_OUTPUT_WITH_MODE( "LFO 5 output", 1, 17 )
	NT_PARAMETER_CV_OUTPUT_WITH_MODE( "LFO 6 output", 1, 18 )

	{ .name = "Rate",        .min = 0,    .max = 1000, .def = 500, .unit = kNT_unitPercent, .scaling = kNT_scaling10, .enumStrings = NULL },
	{ .name = "Range",       .min = 0,    .max = 2,    .def = 1,   .unit = kNT_unitEnum,    .scaling = 0, .enumStrings = enumRange },
	{ .name = "Rate spread", .min = -100, .max = 100,  .def = 0,   .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Phase",       .min = -100, .max = 100,  .def = 0,   .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Shape",       .min = 0,    .max = pl::kNumShapes - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = enumShapes },
	{ .name = "Skew",        .min = 5,    .max = 95,   .def = 50,  .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Depth",       .min = 0,    .max = 100,  .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Polarity",    .min = 0,    .max = 2,    .def = 0,   .unit = kNT_unitEnum,    .scaling = 0, .enumStrings = enumPolarity },
	{ .name = "Offset",      .min = -500, .max = 500,  .def = 0,   .unit = kNT_unitVolts,   .scaling = kNT_scaling100, .enumStrings = NULL },
};
static_assert( ARRAY_SIZE(parameters) == kNumParams, "parameter table and enum are out of sync" );

static const uint8_t pageLfo[]   = { kParamRate, kParamRange, kParamRateSpread, kParamPhase, kParamShape, kParamSkew };
static const uint8_t pageOut[]   = { kParamDepth, kParamPolarity, kParamOffset };
static const uint8_t pageRoute[] = { kParamRateCV, kParamPhaseCV, kParamReset, kParamOut1, kParamOut1Mode, kParamOut2, kParamOut2Mode,
                                     kParamOut3, kParamOut3Mode, kParamOut4, kParamOut4Mode, kParamOut5, kParamOut5Mode, kParamOut6, kParamOut6Mode };
static const _NT_parameterPage pages[] = {
	{ .name = "LFO",     .numParams = ARRAY_SIZE(pageLfo),   .params = pageLfo },
	{ .name = "Output",  .numParams = ARRAY_SIZE(pageOut),   .params = pageOut },
	{ .name = "Routing", .numParams = ARRAY_SIZE(pageRoute), .params = pageRoute },
};
static const _NT_parameterPages parameterPages = { .numPages = ARRAY_SIZE(pages), .pages = pages };

struct _phaseSixAlgorithm : public _NT_algorithm { pl::Bank bank; };

static void calculateRequirements( _NT_algorithmRequirements& req, const int32_t* specifications )
{
	req.numParameters = ARRAY_SIZE(parameters);
	req.sram = sizeof(_phaseSixAlgorithm);
	req.dram = 0; req.dtc = 0; req.itc = 0;
}

static _NT_algorithm* construct( const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements& req, const int32_t* specifications )
{
	_phaseSixAlgorithm* alg = new (ptrs.sram) _phaseSixAlgorithm();
	alg->parameters = parameters;
	alg->parameterPages = &parameterPages;
	alg->bank.init( NT_globals.sampleRate );
	return alg;
}

static void parameterChanged( _NT_algorithm* self, int p ) {}

static inline const float* busIn( const float* busFrames, int param, int numFrames )
{
	return param > 0 ? busFrames + ( param - 1 ) * numFrames : NULL;
}

static const int kOutParam[pl::kChannels] = { kParamOut1, kParamOut2, kParamOut3, kParamOut4, kParamOut5, kParamOut6 };

static void step( _NT_algorithm* self, float* busFrames, int numFramesBy4 )
{
	_phaseSixAlgorithm* a = (_phaseSixAlgorithm*)self;
	const int16_t* v = a->v;
	const int numFrames = numFramesBy4 * 4;

	pl::Params P;
	static const float rangeBase[3] = { 0.001f, 0.05f, 5.f };      // 0.001-1 Hz, 0.05-51 Hz, 5-5120 Hz
	P.rateHz     = rangeBase[v[kParamRange]] * ntm::exp2( 10.f * 0.001f * v[kParamRate] );
	P.rateSpread = 0.01f * v[kParamRateSpread];
	P.phase      = 0.01f * v[kParamPhase];
	P.shape      = v[kParamShape];
	P.skew       = 0.01f * v[kParamSkew];
	P.depth      = 0.01f * v[kParamDepth];
	P.polarity   = v[kParamPolarity];
	P.offsetV    = 0.01f * v[kParamOffset];

	pl::Inputs in;
	in.rateCV  = busIn( busFrames, v[kParamRateCV], numFrames );
	in.phaseCV = busIn( busFrames, v[kParamPhaseCV], numFrames );
	in.reset   = busIn( busFrames, v[kParamReset], numFrames );

	// Render into scratch (frames of at most 64), then write/add to the chosen buses.
	const int kChunk = 64;
	float tmp[pl::kChannels][kChunk];
	float* outp[pl::kChannels];
	for ( int c = 0; c < pl::kChannels; ++c ) outp[c] = tmp[c];

	for ( int off = 0; off < numFrames; off += kChunk )
	{
		int n = numFrames - off; if ( n > kChunk ) n = kChunk;
		pl::Inputs ci = in;
		if ( ci.rateCV ) ci.rateCV += off;
		if ( ci.phaseCV ) ci.phaseCV += off;
		if ( ci.reset ) ci.reset += off;
		a->bank.process( P, ci, outp, n );
		for ( int c = 0; c < pl::kChannels; ++c )
		{
			int bus = v[kOutParam[c]];
			if ( bus <= 0 ) continue;
			float* dst = busFrames + ( bus - 1 ) * numFrames + off;
			if ( v[kOutParam[c] + 1] ) for ( int i = 0; i < n; ++i ) dst[i] = tmp[c][i];
			else                       for ( int i = 0; i < n; ++i ) dst[i] += tmp[c][i];
		}
	}
}

static const _NT_factory factory =
{
	.guid = NT_MULTICHAR( 'P', 'h', 'S', '6' ),
	.name = "Phase Six",
	.description = "Six phase-shiftable LFOs with rate spread and 20 shapes",
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
	case kNT_selector_version:     return kNT_apiVersionCurrent;
	case kNT_selector_numFactories: return 1;
	case kNT_selector_factoryInfo:  return (uintptr_t)( ( data == 0 ) ? &factory : NULL );
	}
	return 0;
}
