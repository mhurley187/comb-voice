// nt_math.h - self-contained math so the plug-in has NO libm/libc dependencies.
// A disting NT plug-in is a relocatable object whose undefined symbols must be resolved by the
// firmware's loader; not every libm/libc function is guaranteed to be exported. Everything the DSP
// needs is implemented here instead (accuracy is checked against libm in test_math.cpp).
#pragma once
#include <stdint.h>

namespace ntm {

static const float kPi = 3.14159265f;
static const float kHalfPi = 1.57079633f;
static const float kTwoPi = 6.28318531f;

inline float fabs(float x) { return x < 0.f ? -x : x; }

// floor for |x| < 2^31
inline float floor(float x) { int i = (int)x; return (float)(i - (x < (float)i ? 1 : 0)); }

// 2^x, x clamped to [-126, 126]. Degree-7 polynomial on the fraction.
inline float exp2(float x)
{
	x = x < -126.f ? -126.f : (x > 126.f ? 126.f : x);
	float fl = floor(x);
	float f = x - fl;
	float p = 1.f + f * (0.69314718f + f * (0.24022651f + f * (0.05550411f + f * (0.00961813f
	        + f * (0.00133336f + f * (0.00015404f + f * 0.0000152527f))))));
	union { float f; uint32_t u; } s;
	s.u = (uint32_t)((int)fl + 127) << 23;
	return p * s.f;
}

inline float exp(float x) { return exp2(x * 1.44269504f); }

// sin: fold into [-pi/2, pi/2], then an odd polynomial
inline float sin(float x)
{
	float t = x * 0.15915494f;                                   // in turns
	t -= (float)(int)(t + (t >= 0.f ? 0.5f : -0.5f));            // [-0.5, 0.5]
	if (t > 0.25f) t = 0.5f - t; else if (t < -0.25f) t = -0.5f - t;
	float a = t * kTwoPi, a2 = a * a;
	return a * (1.f + a2 * (-0.16666667f + a2 * (0.00833333f + a2 * (-0.000198413f
	     + a2 * (0.0000027557f + a2 * -0.0000000250521f)))));
}
inline float cos(float x) { return sin(x + kHalfPi); }

// atan2 (Abramowitz & Stegun 4.4.49)
inline float atan2(float y, float x)
{
	float ax = fabs(x), ay = fabs(y);
	if (ax == 0.f && ay == 0.f) return 0.f;
	float z = (ay <= ax) ? ay / ax : ax / ay, z2 = z * z;
	float a = z * (0.9998660f + z2 * (-0.3302995f + z2 * (0.1801410f + z2 * (-0.0851330f + z2 * 0.0208351f))));
	if (ay > ax) a = kHalfPi - a;
	if (x < 0.f) a = kPi - a;
	if (y < 0.f) a = -a;
	return a;
}

// sqrt: bit-hack seed + 3 Newton steps
inline float sqrt(float x)
{
	if (x <= 0.f) return 0.f;
	union { float f; uint32_t u; } s;
	s.f = x;
	s.u = 0x1fbd1df5u + (s.u >> 1);
	float y = s.f;
	y = 0.5f * (y + x / y);
	y = 0.5f * (y + x / y);
	y = 0.5f * (y + x / y);
	return y;
}

} // namespace ntm
