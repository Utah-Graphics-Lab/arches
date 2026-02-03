#pragma once

#include <cstdint>

typedef unsigned int uint;

inline float min(int a, int b) { return (b < a) ? b : a; }
inline float max(int a, int b) { return (a < b) ? b : a; }

static float f32_to_bf24(float f)
{
	uint u = *(uint*)&f;
	if(f < 0) u += 255;
	u &= 0xffffff00;
	return *(float*)&u;
}