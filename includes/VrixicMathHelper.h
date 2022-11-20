#pragma once

#include <cstdlib>

class Vector3D;
class Vector4D;
class Matrix4D;
//class Plane;

#define PI (3.1415926535897932f)

/* Smallest float point number  */
#define EPSILON (1.192092896e-07f)

static constexpr float DEGTORADS = PI / 180.0f;
static constexpr float RADSTODEG = 180.0f / PI;

/* --- */
enum PlaneIntersectionResult
{
	Back = -1,
	Intersection, // Straddling
	Front
};

/* These Functions can be implemented in different locations if not implemented in here */
struct MathUtils
{
public:
	template<class T>
	/* Return minimum between a and b */
	inline static T Min(T a, T b)
	{
		return a < b ? a : b;
	}

	template<class T>
	/* Return maximum between a and b */
	inline static T Max(T a, T b)
	{
		return a > b ? a : b;
	}

	template<class T>
	inline static T Clamp(T min, T max, T test)
	{
		T r = test;
		if (test > max)
		{
			r = max;
		}
		else if (test < min)
		{
			r = min;
		}

		return r;
	}

	inline static float RandomRange(float min, float max)
	{
		float r = rand() / (float)RAND_MAX;
		return (max - min) * r + min;
	}

	inline static float Lerp(float start, float end, float ratio)
	{
		return static_cast<float>(end - start) * ratio + start;
	}

	/* Converts Degrees to radians */
	inline static float DegreesToRadians(float degrees)
	{
		return (DEGTORADS * degrees);
	}

	inline static float RadiansToDegrees(float radians)
	{
		return (RADSTODEG * radians);
	}
};
