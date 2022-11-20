#pragma once

#include "VrixicMathHelper.h"

#include "Vector3D.h"
#include "Vector4D.h"
#include "Matrix4D.h"
#include "Plane.h"

namespace Vrixic
{
	namespace Math
	{
		/* Manhattan distance -> non-accurate, but fast distance calculation*/
		inline static float ManhattanDistance(const Vector3D& v1, const Vector3D& v2);
	}
}
