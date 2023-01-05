#pragma once
#include "Vector3D.h"

namespace Vrixic
{
	namespace Math
	{
		/* Quaternion representations -> [w, vector] -> [w, x, y, z]*/
		/* For a given vector V which is the axis of rotation and angleInDegrees w, Quat = [V, w] */
		struct Quat
		{
		public:
			float X;
			float Y;
			float Z;
			float W;

		public:
			inline Quat();

			inline Quat(float x, float y, float z, float w);

			inline Quat(const Vector3D& v, float w);

		public:
			/* Unary operator overloads */

			inline Quat operator+(const Quat& q) const;

			inline Quat operator-(const Quat& q) const;

			/* Returns the negated copy of vector */
			inline Quat operator-() const;

			/* Product of 2 Quaternionss: Q1 * Q2 = [w1 * w2 - Dot(v1, v2), w1*v2 + w2*v1 + Cross(v1, v2)] */
			inline Quat operator*(const Quat& q) const;

			inline Quat operator+=(const Quat& q);

			inline Quat operator-=(const Quat& q);

			inline Quat operator*=(const Quat& q);

		public:
			/* Factory function for making a rotation quat with an axis and an angleInDegrees */
			inline static Quat MakeRotationQuat(float axisX, float axisY, float axisZ, float angleInDegrees);

			/* Factory function for making a rotation quat with an axis and an angleInDegrees */
			inline static Quat MakeRotationQuat(const Vector3D& axis, float angleInDegrees);

			inline static Quat Identity();

			inline static float DotProduct(const Quat& a, const Quat& b);

			inline void SetIdentity();

			inline float Length() const;

			inline float LengthSquared() const;

			inline const Quat& Normalize();

			/*
			* Returns the conjugate of this quaternion
			* 
			* if |Quat| = 1 (meaning quaternion is normalized), the Inverse() = Conjugate()
			*/
			inline Quat Conjugate() const;

			/* 
			* return Inverse of quaternion 
			* 
			* if |Quat| = 1 (meaning quaternion is normalized), the Inverse() = Conjugate()
			*/
			inline Quat Inverse() const;

			/* 
			* Faster as Inverse() = Conjugate(), it is implied that Quat is normalized 
			* 
			* Passive Rotation -> Coordinate system is rotated with respect to the point
			* Rotate a vector by this quaternion -> for a point q and quat q -> qpq^-1
			*/
			inline Vector3D RotateVector(const Vector3D& v) const;

			/* 
			* Slower as it calculates the Inverse() 
			* 
			* Passive Rotation -> Coordinate system is rotated with respect to the point
			* Rotate a vector by this quaternion -> for a point q and quat q -> qpq^-1
			*/
			inline Vector3D RotateVectorSlow(const Vector3D& v) const;
		};

		inline Quat::Quat()
			: X(0.0f), Y(0.0f), Z(0.0f), W(0.0f) { }

		inline Quat::Quat(const Vector3D& v, float w)
			: X(v.X), Y(v.Y), Z(v.Z), W(w) { } 

		inline Quat::Quat(float x, float y, float z, float w)
			: X(x), Y(y), Z(z), W(w) { }

		inline Quat Quat::operator+(const Quat& q) const 
		{
			return Quat(X + q.X, Y + q.Y, Z + q.Z, W + q.W);
		}

		inline Quat Quat::operator-(const Quat& q) const 
		{
			return Quat(X - q.X, Y - q.Y, Z - q.Z, W - q.W);
		}

		inline Quat Quat::operator-() const 
		{
			return Quat(-X, -Y, -Z, -W);
		}

		inline Quat Quat::operator*(const Quat& q) const 
		{
			/* Q1 * Q2 = [w1 * w2 - Dot(v1, v2), w1*v2 + w2*v1 + Cross(v1, v2)] */
			float ResultX = W * q.X + q.W * X + Y * q.Z - Z * q.Y;
			float ResultY = W * q.Y + q.W * Y + Z * q.X - X * q.Z;
			float ResultZ = W * q.Z + q.W * Z + X * q.Y - Y * q.X;
			float ResultW = W * q.W - (X * q.X + Y * q.Y + Z * q.Z);

			return Quat(ResultX, ResultY, ResultZ, ResultW);
		}

		inline Quat Quat::operator+=(const Quat& q) 
		{
			X += q.X;
			Y += q.Y;
			Z += q.Z;
			W += q.W;

			return *this;
		}

		inline Quat Quat::operator-=(const Quat& q) 
		{
			X -= q.X;
			Y -= q.Y;
			Z -= q.Z;
			W -= q.W;

			return *this;
		}

		inline Quat Quat::operator*=(const Quat& q) 
		{
			float ResultX = W * q.X + q.W * X + Y * q.Z - Z * q.Y;
			float ResultY = W * q.Y + q.W * Y + Z * q.X - X * q.Z;
			float ResultZ = W * q.Z + q.W * Z + X * q.Y - Y * q.X;
			float ResultW = W * q.W - (X * q.X + Y * q.Y + Z * q.Z);

			X = ResultX;
			Y = ResultY;
			Z = ResultZ;
			W = ResultW;

			return *this;
		}

		inline Quat Quat::MakeRotationQuat(float axisX, float axisY, float axisZ, float angleInDegrees)
		{
			float HalfAngleInRads = MathUtils::DegreesToRadians(angleInDegrees * 0.5f);

			float C = cos(HalfAngleInRads);
			float S = sin(HalfAngleInRads);

			return Quat(axisX * S, axisY * S, axisZ * S, C);
		}

		inline Quat Quat::MakeRotationQuat(const Vector3D& axis, float angleInDegrees)
		{
			float HalfAngleInRads = MathUtils::DegreesToRadians(angleInDegrees * 0.5f);

			float C = cos(HalfAngleInRads);
			float S = sin(HalfAngleInRads);

			return Quat(axis.X * S, axis.Y * S, axis.Z * S, C);
		}

		inline Quat Quat::Identity()
		{
			return Quat(0.0f, 0.0f, 0.0f, 1.0f);
		}

		inline void Quat::SetIdentity()
		{
			X = 0.0f;
			Y = 0.0f;
			Z = 0.0f;
			W = 1.0f;
		}

		inline float Quat::DotProduct(const Quat& a, const Quat& b)
		{
			return a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W;
		}

		inline float Quat::Length() const
		{
			return sqrtf(X * X + Y * Y + Z * Z + W * W);
		}

		inline float Quat::LengthSquared() const
		{
			return (X * X + Y * Y + Z * Z + W * W);
		}

		inline const Quat& Quat::Normalize()
		{
			const float Magnitude = (1.0f / (Length() + EPSILON));

			X *= Magnitude;
			Y *= Magnitude;
			Z *= Magnitude;
			W *= Magnitude;

			return *this;
		}

		inline Quat Quat::Conjugate() const
		{
			return Quat(-X, -Y, -Z, W);
		}

		inline Quat Quat::Inverse() const
		{
			Quat Con = Conjugate();
			float LenRecip = 1.0f / LengthSquared();

			return Quat(Con.X * LenRecip, Con.Y * LenRecip, Con.Z * LenRecip, Con.W * LenRecip);
		}

		inline Vector3D Quat::RotateVector(const Vector3D& v) const
		{
			Quat ToRotate = Quat(v, 0.0f);
			Quat Inv = Conjugate();

			Quat Result = *this * ToRotate * Inv;

			return Vector3D(Result.X, Result.Y, Result.Z);
		}

		inline Vector3D Quat::RotateVectorSlow(const Vector3D& v) const
		{
			Quat ToRotate = Quat(v, 0.0f);
			Quat Inv = Inverse();

			Quat Result = *this * ToRotate * Inv;

			return Vector3D(Result.X, Result.Y, Result.Z);
		}
	}
}
