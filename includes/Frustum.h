#pragma once
#include "GenericDefines.h"
#include "VrixicMath.h"

struct Frustum
{
	enum {
		TOP = 0, BOTTOM, LEFT,
		RIGHT, NEARP, FARP
	};

	Plane Planes[6];

	/*
	*  WidthMultiplier -> scales the width of the frustum
	*/
	float AspectRatio, WidthMultiplier, NearPlaneDist, FarPlaneDist;

	float FarPlaneHeight, FarPlaneWidth;
	float NearPlaneHeight, NearPlaneWidth;

	/* For Debug reasons/ visuals */
#if _DEBUG
	Vector3D FarPlaneTopLeft, FarPlaneTopRight, FarPlaneBottomLeft, FarPlaneBottomRight;
	Vector3D NearPlaneTopLeft, NearPlaneTopRight, NearPlaneBottomLeft, NearPlaneBottomRight;

	Vector3D PlaneCenters[6];
#endif // _DEBUG


private:
	float WidthMultiplierRecip;

public:
	Frustum() : AspectRatio(0.0f), WidthMultiplier(0.0f), NearPlaneDist(0.0f), FarPlaneDist(0.0f),
				FarPlaneHeight(0.0f), FarPlaneWidth(0.0f), NearPlaneHeight(0.0f), NearPlaneWidth(0.0f),
				WidthMultiplierRecip(0.0f) { }

	Frustum(float aspectRatio, float widthMultiplier, float nearPlaneDist, float farPlaneDist)
	{
		SetFrustumInternals(aspectRatio, widthMultiplier, nearPlaneDist, farPlaneDist);
	}

public:
	void SetFrustumInternals(float aspectRatio, float widthMultiplier, float nearPlaneDist, float farPlaneDist)
	{
		AspectRatio = aspectRatio;
		WidthMultiplier = widthMultiplier;
		WidthMultiplierRecip = 1.0f / widthMultiplier;
		NearPlaneDist = nearPlaneDist;
		FarPlaneDist = farPlaneDist;

		RecalculateFustrumInternals();
	}

	void CreateFrustum(const Matrix4D& camModel)
	{
		Vector3D CameraPosition = camModel[3].ToVector3D();
		Vector3D CameraRight = camModel[0].ToVector3D();
		Vector3D CameraUp = camModel[1].ToVector3D();
		Vector3D CameraForward = camModel[2].ToVector3D();

		Vector3D FarPlaneCenter = CameraPosition + CameraForward * FarPlaneDist;
		Vector3D NearPlaneCenter = CameraPosition + CameraForward * NearPlaneDist;

		Vector3D FTL = FarPlaneCenter + (CameraUp * FarPlaneHeight * 0.5f) - (CameraRight * FarPlaneWidth * 0.5f);
		Vector3D FTR = FarPlaneCenter + (CameraUp * FarPlaneHeight * 0.5f) + (CameraRight * FarPlaneWidth * 0.5f);
		Vector3D FBL = FarPlaneCenter - (CameraUp * FarPlaneHeight * 0.5f) - (CameraRight * FarPlaneWidth * 0.5f);
		Vector3D FBR = FarPlaneCenter - (CameraUp * FarPlaneHeight * 0.5f) + (CameraRight * FarPlaneWidth * 0.5f);

		Vector3D NTL = NearPlaneCenter + (CameraUp * NearPlaneHeight * 0.5f) - (CameraRight * NearPlaneWidth * 0.5f);
		Vector3D NTR = NearPlaneCenter + (CameraUp * NearPlaneHeight * 0.5f) + (CameraRight * NearPlaneWidth * 0.5f);
		Vector3D NBL = NearPlaneCenter - (CameraUp * NearPlaneHeight * 0.5f) - (CameraRight * NearPlaneWidth * 0.5f);
		Vector3D NBR = NearPlaneCenter - (CameraUp * NearPlaneHeight * 0.5f) + (CameraRight * NearPlaneWidth * 0.5f);

		/* Keeps the corners for debug visualizations */
#if _DEBUG
		FarPlaneTopLeft = FTL;
		FarPlaneTopRight = FTR;
		FarPlaneBottomLeft = FBL;
		FarPlaneBottomRight = FBR;

		NearPlaneTopLeft = NTL;
		NearPlaneTopRight = NTR;
		NearPlaneBottomLeft = NBL;
		NearPlaneBottomRight = NBR;
#endif

		MakePlaneFromThreePoints(FBL, FTL, FTR, Planes[FARP]);
		MakePlaneFromThreePoints(NTR, NTL, NBL, Planes[NEARP]);
		MakePlaneFromThreePoints(FTR, FTL, NTL, Planes[TOP]);
		MakePlaneFromThreePoints(NBL, FBL, FBR, Planes[BOTTOM]);
		MakePlaneFromThreePoints(NBL, FTL, FBL, Planes[LEFT]);
		MakePlaneFromThreePoints(FBR, FTR, NTR, Planes[RIGHT]);

#if _DEBUG
		/* Debug Visualization */
		PlaneCenters[FARP] = (FBL + FTL + FTR + FBR) * 0.25f;
		PlaneCenters[NEARP] = (NBL + NTL + NTR + NBR) * 0.25f;
		PlaneCenters[TOP] = (NTL + FTL + FTR + NTR) * 0.25f;
		PlaneCenters[BOTTOM] = (NBL + FBL + FBR + NBR) * 0.25f;
		PlaneCenters[LEFT] = (NBL + NTL + FTL + FBL) * 0.25f;
		PlaneCenters[RIGHT] = (NBR + NTR + FTR + FBR) * 0.25f;
#endif
	}

	PlaneIntersectionResult TestAABB(const Vector3D& aabbMin, const Vector3D& aabbMax)
	{
		PlaneIntersectionResult Result;
		for (uint32 i = 0; i < 6; ++i)
		{
			if (VrixicMath::IntersectAABBOnPlane(aabbMin, aabbMax, Planes[i]) == PlaneIntersectionResult::Back)
			{
				return PlaneIntersectionResult::Back;
			}
		}

		return PlaneIntersectionResult::Front;
	}

private:
	void RecalculateFustrumInternals()
	{
		NearPlaneHeight = NearPlaneDist * WidthMultiplierRecip;
		NearPlaneWidth = NearPlaneHeight * AspectRatio;

		FarPlaneHeight = FarPlaneDist * WidthMultiplierRecip;
		FarPlaneWidth = FarPlaneHeight * AspectRatio;
	}

	inline static void MakePlaneFromThreePoints(const Vector3D& a, const Vector3D& b, const Vector3D& c, Plane& outPlane)
	{
		Vector3D EdgeA = b - a;
		Vector3D EdgeB = c - b;
		Vector3D Normal = Vector3D::CrossProduct(EdgeA, EdgeB);
		Normal.Normalize();

		outPlane.X = Normal.X;
		outPlane.Y = Normal.Y;
		outPlane.Z = Normal.Z;

		outPlane.Distance = Vector3D::DotProduct(Normal, a);
	}
};

