#include <iostream>
#include "../../includes/VrixicMath.h"
#include "../../includes/ProjectionMatrix4D.h"
#include "../../includes/Quat.h"

/** SandBox -> Testing math operations **/

int main()
{
    using namespace Vrixic::Math;
    Vector3D v(45.0f);

    Matrix4D m1 = Matrix4D::MakeRotX(v.X) * Matrix4D::MakeRotY(v.Y) * Matrix4D::MakeRotZ(v.Z);

    ProjectionMatrix4D P0;
    ProjectionMatrix4D P1 = ProjectionMatrix4D::MakeProjectionDirectXLH(800.0f / 600.0f, 65.0f, 0.1f, 100.0f);
    ProjectionMatrix4D P2 = ProjectionMatrix4D::MakeProjectionDirectXRH(800.0f / 600.0f, 65.0f, 0.1f, 100.0f);
    ProjectionMatrix4D P3 = ProjectionMatrix4D::MakeProjectionVulkanLH(800.0f / 600.0f, 65.0f, 0.1f, 100.0f);

    Matrix4D d1(1, 1, 1, 1, 2, 3, 1, 1, 4, 5, 1, 1, 6, 3, 5, 1);
    std::cout << d1.Determinant() << std::endl;

    //Quat Q1(1.0f, 2.0f, 3.0f, 1.0f);
    //Quat Q2(4.0f, 5.0f, 6.0f, 1.0f);
    //Quat Q3 = Q1 * Q2;
    //Quat Q4 = Q1 * Q2 * Q3;
    //Quat Q5 = Q3.Inverse();
    //Q1 *= Q2;

    //Q3.Normalize();

    Quat Q1 = Quat::MakeRotationQuat(0, 1, 0, 90.0f); // Rotate 90 degrees on yaw (y)
    Quat Q2 = Quat::MakeRotationQuat(1, 0, 0, 45.0f); // Then rotate 45 degress on pitch (x)
    Quat Q3 = Q2 * Q1; // First rotation should go last as quaternion rotation order matters

    /* rotate Vector by quaternion Q3 */
    Vector3D V1(1, 0, 0);
    Vector3D RotateV1WithQ1 = Q1.RotateVector(V1);
    Vector3D RotatedV1WithQ2 = Q2.RotateVector(RotateV1WithQ1);
    Vector3D RotateV1WithQ3_Slow = Q3.RotateVectorSlow(V1);
    Vector3D RotateV1WithQ3_Fast = Q3.RotateVector(V1);

    return 0;
}