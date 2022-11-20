#include <iostream>
#include "../../includes/VrixicMath.h"
#include "../../includes/ProjectionMatrix4D.h"

/** SandBox -> Testing math operations **/

#define VM Vrixic::Math

int main()
{
    VM::Vector3D v(45.0f);

    VM::Matrix4D m1 = VM::Matrix4D::MakeRotX(v.X) * VM::Matrix4D::MakeRotY(v.Y) * VM::Matrix4D::MakeRotZ(v.Z);

    VM::ProjectionMatrix4D P1 = VM::ProjectionMatrix4D::MakeProjectionDirectXLH(800.0f / 600.0f, 65.0f, 0.1f, 100.0f);
    VM::ProjectionMatrix4D P2 = VM::ProjectionMatrix4D::MakeProjectionDirectXRH(800.0f / 600.0f, 65.0f, 0.1f, 100.0f);
    VM::ProjectionMatrix4D P3 = VM::ProjectionMatrix4D::MakeProjectionVulkanLH(800.0f / 600.0f, 65.0f, 0.1f, 100.0f);

    VM::Matrix4D d1(1, 1, 1, 1, 2, 3, 1, 1, 4, 5, 1, 1, 6, 3, 5, 1);
    std::cout << d1.Determinant() << std::endl;

    return 0;
}