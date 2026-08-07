#pragma once

#include "ik_equations/model/FixedRigidTransform.hpp"
#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/symbolic/ExpressionEvaluator.hpp"

#include <array>
#include <vector>

// Independent numeric forward kinematics, used only to cross-check the
// symbolic pipeline. Deliberately NOT part of kinemaforge_ik: the only value
// this code has is being a second implementation, and that value disappears
// the moment anything in the product starts calling it.
//
// Independence is by representation, not by rewording: rotations are carried
// as quaternions and composed by quaternion multiplication, whereas production
// builds Rodrigues matrices and multiplies 3x3 blocks. A 3x3 matrix appears
// here exactly once, when the final result is handed to a test.
//
// What this does NOT protect against: a shared misreading of the URDF spec.
// Both sides get their axes, origins and joint order from the same
// KinematicChain, and this reference was written from the same understanding
// of "rpy" as the production code. Only the hand-computed oracles in
// test_numeric_fk_validation.cpp guard that.
namespace kinemaforge::testsupport {

struct Vector3d
{
    double x{};
    double y{};
    double z{};
};

// (w, x, y, z); the default is the identity rotation.
struct Quaternion
{
    double w{1.0};
    double x{};
    double y{};
    double z{};
};

struct RigidTransform
{
    Quaternion rotation;
    Vector3d translation;
};

using Matrix3 = std::array<std::array<double, 3>, 3>;

// One value per actuated joint, in chain order. Deliberately positional: the
// reference must not address joints by the same symbol names the symbolic side
// uses, or a duplicated name would be confirmed rather than caught.
using JointConfiguration = std::vector<double>;

Quaternion multiply(const Quaternion& lhs, const Quaternion& rhs);
Quaternion fromAxisAngle(const ik::Vector3& axis, double angle);
Quaternion fromRollPitchYaw(const ik::Vector3& rpy);
double norm(const Quaternion& quaternion);

// Rotates a vector without ever forming a rotation matrix.
Vector3d rotate(const Quaternion& quaternion, const Vector3d& vector);

RigidTransform compose(const RigidTransform& lhs, const RigidTransform& rhs);

// The one and only place a matrix is built.
Matrix3 toRotationMatrix(const Quaternion& quaternion);

RigidTransform numericForwardKinematics(const ik::KinematicChain& chain,
                                        const JointConfiguration& configuration);

// The symbolic side's view of the same configuration, addressed by name.
// Kept next to numericForwardKinematics on purpose: both walk the chain the
// same way, and sharing one isActuated plus one consumption order is what
// keeps the positional and the by-name mapping from drifting apart.
ik::SymbolValues makeSymbolValues(const ik::KinematicChain& chain,
                                  const JointConfiguration& configuration);

// Forward kinematics to the TCP, composed the same way the symbolic side does
// it but in a different representation:
//
//     T_base_tcp = T_base_tip * T_tip_tcp
//
// compose() implements p = p_a + rotate(q_a, p_b), which is exactly the
// tool-frame semantics -- so this validates the composition order
// independently, with quaternions instead of matrices.
RigidTransform numericTcpForwardKinematics(const ik::KinematicChain& chain,
                                           const JointConfiguration& configuration,
                                           const ik::FixedRigidTransform& tcp);

// Alternating lower + 5% of span, upper - 5% of span, derived from the loaded
// model rather than hard-coded. Deliberately not the exact limits: this
// validates kinematics, not boundary handling.
JointConfiguration nearLimitConfiguration(const ik::KinematicChain& chain);

} // namespace kinemaforge::testsupport
