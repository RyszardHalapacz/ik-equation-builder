#include "support/NumericForwardKinematics.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace kinemaforge::testsupport {

namespace {

Vector3d cross(const Vector3d& lhs, const Vector3d& rhs)
{
    return {lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x};
}

Vector3d add(const Vector3d& lhs, const Vector3d& rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vector3d scale(const Vector3d& vector, double factor)
{
    return {vector.x * factor, vector.y * factor, vector.z * factor};
}

Vector3d toVector3d(const ik::Vector3& vector)
{
    return {vector.x, vector.y, vector.z};
}

bool isActuated(ik::JointType type)
{
    return type == ik::JointType::Revolute
        || type == ik::JointType::Continuous
        || type == ik::JointType::Prismatic;
}

} // namespace

Quaternion multiply(const Quaternion& lhs, const Quaternion& rhs)
{
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w
    };
}

Quaternion fromAxisAngle(const ik::Vector3& axis, double angle)
{
    const double half = 0.5 * angle;
    const double sine = std::sin(half);
    return {std::cos(half), axis.x * sine, axis.y * sine, axis.z * sine};
}

Quaternion fromRollPitchYaw(const ik::Vector3& rpy)
{
    // URDF fixed-axis convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    const Quaternion roll = fromAxisAngle(ik::Vector3{1.0, 0.0, 0.0}, rpy.x);
    const Quaternion pitch = fromAxisAngle(ik::Vector3{0.0, 1.0, 0.0}, rpy.y);
    const Quaternion yaw = fromAxisAngle(ik::Vector3{0.0, 0.0, 1.0}, rpy.z);
    return multiply(multiply(yaw, pitch), roll);
}

double norm(const Quaternion& quaternion)
{
    return std::sqrt(quaternion.w * quaternion.w + quaternion.x * quaternion.x +
                     quaternion.y * quaternion.y + quaternion.z * quaternion.z);
}

Vector3d rotate(const Quaternion& quaternion, const Vector3d& vector)
{
    // v + 2w(u x v) + 2u x (u x v), with u the vector part.
    //
    // Deliberately not "convert to a matrix and multiply": that would be the
    // production representation, and a shared mistake could then survive in
    // both places.
    const Vector3d vectorPart{quaternion.x, quaternion.y, quaternion.z};
    const Vector3d twiceCross = scale(cross(vectorPart, vector), 2.0);
    return add(add(vector, scale(twiceCross, quaternion.w)),
               cross(vectorPart, twiceCross));
}

RigidTransform compose(const RigidTransform& lhs, const RigidTransform& rhs)
{
    return {multiply(lhs.rotation, rhs.rotation),
            add(lhs.translation, rotate(lhs.rotation, rhs.translation))};
}

Matrix3 toRotationMatrix(const Quaternion& quaternion)
{
    const double w = quaternion.w;
    const double x = quaternion.x;
    const double y = quaternion.y;
    const double z = quaternion.z;

    return Matrix3{{
        {{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),       2.0 * (x * z + w * y)}},
        {{2.0 * (x * y + w * z),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)}},
        {{2.0 * (x * z - w * y),       2.0 * (y * z + w * x),       1.0 - 2.0 * (x * x + y * y)}}
    }};
}

RigidTransform numericForwardKinematics(const ik::KinematicChain& chain,
                                        const JointConfiguration& configuration)
{
    RigidTransform result;   // identity
    std::size_t next = 0;

    for (const auto& joint : chain.joints)
    {
        const RigidTransform origin{fromRollPitchYaw(joint.origin.rpy),
                                    toVector3d(joint.origin.translation)};

        if (!isActuated(joint.type))
        {
            result = compose(result, origin);
            continue;
        }

        assert(next < configuration.size() &&
               "configuration has fewer values than the chain has actuated joints");
        const double value = configuration[next++];

        RigidTransform motion;   // identity
        if (joint.type == ik::JointType::Prismatic)
            motion.translation = scale(toVector3d(joint.axis), value);
        else
            motion.rotation = fromAxisAngle(joint.axis, value);

        result = compose(result, compose(origin, motion));
    }

    assert(next == configuration.size() && "configuration has unused trailing values");
    return result;
}

ik::SymbolValues makeSymbolValues(const ik::KinematicChain& chain,
                                  const JointConfiguration& configuration)
{
    ik::SymbolValues values;
    std::size_t next = 0;

    for (const auto& joint : chain.joints)
    {
        if (!isActuated(joint.type))
            continue;

        assert(joint.variable.has_value() && "an actuated joint must carry a variable");
        assert(next < configuration.size() &&
               "configuration has fewer values than the chain has actuated joints");

        const auto [iterator, inserted] =
            values.emplace(joint.variable->name, configuration[next++]);
        (void) iterator;

        // A duplicate name means the KinematicChain contract is broken, not
        // that this configuration is malformed -- so it throws rather than
        // asserting, and stays live under NDEBUG.
        //
        // Detecting it here matters: emplace does NOT overwrite on a
        // duplicate, it simply does not insert. The symbolic side would then
        // silently evaluate two joints from one binding, and if the two
        // configuration values happened to be equal the matrix comparison
        // would not notice at all.
        if (!inserted)
            throw std::logic_error("duplicate actuated-joint variable name: " +
                                   joint.variable->name);
    }

    assert(next == configuration.size() && "configuration has unused trailing values");
    return values;
}

RigidTransform numericTcpForwardKinematics(const ik::KinematicChain& chain,
                                           const JointConfiguration& configuration,
                                           const ik::FixedRigidTransform& tcp)
{
    return compose(numericForwardKinematics(chain, configuration),
                   RigidTransform{fromRollPitchYaw(tcp.rpy),
                                  toVector3d(tcp.translation)});
}

JointConfiguration nearLimitConfiguration(const ik::KinematicChain& chain)
{
    JointConfiguration configuration;
    bool useLower = true;

    for (const auto& joint : chain.joints)
    {
        if (!joint.variable) continue;

        const double span = joint.limits.upper - joint.limits.lower;
        configuration.push_back(useLower ? joint.limits.lower + 0.05 * span
                                         : joint.limits.upper - 0.05 * span);
        useLower = !useLower;
    }
    return configuration;
}

} // namespace kinemaforge::testsupport
