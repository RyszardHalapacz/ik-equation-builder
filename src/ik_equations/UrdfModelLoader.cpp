#include "ik_equations/UrdfModelLoader.hpp"

#include "kinematics/robot_model_loader.hpp"

#include <stdexcept>
#include <string>

namespace kinemaforge::ik {

namespace {

JointType mapJointType(mt::kinematics::JointType type)
{
    switch (type)
    {
    case mt::kinematics::JointType::revolute:  return JointType::Revolute;
    case mt::kinematics::JointType::prismatic: return JointType::Prismatic;
    case mt::kinematics::JointType::fixed:      return JointType::Fixed;
    }
    return JointType::Fixed;
}

Vector3 mapVector3(mt::kinematics::Vec3 const& v)
{
    return Vector3{v.x, v.y, v.z};
}

UrdfJoint mapJoint(mt::kinematics::Joint const& joint)
{
    UrdfJoint result;
    result.name = joint.name;
    result.parentLink = joint.parent_link;
    result.childLink = joint.child_link;
    result.type = mapJointType(joint.type);
    result.origin.translation = mapVector3(joint.origin_xyz);
    result.origin.rpy = mapVector3(joint.origin_rpy);
    result.axis = mapVector3(joint.axis);
    result.limits.lower = joint.limits.lower;
    result.limits.upper = joint.limits.upper;
    result.limits.velocity = joint.limits.velocity;
    result.limits.effort = joint.limits.effort;
    // The copied loader requires <limit> for every non-fixed joint, so
    // presence of position limits tracks 1:1 with "is actuated".
    result.limits.hasPositionLimits = (result.type != JointType::Fixed);
    return result;
}

std::string describe(mt::kinematics::LoadError error)
{
    using mt::kinematics::LoadError;
    switch (error)
    {
    case LoadError::file_not_found:             return "URDF file not found";
    case LoadError::parse_failure:              return "failed to parse URDF XML";
    case LoadError::unsupported_joint_type:     return "unsupported joint type in URDF";
    case LoadError::incomplete_kinematic_chain: return "URDF does not describe a complete kinematic chain";
    case LoadError::invalid_limits:             return "invalid or missing joint limits in URDF";
    }
    return "unknown URDF load error";
}

} // namespace

RobotDescription UrdfModelLoader::load(const std::filesystem::path& urdfPath) const
{
    auto result = mt::kinematics::load_urdf(urdfPath);
    if (!result)
    {
        throw std::runtime_error(
            "UrdfModelLoader: " + describe(result.error()) + " (" + urdfPath.string() + ")");
    }

    RobotDescription description;
    description.name = std::string(result->model.name());

    for (auto const& link : result->model.links())
        description.links.push_back(UrdfLink{link.name});

    for (auto const& joint : result->model.joints())
        description.joints.push_back(mapJoint(joint));

    return description;
}

} // namespace kinemaforge::ik
