#include "ik_equations/UrdfModelLoader.hpp"

#include "kinematics/robot_model_loader.hpp"

#include <stdexcept>
#include <string>

namespace kinemaforge::ik {

namespace {

// Two protections, covering different things:
//
//   * no `default` -- adding an enumerator to mt::kinematics::JointType makes
//     -Wswitch fire, which -Werror turns into a build failure in CI. This is
//     what must stop `continuous` from silently mapping to Fixed once the
//     lower layer learns about it. Previously this function ended in
//     `return JointType::Fixed;`, which made exactly that regression silent.
//   * the throw -- a value forged with static_cast has no enumerator to be
//     caught at compile time. std::logic_error, not a LoadErrorCode: the lower
//     layer's parse_joint_type already rejected every string it does not know,
//     so a value arriving here is a defect in this repository, not in the
//     user's file. logic_error does NOT derive from std::runtime_error, which
//     is exactly what IkEquationBuilder catches, so it propagates to the caller
//     instead of being reported as a URDF problem.
//
// Neither protection replaces the other. Same shape as componentIndex() in
// IkEquationSystem.cpp, which rejects rather than throws because there the
// caller has a std::expected to carry the error.
//
// The throw is unreachable through any public path: load() is the only caller,
// and parse_joint_type admits exactly three strings. It is defence in depth
// against a future path that skips that validation.
JointType mapJointType(mt::kinematics::JointType type)
{
    switch (type)
    {
    case mt::kinematics::JointType::revolute:  return JointType::Revolute;
    case mt::kinematics::JointType::prismatic: return JointType::Prismatic;
    case mt::kinematics::JointType::fixed:     return JointType::Fixed;
    }

    throw std::logic_error(
        "UrdfModelLoader: unmapped mt::kinematics::JointType value " +
        std::to_string(static_cast<int>(type)));
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

std::string describeCode(mt::kinematics::LoadErrorCode code)
{
    using Code = mt::kinematics::LoadErrorCode;
    switch (code)
    {
    case Code::file_not_found:             return "URDF file not found";
    case Code::parse_failure:              return "failed to parse URDF XML";
    case Code::unsupported_joint_type:     return "unsupported joint type in URDF";
    case Code::malformed_vector:           return "malformed vector attribute";
    case Code::degenerate_axis:            return "actuated joint has a zero-length axis";
    case Code::missing_limit_element:      return "actuated joint has no <limit> element";
    case Code::duplicate_limit_element:    return "joint has more than one <limit> element";
    case Code::missing_limit_attribute:    return "required joint limit attribute is missing";
    case Code::malformed_limit_value:      return "joint limit value is not a single finite number";
    case Code::invalid_limit_constraint:   return "joint limits violate a required relation";
    case Code::invalid_link_declaration:   return "link declaration has an empty or duplicate name";
    case Code::undeclared_parent_link:     return "joint parent refers to a link that is not declared";
    case Code::undeclared_child_link:      return "joint child refers to a link that is not declared";
    case Code::duplicate_child_link:       return "link is the child of more than one joint";
    case Code::cyclic_topology:            return "URDF topology contains a cycle";
    case Code::disconnected_topology:      return "URDF does not describe one connected tree";
    }
    return "unknown URDF load error";
}

// The lower layer returns data, not prose; the message is assembled here.
std::string describe(const mt::kinematics::LoadError& error)
{
    std::string message = describeCode(error.code);
    if (!error.jointName.empty())
        message += " in joint '" + error.jointName + "'";
    if (!error.attribute.empty())
        message += ", attribute '" + error.attribute + "'";
    // Printed whenever a raw value applies, including when it is empty —
    // xyz="" is exactly the case a reader needs to see quoted.
    if (error.rawValue)
        message += ": \"" + *error.rawValue + "\"";
    return message;
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
