#pragma once

#include "ik_equations/model/UrdfJoint.hpp"

#include <string>
#include <vector>

namespace kinemaforge::ik {

// Instances produced by UrdfModelLoader carry canonical geometry:
//   * origin.translation and origin.rpy hold only finite values
//   * an actuated joint's axis is finite, non-zero and numerically
//     normalized; it is expressed in the joint frame, not the parent's
//   * a fixed joint's axis carries no meaning
//
// This is a property of the loader, not of the type: RobotDescription is a
// plain aggregate and a hand-built one guarantees nothing.
//
// "Numerically normalized" is not a bitwise promise: axis-aligned input
// normalizes exactly, an arbitrary axis lands within a few ULP of length 1.
struct RobotDescription
{
    std::string name;

    std::vector<UrdfLink> links;
    std::vector<UrdfJoint> joints;
};

} // namespace kinemaforge::ik
