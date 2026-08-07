#pragma once

#include "ik_equations/model/Vector3.hpp"

#include <array>
#include <variant>

namespace kinemaforge::ik {

// values[row][column], row-major, both indices 0..2 -- the same indexing
// OrientationEquationSource::row()/column() use.
//
// A plain aggregate: no operator(), no methods. SymbolicMatrix has an
// operator() because it bounds-checks; here there is nothing to check beyond
// what std::array already does.
//
// Validity is not a construction condition -- see TargetValidation.
struct RotationMatrix3
{
    std::array<std::array<double, 3>, 3> values{};
};

// A point expressed in the base frame of whatever SymbolicTransform the
// constraint builder is given. The model deliberately carries no frame name:
// it does not know, and must not know, whether that transform ends at the TCP
// or at the chain tip. Pairing the right target with the right transform is
// the caller's responsibility, and no type here can check it.
struct PositionTarget
{
    Vector3 position;   // metres
};

// T_base_target: the position and orientation of the target frame, expressed
// in the base frame. Not "a pose" -- the direction is part of the contract.
struct PoseTarget
{
    Vector3 position;             // metres
    RotationMatrix3 orientation;
};

// A closed set. A struct of optionals would admit a target that constrains
// nothing, and combinations nobody defined.
using IkTarget = std::variant<PositionTarget, PoseTarget>;

} // namespace kinemaforge::ik
