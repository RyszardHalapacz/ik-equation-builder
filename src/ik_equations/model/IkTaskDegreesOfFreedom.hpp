#pragma once

#include "ik_equations/model/Equation.hpp"

#include <cstddef>
#include <optional>

namespace kinemaforge::ik {

// How many degrees of freedom of the TASK SPACE a kind of task removes.
//
// The long name is deliberate: this is not the rank of the robot's Jacobian and
// not the rank of the equation system. It is a property of the geometry of the
// constraint, independent of the robot and of the configuration -- which is
// exactly what makes it honest to state at all.
//
//   Position                  3   position fixed, orientation free
//   Pose                      6   nothing free
//   ToolDirection             2   a unit direction lives on a sphere: TWO
//                                 degrees of freedom, not three. The third
//                                 equation is dependent on the solution set,
//                                 because |R(q) e_t| = 1 holds identically for a
//                                 rigid transform and |d| = 1 is enforced by the
//                                 builder's canonicalisation.
//   PositionAndToolDirection  5   spin about the tool axis stays free
//   DirectionAngle            1   the tool axis lands on a cone about n: ONE
//                                 degree of freedom removed, not two. True only
//                                 because the angle interval is open -- at 0 and
//                                 pi the cone degenerates to a line and the
//                                 constraint removes two. Those targets are
//                                 rejected, and belong to ToolDirection.
//   PositionAndDirectionAngle 4   position (3) plus the cone (1); the axis stays
//                                 free on the cone and the spin stays free too
//
// THE EQUATION COUNT IS NOT THIS NUMBER. ToolDirection carries three equations
// and removes two; Pose carries twelve and removes six.
//
// Pose == 6 is true only because the model now requires a complete orientation
// block: nine cells, not "at least one". Before that tightening this function
// would have been lying about a system the library itself could build.
//
// nullopt for a value forged with static_cast<IkTaskKind>(99). Returning 0 would
// be worse than useless: it reads as a legitimate "constrains nothing".
[[nodiscard]] constexpr std::optional<std::size_t>
taskSpaceConstrainedDegreesOfFreedom(IkTaskKind kind) noexcept
{
    switch (kind)
    {
    case IkTaskKind::Position:                 return 3;
    case IkTaskKind::Pose:                     return 6;
    case IkTaskKind::ToolDirection:            return 2;
    case IkTaskKind::PositionAndToolDirection: return 5;
    case IkTaskKind::DirectionAngle:            return 1;
    case IkTaskKind::PositionAndDirectionAngle: return 4;
    }
    return std::nullopt;
}

} // namespace kinemaforge::ik
