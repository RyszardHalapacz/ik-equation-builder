#pragma once

#include "ik_equations/model/ToolAxis.hpp"
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

// A direction the tool axis must point along, in the base frame.
//
// direction is finite and NON-ZERO. It does NOT have to be a unit vector and
// this type does not promise that it is: a direction is a ray, its length
// carries no semantics, and a normal produced by a cross product has whatever
// length it has. Canonicalisation belongs to ConstraintBuilder, which is the
// only thing that reads it -- validation checks, it never rewrites.
//
// "Alignment" here means an ORIENTED direction: R(q) e_t = d points the tool
// along d. The opposite setting is a different target, -d, not the same one.
//
// No position: "stand so the nozzle looks that way", without saying where. It
// leaves four degrees of freedom on a six-axis robot, which is a task, not a
// question -- the wrist subproblem of an analytic solver has exactly this shape.
struct ToolDirectionTarget
{
    Vector3 direction;
    ToolAxis axis{ToolAxis::Z};
};

struct PositionAndToolDirectionTarget
{
    Vector3 position;    // metres
    Vector3 direction;   // finite, non-zero; length carries no semantics
    ToolAxis axis{ToolAxis::Z};
};

// The angle between the tool axis and a reference direction, in the base frame:
//
//     dot(R(q) e_t, n) = cos(angle)      ONE equation, ONE degree of freedom
//
// The tool axis ends up somewhere on a cone about n. What stays free is where on
// that cone it lands AND the spin about the axis itself.
//
// reference is finite and non-zero, and does NOT have to be a unit vector --
// same contract as a direction, for the same reason: it names a ray. A surface
// normal from a cross product is the expected source, hence any length.
//
// The name says nothing about surfaces: the model does not know, and has no way
// to know, that n came from a normal. This is an angle to a reference direction,
// and a surface normal is one source of it.
//
// angle is in radians and its interval is OPEN: 0 and pi are rejected. There the
// cone collapses to a line, the constraint removes two rotational degrees of
// freedom instead of one, and the scalar equation goes singular -- and that
// geometry already has an exact description in ToolDirectionTarget.
struct DirectionAngleTarget
{
    Vector3 reference;
    double angle{};
    ToolAxis axis{ToolAxis::Z};
};

struct PositionAndDirectionAngleTarget
{
    Vector3 position;     // metres
    Vector3 reference;    // finite, non-zero; length carries no semantics
    double angle{};       // radians, open interval
    ToolAxis axis{ToolAxis::Z};
};

// A closed set. A struct of optionals would admit a target that constrains
// nothing, and combinations nobody defined -- and it would turn the kind of
// task into a runtime value, when IkTaskKind is meant to classify.
using IkTarget = std::variant<PositionTarget,
                              PoseTarget,
                              ToolDirectionTarget,
                              PositionAndToolDirectionTarget,
                              DirectionAngleTarget,
                              PositionAndDirectionAngleTarget>;

} // namespace kinemaforge::ik
