#pragma once

#include "ik_equations/model/Equation.hpp"
#include "ik_equations/model/IkEquationSystem.hpp"
#include "ik_equations/model/IkTarget.hpp"
#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/model/TargetValidation.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"

#include <expected>
#include <optional>
#include <string>
#include <vector>

namespace kinemaforge::ik {

enum class ConstraintBuilderErrorCode
{
    InvalidTarget,     // the target itself is not usable
    UndeclaredSymbol,  // an equation depends on a symbol that is not an unknown
    SystemRejected     // IkEquationSystem::create refused the assembled system
};

// The nested error is preserved rather than flattened into a string, and the
// invariant is: targetError has a value iff code == InvalidTarget, systemError
// iff code == SystemRejected. Like IkEquationBuilderError, this is a public
// aggregate -- the guarantee is about what this class produces, not about what
// the struct can hold.
struct ConstraintBuilderError
{
    ConstraintBuilderErrorCode code{};
    std::string message;

    std::optional<TargetValidationError> targetError;
    std::optional<IkEquationSystemError> systemError;
};

// Turns "where the tool must be" into equations: the first component that
// formulates an IK problem rather than describing geometry.
//
//     T(0,3)(q) = target.x        position, both task kinds
//     T(1,3)(q) = target.y
//     T(2,3)(q) = target.z
//
//     T(i,j)(q) = R_target(i,j)   orientation, Pose only, all nine cells
//
// Nine cells for three degrees of freedom is deliberate. A fixed subset of
// three is not independent for every rotation -- for R = I the diagonal cells
// are stationary to first order -- and dropping the third column would move the
// argument that justifies it (both matrices are proper rotations) outside the
// system, which then stops being self-contained. Keeping all nine throws
// nothing away; choosing an independent subproblem belongs to the layer that
// knows the solution structure, in Phase 3.
//
// Consequently: THE EQUATION COUNT IS NOT THE CONSTRAINT COUNT, and
// IkEquationSystem carries no rank information. A target accepted by
// TargetValidation may also be only an approximation of SO(3) -- the tolerance
// is 1e-8 -- so nine exact equalities need not have an exact solution. Phase 3
// selects the independent subproblem and verifies the rest numerically.
//
// Equations are stored as lhs = rhs, never as lhs - rhs = 0: subtracting would
// destroy which side came from the robot and which from the task, and Equation
// says so explicitly. Normalisation belongs to the future simplifier.
//
// Two preconditions cannot be checked here, and pretending otherwise would be
// worse than saying it plainly:
//
//   * WHICH transform. This class does not know whether it was handed
//     forwardKinematics() or tcpForwardKinematics(), and must not: the frame a
//     target lives in is the caller's contract. Pass the TCP transform when a
//     TCP is set.
//
//   * WHETHER the pair matches. transform and chain are separate arguments and
//     nothing couples them. Symbol names are positional ("q1", "q2", ...), so
//     the FK of one six-axis robot passes the symbol check against the chain of
//     another. What IS guaranteed is narrower and exact: every symbol appearing
//     in the equations is a declared unknown of the chain given. Detecting
//     provenance would need symbol identity stronger than a name -- out of
//     scope, recorded in STATUS.md.
//
// An unknown that does NOT appear in the equations is accepted, not an error: a
// wrist roll with no tool offset does not move the TCP position, so q6 is
// legitimately absent from a position-only system. The inclusion holds one way.
class ConstraintBuilder
{
public:
    explicit ConstraintBuilder(ExpressionFactory factory = {});

    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    build(const SymbolicTransform& transform,
          const KinematicChain& chain,
          const PositionTarget& target) const;

    // Position AND orientation. Position equations preserve the same lhs nodes
    // and sources as the PositionTarget overload; rhs constants carry the same
    // values. Non-zero rhs constants are not required to share node identity --
    // constant() builds a fresh node per call, and only zero is shared. Pinned
    // by PositionEquationsMatchPositionOnlyBuild.
    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    build(const SymbolicTransform& transform,
          const KinematicChain& chain,
          const PoseTarget& target) const;

    // Direction of the tool axis, with no constraint on where the tool is:
    //
    //     R(q) e_t = d        three equations, TWO degrees of freedom
    //
    // Three equations for two degrees of freedom, again -- a unit direction
    // lives on a sphere. The third equation is dependent on the solution set,
    // because |R(q) e_t| = 1 holds identically for a rigid transform and |d| = 1
    // is enforced here by canonicalisation. What stays free is the spin about
    // the tool axis, and that is the whole point: a nozzle that must look
    // somewhere must not be handed a full pose by accident.
    //
    // The direction may be any finite, non-zero vector; it is made a unit vector
    // here, never in the target and never by validation.
    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    build(const SymbolicTransform& transform,
          const KinematicChain& chain,
          const ToolDirectionTarget& target) const;

    // Position AND tool axis direction: three plus three equations, five degrees
    // of freedom, spin about the tool axis still free.
    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    build(const SymbolicTransform& transform,
          const KinematicChain& chain,
          const PositionAndToolDirectionTarget& target) const;

    // The angle between the tool axis and a reference direction:
    //
    //     dot(R(q) e_t, n) = cos(angle)     ONE equation, ONE degree of freedom
    //
    // The first left-hand side in this project that is not a cell of the FK
    // matrix but a computed expression. The tool axis lands somewhere on a cone
    // about n; where on the cone, and the spin about the axis, stay free.
    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    build(const SymbolicTransform& transform,
          const KinematicChain& chain,
          const DirectionAngleTarget& target) const;

    // Position AND the cone: four equations, four degrees of freedom.
    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    build(const SymbolicTransform& transform,
          const KinematicChain& chain,
          const PositionAndDirectionAngleTarget& target) const;

private:
    [[nodiscard]] std::vector<Equation>
    positionEquations(const SymbolicTransform& transform, const Vector3& position) const;

    void appendOrientationEquations(std::vector<Equation>& equations,
                                    const SymbolicTransform& transform,
                                    const RotationMatrix3& orientation) const;

    // Precondition: the direction passed validation, so it is finite and
    // non-zero. Scaled rather than a plain hypot -- see the comment in the .cpp.
    [[nodiscard]] static Vector3 normalizedDirection(const Vector3& direction) noexcept;

    void appendDirectionEquations(std::vector<Equation>& equations,
                                  const SymbolicTransform& transform,
                                  ToolAxis axis,
                                  const Vector3& unitDirection) const;

    void appendDirectionAngleEquation(std::vector<Equation>& equations,
                                      const SymbolicTransform& transform,
                                      ToolAxis axis,
                                      const Vector3& unitReference,
                                      double angle) const;

    // The shared spine: unknowns, the symbol guarantee, and the model's own
    // validation. Both overloads reach the model only through here, which is
    // what makes "every symbol in the equations is a declared unknown" a
    // property of the class rather than of one function.
    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuilderError>
    assemble(const KinematicChain& chain,
             IkTaskKind taskKind,
             std::vector<Equation> equations) const;

    ExpressionFactory factory_;
};

} // namespace kinemaforge::ik
