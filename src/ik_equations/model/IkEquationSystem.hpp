#pragma once

#include "ik_equations/model/Equation.hpp"
#include "ik_equations/model/JointVariable.hpp"

#include <expected>
#include <span>
#include <string>
#include <vector>

namespace kinemaforge::ik {

enum class IkEquationSystemErrorCode
{
    NoEquations,
    NoUnknowns,
    EmptyUnknownName,
    DuplicateUnknownName,
    DuplicateUnknownIndex,
    UnorderedUnknowns,
    InvalidToolAxis,
    MixedToolAxes,
    TaskEquationMismatch,
    DuplicateEquationSource,
    UnorderedEquations
};

struct IkEquationSystemError
{
    IkEquationSystemErrorCode code{};
    std::string message;
};

// The input of the future EquationSimplifier and EquationSolver, and the
// output of the future ConstraintBuilder.
//
// Self-contained on purpose: it carries its own unknowns, so the pipeline
// IkEquationSystem -> EquationSimplifier -> EquationSolver is one object
// rather than three that have to be carried together. Recovering the unknowns
// from the equation trees is not an option -- there is no public symbol
// collector, DAG traversal order is not joint order, sorting by name gives
// q1, q10, q2, and a symbolic target parameter would be indistinguishable
// from a joint variable.
//
// What the equation count does NOT mean: this type stores no rank or
// independence information, so equations may be redundant. For a Pose system
// built by ConstraintBuilder from a rigid-FK transform, nine orientation
// equations represent a target orientation whose geometric space has three
// rotational degrees of freedom -- the qualifier matters, because the builder
// accepts any SymbolicTransform and does not check that the 3x3 block is a
// rotation at all. And when the right-hand sides come from a PoseTarget, those
// values were accepted within TargetValidation tolerance (1e-8), so the nine
// exact equalities need not have an exact solution even for a reachable pose.
// A consumer must therefore not read "twelve equations" as "twelve independent
// constraints": selecting an independent subproblem, and verifying the
// remaining equations numerically, belongs to the solver.
//
// Closed, because create() enforces nine invariants an aggregate cannot.
// Cheap to copy anyway: copying duplicates Expression handles, never trees.
class IkEquationSystem
{
public:
    // Validates in a fixed order, so each code has a deterministic meaning
    // when several invariants are broken at once:
    //
    //   NoEquations, NoUnknowns, EmptyUnknownName, DuplicateUnknownName,
    //   DuplicateUnknownIndex, UnorderedUnknowns, InvalidToolAxis,
    //   MixedToolAxes, TaskEquationMismatch, DuplicateEquationSource,
    //   UnorderedEquations
    //
    // Two orderings are forced rather than arbitrary:
    //
    //   DuplicateUnknownIndex before UnorderedUnknowns -- two equal indices
    //   already break a strictly increasing sequence, so the reverse order
    //   would report a duplicate as bad ordering: true, but useless.
    //
    //   DuplicateEquationSource before UnorderedEquations -- for the same
    //   reason, and because two equations occupying the same canonical
    //   constraint slot are either redundant or contradictory, which is worth
    //   its own diagnosis.
    //
    //   InvalidToolAxis and MixedToolAxes before TaskEquationMismatch -- an
    //   axis that is not an enumerator makes every later statement about "the"
    //   axis meaningless, and a system whose direction equations disagree about
    //   the axis is contradictory whatever the task kind says. Both are checked
    //   in their own pass over the equations, so the reported code does not
    //   depend on which broken equation happens to come first.
    [[nodiscard]] static std::expected<IkEquationSystem, IkEquationSystemError>
    create(IkTaskKind taskKind,
           std::vector<JointVariable> unknowns,
           std::vector<Equation> equations);

    [[nodiscard]] IkTaskKind taskKind() const noexcept;

    // Ordered by ascending JointVariable::index -- part of the contract.
    [[nodiscard]] std::span<const JointVariable> unknowns() const noexcept;

    // Position X, Y, Z first; then orientation cells in row-major order; then
    // direction components X, Y, Z; then the single direction-angle equation.
    // Only the source kinds legal for this task kind are present, and a system
    // carrying any tool-axis equation names exactly one tool axis.
    [[nodiscard]] std::span<const Equation> equations() const noexcept;

private:
    IkEquationSystem(IkTaskKind taskKind,
                     std::vector<JointVariable> unknowns,
                     std::vector<Equation> equations);

    IkTaskKind taskKind_;
    std::vector<JointVariable> unknowns_;
    std::vector<Equation> equations_;
};

} // namespace kinemaforge::ik
