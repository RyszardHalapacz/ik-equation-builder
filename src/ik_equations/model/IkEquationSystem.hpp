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
// Closed, because create() enforces nine invariants an aggregate cannot.
// Cheap to copy anyway: copying duplicates Expression handles, never trees.
class IkEquationSystem
{
public:
    // Validates in a fixed order, so each code has a deterministic meaning
    // when several invariants are broken at once:
    //
    //   NoEquations, NoUnknowns, EmptyUnknownName, DuplicateUnknownName,
    //   DuplicateUnknownIndex, UnorderedUnknowns, TaskEquationMismatch,
    //   DuplicateEquationSource, UnorderedEquations
    //
    // Two orderings are forced rather than arbitrary:
    //
    //   DuplicateUnknownIndex before UnorderedUnknowns -- two equal indices
    //   already break a strictly increasing sequence, so the reverse order
    //   would report a duplicate as bad ordering: true, but useless.
    //
    //   DuplicateEquationSource before UnorderedEquations -- for the same
    //   reason, and because two equations addressing one matrix cell are
    //   either redundant or contradictory, which is worth its own diagnosis.
    [[nodiscard]] static std::expected<IkEquationSystem, IkEquationSystemError>
    create(IkTaskKind taskKind,
           std::vector<JointVariable> unknowns,
           std::vector<Equation> equations);

    [[nodiscard]] IkTaskKind taskKind() const noexcept;

    // Ordered by ascending JointVariable::index -- part of the contract.
    [[nodiscard]] std::span<const JointVariable> unknowns() const noexcept;

    // Position X, Y, Z first; then orientation cells in row-major order.
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
