#pragma once

#include "ik_equations/model/AnalyticIkSolution.hpp"
#include "ik_equations/model/CompactAnalyticIkSolution.hpp"
#include "ik_equations/model/FixedRigidTransform.hpp"
#include "ik_equations/model/IkTarget.hpp"
#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/model/TargetValidation.hpp"

#include <array>
#include <cstddef>
#include <expected>
#include <optional>
#include <string>
#include <vector>

namespace kinemaforge::ik {

enum class AnalyticIkSolverErrorCode
{
    InvalidTarget,
    InvalidKinematicGeometry,
    UnsupportedKinematicStructure,
    NonFiniteComputation,
    // The result would hold more branches than AnalyticIkSolverOptions allows.
    // Nothing is returned: a truncated list reported as success would hide
    // valid configurations from the caller.
    ResourceLimitExceeded
};

struct AnalyticIkSolverError
{
    AnalyticIkSolverErrorCode code{};
    std::string message;
    std::optional<TargetValidationError> targetError;
    // Set for ResourceLimitExceeded: the number of branches the full result
    // needs (an upper estimate, saturated at SIZE_MAX), so the caller can raise
    // the budget or narrow the joint limits deliberately.
    std::optional<std::size_t> requiredBranches;
};

struct AnalyticIkSolverOptions
{
    // Upper bound on branches (regular solutions and singular families) in one
    // result. Every in-limit full turn of every joint is a separate branch, so
    // the count is a product over joints and grows without bound as limits
    // widen. It is computed from the limits BEFORE anything is allocated.
    std::size_t maxBranches{4096};
};

// First analytic family supported by KinemaForge:
//   * exactly six actuated rotary joints,
//   * joint 2 and 3 axes parallel,
//   * joints 4..6 form a spherical wrist,
//   * wrist axes 4 and 6 are parallel and axis 5 is perpendicular to them.
//
// The solver consumes a PoseTarget because decoupling the wrist centre from a
// non-zero tool/flange offset requires target orientation.  Position-only IK is
// under-constrained and is deliberately not guessed here.
class AnalyticIkSolver
{
public:
    [[nodiscard]] std::expected<AnalyticIkSolution, AnalyticIkSolverError>
    solve(const KinematicChain& chain,
          const PoseTarget& target,
          const std::optional<FixedRigidTransform>& tcp = std::nullopt,
          const AnalyticIkSolverOptions& options = {}) const;

    // The same solutions in compact form: geometric branches with, per joint,
    // the set of its in-limit full turns. Its size does not depend on the
    // width of the limits, so there is no branch budget; expand() produces
    // exactly what solve() returns. ResourceLimitExceeded only when turn
    // numbers would not fit an int64_t.
    [[nodiscard]] std::expected<CompactAnalyticIkSolution, AnalyticIkSolverError>
    solveCompact(const KinematicChain& chain,
                 const PoseTarget& target,
                 const std::optional<FixedRigidTransform>& tcp = std::nullopt) const;
};

// The member of a positional family at the given value of its free joint, every
// joint on an in-limit turn; nothing where that member leaves a joint limit or
// the value lies outside [freeLower, freeUpper]. Shared with generated code.
[[nodiscard]] std::optional<std::array<double, 6>> evaluate(const PositionalFamily& family, double value);

// Every branch a compact solution stands for, in AnalyticIkSolution's form.
// Check expandedBranchCount() first: the result is a product over joints.
[[nodiscard]] std::vector<SolutionBranch> expand(const CompactAnalyticIkSolution& solution);

} // namespace kinemaforge::ik
