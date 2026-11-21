#pragma once

#include "ik_equations/solver/AnalyticIkSolver.hpp"

#include <array>
#include <expected>
#include <memory>
#include <optional>

namespace kinemaforge::ik {

// Geometric Jacobian of the tool frame (the flange, or the TCP when one was
// given) and its singular values.
//
//   jacobian[row][joint]: rows 0..2 are the linear velocity of the tool origin
//   in the base frame, rows 3..5 the angular velocity, per unit joint velocity.
//   Linear rows are metres per radian, angular rows are dimensionless, so the
//   singular values mix units the way any "smallest singular value of the
//   Jacobian" does.
//
//   singularValues are descending. They come from a one-sided Jacobi SVD of the
//   Jacobian itself, not from the eigenvalues of J^T J, so a value near zero is
//   accurate to its own size instead of to the square root of round-off.
//   condition is infinity when the smallest value is exactly zero.
struct JacobianAnalysis
{
    std::array<std::array<double, 6>, 6> jacobian{};
    std::array<double, 6> singularValues{};
    double minSingularValue{};
    double condition{};
};

// AnalyticIkSolver with the robot-dependent work done once.
//
// create() validates the chain, builds the geometry, detects the kinematic
// pattern and computes the home transform; solve() and solveCompact() then do
// only what depends on the target. A map generator calls the solver hundreds of
// thousands of times for ONE robot: this is the shape it wants.
//
//   * Results are identical to AnalyticIkSolver::solve / solveCompact for the
//     same chain, tool transform, target and options (same branches, same
//     order -- see the contract in AnalyticIkSolver.hpp).
//   * A chain the solver cannot handle is reported by create(), not on every
//     call. Target errors (InvalidTarget) are still reported per call.
//   * Immutable after creation: one object may be used from any number of
//     threads at once. Copies are independent deep copies (no shared state).
//   * The chain is copied; the caller's chain need not outlive the object.
class PreparedAnalyticIk
{
public:
    PreparedAnalyticIk(const PreparedAnalyticIk& other);
    PreparedAnalyticIk& operator=(const PreparedAnalyticIk& other);
    PreparedAnalyticIk(PreparedAnalyticIk&& other) noexcept;
    PreparedAnalyticIk& operator=(PreparedAnalyticIk&& other) noexcept;
    ~PreparedAnalyticIk();

    [[nodiscard]] static std::expected<PreparedAnalyticIk, AnalyticIkSolverError>
    create(const KinematicChain& chain, const std::optional<FixedRigidTransform>& tcp = std::nullopt);

    [[nodiscard]] std::expected<AnalyticIkSolution, AnalyticIkSolverError>
    solve(const PoseTarget& target, const AnalyticIkSolverOptions& options = {}) const;

    [[nodiscard]] std::expected<CompactAnalyticIkSolution, AnalyticIkSolverError>
    solveCompact(const PoseTarget& target) const;

    // For a joint configuration (radians, in actuated-joint order). Only
    // NonFiniteComputation can fail.
    [[nodiscard]] std::expected<JacobianAnalysis, AnalyticIkSolverError>
    jacobian(const std::array<double, 6>& joints) const;

private:
    // Not shared_ptr: its control block pulls in std::type_info::operator==,
    // which a Debug build with the statically linked MinGW runtime defines twice.
    struct Impl;
    PreparedAnalyticIk() = default;
    std::unique_ptr<Impl> impl_;
};

} // namespace kinemaforge::ik
