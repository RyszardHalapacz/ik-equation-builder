#pragma once

#include "ik_equations/model/AnalyticIkSolution.hpp"
#include "ik_equations/model/FixedRigidTransform.hpp"
#include "ik_equations/model/IkTarget.hpp"
#include "ik_equations/model/KinematicChain.hpp"
#include "ik_equations/model/TargetValidation.hpp"

#include <expected>
#include <optional>
#include <string>

namespace kinemaforge::ik {

enum class AnalyticIkSolverErrorCode
{
    InvalidTarget,
    InvalidKinematicGeometry,
    UnsupportedKinematicStructure,
    NonFiniteComputation
};

struct AnalyticIkSolverError
{
    AnalyticIkSolverErrorCode code{};
    std::string message;
    std::optional<TargetValidationError> targetError;
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
          const std::optional<FixedRigidTransform>& tcp = std::nullopt) const;
};

} // namespace kinemaforge::ik
