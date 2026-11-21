#pragma once

#include "ik_equations/model/AnalyticIkSolution.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

namespace kinemaforge::ik {

// Compact analytic IK results.
//
// AnalyticIkSolution lists every combination of in-limit full turns of every
// joint as its own branch -- a product over joints that grows with the width
// of the limits and is therefore bounded by AnalyticIkSolverOptions::maxBranches.
// The compact form keeps the geometric branches and, per joint, the SET of its
// in-limit turns. Its size is independent of the limits; nothing is refused
// for being too wide. expand() turns it into exactly what solve() returns.
//
// Values far from zero lose precision like any double: base + 2*pi*k carries
// an absolute error of ~ulp(2*pi*k), i.e. ~1e-10 rad at k ~ 1e5.

// One joint's alternatives: value(k) = base + 2*pi*k for k = firstTurn..lastTurn,
// snapped onto the limits at the ends. Unlimited joints: one value (k = 0).
struct JointTurns
{
    std::size_t actuatedJointPosition{};
    std::string jointName;
    double base{};
    std::int64_t firstTurn{0};
    std::int64_t lastTurn{-1};
    bool limited{};
    double lower{};
    double upper{};

    [[nodiscard]] std::size_t count() const noexcept
    {
        if (lastTurn < firstTurn)
            return 0U;
        const auto span = static_cast<std::uint64_t>(lastTurn) - static_cast<std::uint64_t>(firstTurn);
        constexpr auto maximum = std::numeric_limits<std::size_t>::max();
        return span >= static_cast<std::uint64_t>(maximum) ? maximum : static_cast<std::size_t>(span) + 1U;
    }

    [[nodiscard]] double value(std::int64_t turn) const noexcept
    {
        const double candidate = base + 2.0 * std::numbers::pi_v<double> * static_cast<double>(turn);
        if (!limited)
            return candidate;
        return candidate < lower ? lower : (candidate > upper ? upper : candidate);
    }
};

// A singular wrist family in compact form: for each dependent turn
// k = firstTurn..lastTurn the relation is
//     dependent = baseOffset + 2*pi*k + multiplier * free,
// over a free interval that depends on k (expand() computes it, together with
// the exact offset that keeps every returned value within the limits).
struct CompactCoupledFamily
{
    std::size_t freeActuatedJointPosition{};
    std::string freeJointName;
    std::size_t dependentActuatedJointPosition{};
    std::string dependentJointName;
    double multiplier{};
    double baseOffset{};
    std::int64_t firstTurn{0};
    std::int64_t lastTurn{-1};
    double freeLower{};
    double freeUpper{};
    bool dependentLimited{};
    double dependentLower{};
    double dependentUpper{};

    [[nodiscard]] std::size_t count() const noexcept
    {
        return JointTurns{0, {}, 0.0, firstTurn, lastTurn, false, 0.0, 0.0}.count();
    }
};

struct CompactSolutionBranch
{
    // Regular branch: all six joints. Singular wrist: the four determined ones
    // plus `family` for the free/dependent pair.
    std::vector<JointTurns> joints;
    std::optional<CompactCoupledFamily> family;
    ShoulderBranch shoulder{ShoulderBranch::Primary};
    ElbowBranch elbow{ElbowBranch::Positive};
    WristBranch wrist{WristBranch::Primary};
    std::optional<IkSingularityKind> singularity;

    // Branches this expands to (saturated at SIZE_MAX).
    [[nodiscard]] std::size_t count() const noexcept
    {
        constexpr auto maximum = std::numeric_limits<std::size_t>::max();
        std::size_t total = 1U;
        const auto multiply = [&](std::size_t factor) {
            if (total == 0U || factor == 0U)
                total = 0U;
            else
                total = total > maximum / factor ? maximum : total * factor;
        };
        for (const auto& joint : joints)
            multiply(joint.count());
        if (family)
            multiply(family->count());
        return total;
    }
};

struct CompactAnalyticIkSolution
{
    AnalyticIkStatus status{AnalyticIkStatus::NoSolution};
    std::vector<CompactSolutionBranch> branches;
    std::string message;
    double orientationCorrection{};
    // As AnalyticIkSolution::positionalFamilies; not counted by
    // expandedBranchCount() and not produced by expand().
    std::vector<PositionalFamilyBranch> positionalFamilies;

    // Number of branches expand() returns (saturated at SIZE_MAX).
    [[nodiscard]] std::size_t expandedBranchCount() const noexcept
    {
        constexpr auto maximum = std::numeric_limits<std::size_t>::max();
        std::size_t total = 0U;
        for (const auto& branch : branches)
        {
            const std::size_t count = branch.count();
            total = total > maximum - count ? maximum : total + count;
        }
        return total;
    }
};

} // namespace kinemaforge::ik
