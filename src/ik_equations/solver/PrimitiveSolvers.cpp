#include "ik_equations/solver/PrimitiveSolvers.hpp"
#include "ik_equations/solver/detail/AnalyticIkKernels.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace kinemaforge::ik {
namespace {

constexpr double twoPi = 2.0 * std::numbers::pi_v<double>;

double clampedUnit(double value) noexcept
{
    return std::clamp(value, -1.0, 1.0);
}

void appendDistinct(std::vector<double>& values, double value, double tolerance)
{
    const double normalized = normalizeAngle(value);
    const bool duplicate = std::any_of(values.begin(), values.end(), [&](double existing) {
        return std::abs(normalizeAngle(existing - normalized)) <= tolerance;
    });
    if (!duplicate)
        values.push_back(normalized);
}

} // namespace

double normalizeAngle(double angle) noexcept
{
    return detail::normalizeAngle(angle);
}

ScalarSolutions solveLinear(double a, double b, double tolerance)
{
    if (!std::isfinite(a) || !std::isfinite(b) || tolerance < 0.0 || !std::isfinite(tolerance))
        return {ScalarSolutionKind::None, {}};

    if (std::abs(a) <= tolerance)
    {
        if (std::abs(b) <= tolerance)
            return {ScalarSolutionKind::Infinite, {}};
        return {ScalarSolutionKind::None, {}};
    }

    return {ScalarSolutionKind::Finite, {-b / a}};
}

ScalarSolutions solveSine(double value, double tolerance)
{
    if (!std::isfinite(value) || tolerance < 0.0 || !std::isfinite(tolerance))
        return {ScalarSolutionKind::None, {}};
    if (value < -1.0 - tolerance || value > 1.0 + tolerance)
        return {ScalarSolutionKind::None, {}};

    const double clamped = clampedUnit(value);
    const double first = std::asin(clamped);
    const double second = std::numbers::pi_v<double> - first;

    ScalarSolutions result{ScalarSolutionKind::Finite, {}};
    appendDistinct(result.values, first, tolerance);
    appendDistinct(result.values, second, tolerance);
    return result;
}

ScalarSolutions solveCosine(double value, double tolerance)
{
    if (!std::isfinite(value) || tolerance < 0.0 || !std::isfinite(tolerance))
        return {ScalarSolutionKind::None, {}};
    if (value < -1.0 - tolerance || value > 1.0 + tolerance)
        return {ScalarSolutionKind::None, {}};

    const double angle = std::acos(clampedUnit(value));
    ScalarSolutions result{ScalarSolutionKind::Finite, {}};
    appendDistinct(result.values, angle, tolerance);
    appendDistinct(result.values, -angle, tolerance);
    return result;
}

ScalarSolutions solveSinCos(double a, double b, double c, double tolerance)
{
    // One implementation, shared with the generated runtime.
    auto solutions = detail::solveSinCos(a, b, c, tolerance);
    switch (solutions.kind)
    {
    case detail::SinCosKind::None:     return {ScalarSolutionKind::None, {}};
    case detail::SinCosKind::Infinite: return {ScalarSolutionKind::Infinite, {}};
    case detail::SinCosKind::Finite:   return {ScalarSolutionKind::Finite, std::move(solutions.values)};
    }
    return {ScalarSolutionKind::None, {}};
}

std::vector<Planar2RBranch> solvePlanar2R(double targetX,
                                         double targetY,
                                         double link1Length,
                                         double link2Length,
                                         double link2HomeAngle,
                                         double secondJointDirectionSign,
                                         double tolerance)
{
    // One implementation, shared with both analytic solvers. This public
    // primitive keeps its contract: no limits, and the folded equal-link case
    // yields its one arbitrary member marked singular.
    const auto arm = detail::solvePlanarArm(targetX, targetY, link1Length, link2Length,
                                            link2HomeAngle, secondJointDirectionSign, {},
                                            tolerance);
    std::vector<Planar2RBranch> result;
    result.reserve(arm.branches.size());
    for (const auto& branch : arm.branches)
        result.push_back({branch.shoulder, branch.elbow,
                          branch.positiveElbow ? ElbowBranch::Positive : ElbowBranch::Negative,
                          branch.singular});
    return result;
}

} // namespace kinemaforge::ik
