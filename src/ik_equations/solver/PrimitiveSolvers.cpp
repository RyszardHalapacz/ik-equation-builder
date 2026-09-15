#include "ik_equations/solver/PrimitiveSolvers.hpp"

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
    if (!std::isfinite(angle))
        return angle;

    double result = std::remainder(angle, twoPi);
    // Keep a single representative for the seam.  std::remainder may return
    // +pi or -pi depending on the input quotient; the solver uses [-pi, pi).
    if (result >= std::numbers::pi_v<double>)
        result -= twoPi;
    return result;
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
    if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c) ||
        tolerance < 0.0 || !std::isfinite(tolerance))
        return {ScalarSolutionKind::None, {}};

    const double amplitude = std::hypot(a, b);
    if (amplitude <= tolerance)
    {
        if (std::abs(c) <= tolerance)
            return {ScalarSolutionKind::Infinite, {}};
        return {ScalarSolutionKind::None, {}};
    }

    const double ratio = c / amplitude;
    if (ratio < -1.0 - tolerance || ratio > 1.0 + tolerance)
        return {ScalarSolutionKind::None, {}};

    // a*sin(x)+b*cos(x) = r*sin(x+phi), phi=atan2(b,a).
    const double phase = std::atan2(b, a);
    const ScalarSolutions shifted = solveSine(clampedUnit(ratio), tolerance);

    ScalarSolutions result{ScalarSolutionKind::Finite, {}};
    for (double value : shifted.values)
        appendDistinct(result.values, value - phase, tolerance);
    return result;
}

std::vector<Planar2RBranch> solvePlanar2R(double targetX,
                                         double targetY,
                                         double link1Length,
                                         double link2Length,
                                         double link2HomeAngle,
                                         double secondJointDirectionSign,
                                         double tolerance)
{
    if (!std::isfinite(targetX) || !std::isfinite(targetY) ||
        !std::isfinite(link1Length) || !std::isfinite(link2Length) ||
        !std::isfinite(link2HomeAngle) || !std::isfinite(secondJointDirectionSign) ||
        !std::isfinite(tolerance) || tolerance < 0.0 ||
        link1Length <= tolerance || link2Length <= tolerance ||
        std::abs(std::abs(secondJointDirectionSign) - 1.0) > tolerance)
        return {};

    const double radiusSquared = targetX * targetX + targetY * targetY;
    const double denominator = 2.0 * link1Length * link2Length;
    const double cosine = (radiusSquared - link1Length * link1Length -
                           link2Length * link2Length) / denominator;

    if (cosine < -1.0 - tolerance || cosine > 1.0 + tolerance)
        return {};

    const double internalMagnitude = std::acos(clampedUnit(cosine));
    const double targetAngle = std::atan2(targetY, targetX);

    std::vector<Planar2RBranch> result;
    for (double internal : {internalMagnitude, -internalMagnitude})
    {
        const double second = (internal - link2HomeAngle) / secondJointDirectionSign;
        const double vx = link1Length + link2Length * std::cos(internal);
        const double vy = link2Length * std::sin(internal);
        const double first = normalizeAngle(targetAngle - std::atan2(vy, vx));
        const bool singular = std::abs(std::sin(internal)) <= tolerance;
        const ElbowBranch elbow = internal >= 0.0 ? ElbowBranch::Positive : ElbowBranch::Negative;

        const bool duplicate = std::any_of(result.begin(), result.end(), [&](const auto& branch) {
            return std::abs(normalizeAngle(branch.firstJoint - first)) <= tolerance &&
                   std::abs(normalizeAngle(branch.secondJoint - second)) <= tolerance;
        });
        if (!duplicate)
            result.push_back({first, normalizeAngle(second), elbow, singular});
    }

    return result;
}

} // namespace kinemaforge::ik
