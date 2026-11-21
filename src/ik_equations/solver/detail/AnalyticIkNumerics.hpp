#pragma once

#include "ik_equations/model/IkTolerances.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>

namespace kinemaforge::ik::detail {

inline constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;

struct XyxAngles
{
    double alpha{};
    double beta{};
    double gamma{};
    bool singular{};
};

// R = Rx(alpha) Ry(beta) Rx(gamma). Use the off-diagonal entries to
// measure sin(beta): acos(R00) amplifies round-off near both singularities.
inline XyxAngles decomposeXyx(const std::array<std::array<double, 3>, 3>& r,
                              double singularTolerance) noexcept
{
    const double sineBeta = std::hypot(r[1][0], r[2][0]);
    if (sineBeta <= singularTolerance)
    {
        if (r[0][0] >= 0.0)
            return {0.0, 0.0, std::atan2(r[2][1], r[1][1]), true};
        return {0.0, std::numbers::pi_v<double>, -std::atan2(r[1][2], r[1][1]), true};
    }

    const double alpha = std::atan2(r[1][0], -r[2][0]);
    const double beta = std::atan2(sineBeta, r[0][0]);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    // Remove alpha before recovering gamma from a unit-length row. Recovering
    // both outer angles independently from O(sin(beta)) entries loses their
    // accurately determined sum/difference close to a singularity.
    const double gamma = std::atan2(-(ca * r[1][2] + sa * r[2][2]),
                                      ca * r[1][1] + sa * r[2][1]);
    return {alpha, beta, gamma, false};
}

struct TurnRange
{
    std::int64_t first{};
    std::int64_t last{-1}; // empty by default
    // The range is not empty but its bounds do not fit in int64_t. first/last
    // are then meaningless (left empty) and turnCount() reports SIZE_MAX, which
    // every caller treats as "over any branch budget" -- never as "no turns".
    bool saturated{false};
};

// Integers k for which lower <= base + 2*pi*k <= upper. A range whose bounds
// cannot be represented as int64_t is reported as saturated rather than empty:
// treating it as empty turned a reachable target into NoSolution (audit
// 2026-11-16 #8).
inline TurnRange turnsWithin(double lower, double upper, double base = 0.0) noexcept
{
    const double first = std::ceil((lower - base) / kTwoPi - tolerance::kRoundOff);
    const double last = std::floor((upper - base) / kTwoPi + tolerance::kRoundOff);
    if (std::isnan(first) || std::isnan(last) || first > last)
        return {};
    constexpr double integerBound = 0x1p63;
    if (!(first >= -integerBound && first < integerBound &&
          last >= -integerBound && last < integerBound))
        return {.saturated = true};
    return {static_cast<std::int64_t>(first), static_cast<std::int64_t>(last)};
}

// Number of integers in a TurnRange, saturated to SIZE_MAX. Counting never
// materialises the turns, so it is O(1) however wide the joint limits are.
// SIZE_MAX means "too many to count"; see branchBudgetExceeded().
inline std::size_t turnCount(const TurnRange& range) noexcept
{
    constexpr auto maximum = std::numeric_limits<std::size_t>::max();
    if (range.saturated)
        return maximum;
    if (range.last < range.first)
        return 0;
    const auto span = static_cast<std::uint64_t>(range.last) -
                      static_cast<std::uint64_t>(range.first);
    if (span >= static_cast<std::uint64_t>(maximum))
        return maximum;
    return static_cast<std::size_t>(span) + 1U;
}

// A saturated count (SIZE_MAX) is a lower bound, not an exact number, so it
// exceeds every budget -- including a caller-supplied SIZE_MAX.
inline bool branchBudgetExceeded(std::size_t required, std::size_t budget) noexcept
{
    return required > budget || required == std::numeric_limits<std::size_t>::max();
}

inline std::size_t saturatingMultiply(std::size_t lhs, std::size_t rhs) noexcept
{
    constexpr auto maximum = std::numeric_limits<std::size_t>::max();
    if (lhs == 0U || rhs == 0U)
        return 0U;
    return lhs > maximum / rhs ? maximum : lhs * rhs;
}

inline std::size_t saturatingAdd(std::size_t lhs, std::size_t rhs) noexcept
{
    constexpr auto maximum = std::numeric_limits<std::size_t>::max();
    return lhs > maximum - rhs ? maximum : lhs + rhs;
}

// Admission margin for a joint value against its position limits. A value that
// is outside a limit by no more than this is the limit itself, displaced by
// round-off; it is snapped onto the limit BEFORE the final FK check, so the
// value handed to the caller is the value that was verified. See
// model/IkTolerances.hpp for why it is 1e-10 and why that is safe.
inline constexpr double kLimitTolerance = tolerance::kLimitAdmission;

inline bool withinLimit(double value, double lower, double upper) noexcept
{
    return std::isfinite(value) && value >= lower - kLimitTolerance &&
           value <= upper + kLimitTolerance;
}

// Margin within which an ill-conditioned angle outside its limits is moved
// onto the nearest limit before the joints that depend on it are re-solved.
// Two places need it: the elbow near straight/folded (below) and the outer
// wrist angles q4/q6 next to the wrist singularity, where each is recovered
// with an error ~eps/|sin q5| while their sum/difference stays exact.
//
// Near a straight (or folded) elbow the elbow angle is ill-conditioned: an
// error of eps in the wrist distance changes it by ~sqrt(eps), so it is
// recovered with an error of order 1e-8. A configuration generated exactly on
// a limit came back ~8e-10 outside it and was rejected (audit 2026-11-16 #4).
// Moving the angle onto the limit is only half of the repair: the shoulder
// angle must then be recomputed for the moved elbow, or it keeps a
// first-order error. The margin is safe because every branch still has to
// pass the 2e-9 FK check -- away from the singularity, moving the elbow by
// up to 1e-7 rad moves the wrist by far more than that and the branch is
// rejected, exactly as the out-of-limit value would have been.
inline constexpr double kSingularLimitSnapMargin = tolerance::kSingularLimitSnap;

// If `value` lies outside [lower, upper] for every 2*pi turn but some turn of
// it is within `margin` of a limit, that limit; otherwise nullopt.
inline std::optional<double> nearbyLimit(double value, double lower, double upper,
                                         double margin) noexcept
{
    if (!std::isfinite(value) || !std::isfinite(lower) || !std::isfinite(upper) || lower > upper)
        return std::nullopt;
    const auto wrap = [](double x) { return x - kTwoPi * std::floor(x / kTwoPi); };
    // Distance from the nearest turn of `value` at or below `lower` up to it,
    // and from `upper` up to the nearest turn at or above it.
    const double belowLower = wrap(lower - value);
    const double aboveUpper = wrap(value - upper);
    // Some turn lies inside the limits: nothing to snap.
    if (belowLower == 0.0 || aboveUpper == 0.0 || kTwoPi - belowLower <= upper - lower)
        return std::nullopt;
    if (belowLower <= aboveUpper)
        return belowLower <= margin ? std::optional<double>{lower} : std::nullopt;
    return aboveUpper <= margin ? std::optional<double>{upper} : std::nullopt;
}

// Outer XYX angles re-solved with one of them held fixed. With alpha (or
// gamma) given, the remaining two follow from a single well-conditioned
// rotation -- unlike decomposeXyx, whose alpha and gamma separately degrade
// as sin(beta) -> 0.
//   Rx(-alpha) R = Ry(beta) Rx(gamma)      R Rx(-gamma) = Rx(alpha) Ry(beta)
inline XyxAngles xyxWithAlpha(const std::array<std::array<double, 3>, 3>& r, double alpha) noexcept
{
    const double c = std::cos(alpha);
    const double s = std::sin(alpha);
    // Rows of M = Rx(-alpha) * R that are needed.
    const double m00 = r[0][0];
    const double m11 = c * r[1][1] + s * r[2][1];
    const double m12 = c * r[1][2] + s * r[2][2];
    const double m20 = -s * r[1][0] + c * r[2][0];
    return {alpha, std::atan2(-m20, m00), std::atan2(-m12, m11), false};
}

inline XyxAngles xyxWithGamma(const std::array<std::array<double, 3>, 3>& r, double gamma) noexcept
{
    const double c = std::cos(gamma);
    const double s = std::sin(gamma);
    // Entries of N = R * Rx(-gamma) that are needed. The columns of
    // Rx(-gamma) are e0, (0, c, -s) and (0, s, c).
    const double n00 = r[0][0];
    const double n02 = s * r[0][1] + c * r[0][2];
    const double n11 = c * r[1][1] - s * r[1][2];
    const double n21 = c * r[2][1] - s * r[2][2];
    return {std::atan2(n21, n11), std::atan2(n02, n00), gamma, false};
}

struct LimitRange
{
    bool enabled{};
    double lower{};
    double upper{};
};

// A regular wrist solution (q4, q5, q6) = (alpha, beta, gamma / axis6Sign)
// next to the singularity: if q4 or q6 lies just outside its limit, put it on
// the limit and re-solve the other two from `local` for that value. Away from
// the singularity a moved q4/q6 changes the orientation and the FK check
// rejects the branch, as it would have rejected the out-of-limit value.
inline std::array<double, 3> snapWristNearLimits(const std::array<std::array<double, 3>, 3>& local,
                                                 std::array<double, 3> q, double axis6Sign,
                                                 const LimitRange& q4, const LimitRange& q6) noexcept
{
    // q4 and q6 are each recovered with an error ~e/|sin q5|, where e is the
    // orientation error reaching the wrist -- not eps: upstream angles carry
    // ~1e-13 and are amplified (measured: 8.4e-7 rad at q5 = 1e-7). The margin
    // therefore grows as kWristSnapOrientationBudget/|sin q5|. That is safe by
    // construction: moving q4 (or q6) by D and re-solving the rest changes
    // the orientation by only ~|sin q5|*D <= the budget, far below the 2e-9 FK
    // check. (q5 below the singular tolerance is a family and never gets here.)
    constexpr double kWristSnapOrientationBudget = tolerance::kWristSnapOrientationBudget;
    const double margin = std::max(kSingularLimitSnapMargin,
                                   kWristSnapOrientationBudget /
                                       std::max(std::abs(std::sin(q[1])), 1e-300));
    if (q4.enabled)
        if (const auto snapped = nearbyLimit(q[0], q4.lower, q4.upper, margin))
        {
            const auto angles = xyxWithAlpha(local, *snapped);
            q = {*snapped, angles.beta, angles.gamma / axis6Sign};
        }
    if (q6.enabled)
        if (const auto snapped = nearbyLimit(q[2], q6.lower, q6.upper, margin))
        {
            const auto angles = xyxWithGamma(local, *snapped * axis6Sign);
            q = {angles.alpha, angles.beta, *snapped};
        }
    return q;
}

// q5 of a singular wrist family is set to exactly 0 or -pi, while the target
// only puts it within the singular tolerance of that value. When the exact
// value falls just outside q5's limit, the limit is used instead; the family's
// FK check decides, as everywhere else.
inline double snapSingularWristPitch(double q5, const LimitRange& limit) noexcept
{
    if (!limit.enabled)
        return q5;
    return nearbyLimit(q5, limit.lower, limit.upper, kSingularLimitSnapMargin).value_or(q5);
}

// Precondition: withinLimit(value, lower, upper).
inline double snapToLimit(double value, double lower, double upper) noexcept
{
    return value < lower ? lower : (value > upper ? upper : value);
}

// Free-parameter interval of a singular wrist family
//     dependent = offset + multiplier * free
// intersected with the dependent joint's limits. Every value a caller will
// compute from the result -- the free bounds, and offset + multiplier * bound
// evaluated exactly like that -- lies within the limits with NO tolerance.
//
// Two numerical effects used to break that (audit 2026-11-16 #7):
//   * intervals missing each other by less than `tolerance` were joined at
//     their midpoint, a value outside one of the limits by up to tolerance/2;
//   * rounding in (limit - offset) / multiplier and back again can put the
//     dependent value at a bound a few ulps outside its limit.
// A proper interval is repaired by pulling in the offending end (a few ulps);
// a single point by shifting `offset` by the observed overshoot, which moves
// the relation by at most ~tolerance -- orders of magnitude below the 2e-9 FK
// check every branch still has to pass.
// A family that cannot be repaired within that budget is dropped, never
// returned outside its limits.
struct CoupledFreeInterval
{
    double lower{};
    double upper{};
    double offset{};
};

inline std::optional<CoupledFreeInterval>
coupledFreeInterval(double freeLower, double freeUpper, bool dependentLimited,
                    double dependentLower, double dependentUpper, double offset,
                    double multiplier, double tolerance) noexcept
{
    double lower = freeLower;
    double upper = freeUpper;
    if (dependentLimited)
    {
        const double a = (dependentLower - offset) / multiplier;
        const double b = (dependentUpper - offset) / multiplier;
        lower = std::max(lower, std::min(a, b));
        upper = std::min(upper, std::max(a, b));
    }
    if (!(lower <= upper + tolerance))
        return std::nullopt;
    if (lower > upper)
        lower = upper = std::clamp(std::midpoint(lower, upper), freeLower, freeUpper);
    if (!dependentLimited)
        return CoupledFreeInterval{lower, upper, offset};

    const double originalOffset = offset;
    const double budget =
        2.0 * tolerance * std::max(1.0, std::abs(multiplier)) +
        64.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, std::abs(originalOffset));
    constexpr double infinity = std::numeric_limits<double>::infinity();
    for (int iteration = 0; iteration < 256; ++iteration)
    {
        const double atLower = offset + multiplier * lower;
        const double atUpper = offset + multiplier * upper;
        const double low = std::min(atLower, atUpper);
        const double high = std::max(atLower, atUpper);
        const bool below = low < dependentLower;
        const bool above = high > dependentUpper;
        if (!below && !above)
            return CoupledFreeInterval{lower, upper, offset};
        if (lower < upper)
        {
            // A proper interval: pull in the end(s) whose dependent value is
            // outside, leaving the relation alone. Shifting the offset instead
            // oscillates when the interval is exactly as wide as the dependent
            // range -- it repairs one end by breaking the other.
            const double step = (below ? dependentLower - low : high - dependentUpper) /
                                std::abs(multiplier);
            // Below: the end with the smaller dependent value is out; above: the larger.
            const bool moveLower = (atLower <= atUpper) == below;
            if (below && above)
            {
                lower = std::nextafter(lower, upper);
                upper = std::nextafter(upper, lower);
            }
            else if (moveLower)
            {
                const double next = lower + step;
                lower = std::min(next == lower ? std::nextafter(lower, upper) : next, upper);
            }
            else
            {
                const double next = upper - step;
                upper = std::max(next == upper ? std::nextafter(upper, lower) : next, lower);
            }
            continue;
        }
        // A single point (the barely-touching case): only the relation can
        // move, by at most ~tolerance.
        if (below && above)
            return std::nullopt;
        double next = offset + (below ? dependentLower - low : dependentUpper - high);
        if (next == offset)
            next = std::nextafter(offset, below ? infinity : -infinity);
        offset = next;
        if (std::abs(offset - originalOffset) > budget)
            return std::nullopt;
    }
    return std::nullopt;
}

// Largest element-wise distance the solvers accept between an input
// orientation and its nearest rotation. An input that passed the 1e-8
// orthogonality/determinant validation stays below 1e-8 to first order
// (R - Q = Q*(S - I), S - I ~ (R^T R - I)/2); the bound leaves headroom and
// turns anything larger into a rejected target rather than a silent rewrite.
inline constexpr double kMaxOrientationCorrection = tolerance::kMaxOrientationCorrection;

struct NearestRotation
{
    std::array<std::array<double, 3>, 3> rotation{};
    double correction{}; // max |rotation_ij - input_ij|
    bool converged{};
};

// Orthogonal polar factor of a near-rotation, by Newton's iteration
// X <- (X + X^-T)/2 with X^-T = cof(X)/det(X). For a matrix within ~1e-8 of a
// rotation this converges quadratically in two or three steps and yields the
// rotation closest to the input in the Frobenius norm.
inline NearestRotation nearestRotation(const std::array<std::array<double, 3>, 3>& input) noexcept
{
    NearestRotation result{input, 0.0, false};
    auto& x = result.rotation;
    for (int iteration = 0; iteration < 16; ++iteration)
    {
        const std::array<std::array<double, 3>, 3> cofactor{{
            {x[1][1] * x[2][2] - x[1][2] * x[2][1],
             x[1][2] * x[2][0] - x[1][0] * x[2][2],
             x[1][0] * x[2][1] - x[1][1] * x[2][0]},
            {x[0][2] * x[2][1] - x[0][1] * x[2][2],
             x[0][0] * x[2][2] - x[0][2] * x[2][0],
             x[0][1] * x[2][0] - x[0][0] * x[2][1]},
            {x[0][1] * x[1][2] - x[0][2] * x[1][1],
             x[0][2] * x[1][0] - x[0][0] * x[1][2],
             x[0][0] * x[1][1] - x[0][1] * x[1][0]}}};
        const double determinant = x[0][0] * cofactor[0][0] + x[0][1] * cofactor[0][1] +
                                   x[0][2] * cofactor[0][2];
        if (!std::isfinite(determinant) || determinant <= 0.0)
            return result;

        double change = 0.0;
        for (std::size_t row = 0; row < 3; ++row)
            for (std::size_t column = 0; column < 3; ++column)
            {
                const double next = 0.5 * (x[row][column] + cofactor[row][column] / determinant);
                change = std::max(change, std::abs(next - x[row][column]));
                x[row][column] = next;
            }
        if (!std::isfinite(change))
            return result;
        if (change <= 4.0 * std::numeric_limits<double>::epsilon())
        {
            result.converged = true;
            break;
        }
    }

    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            result.correction = std::max(result.correction,
                                         std::abs(x[row][column] - input[row][column]));
    return result;
}

} // namespace kinemaforge::ik::detail
