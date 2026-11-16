#pragma once

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
    const double first = std::ceil((lower - base) / kTwoPi - 1e-12);
    const double last = std::floor((upper - base) / kTwoPi + 1e-12);
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
// value handed to the caller is the value that was verified.
inline constexpr double kLimitTolerance = 1e-12;

inline bool withinLimit(double value, double lower, double upper) noexcept
{
    return std::isfinite(value) && value >= lower - kLimitTolerance &&
           value <= upper + kLimitTolerance;
}

// Margin within which an ELBOW angle outside its limits is moved onto the
// nearest limit before the rest of the arm is solved for it.
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
inline constexpr double kElbowLimitSnapMargin = 1e-7;

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
// Both are repaired by shifting `offset` by the observed overshoot, which
// moves the relation by at most ~tolerance -- orders of magnitude below the
// 2e-9 FK check every branch still has to pass -- and, if the interval is
// wider than the dependent range admits, by pulling its ends in by one ulp.
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
    for (int iteration = 0; iteration < 64; ++iteration)
    {
        const double atLower = offset + multiplier * lower;
        const double atUpper = offset + multiplier * upper;
        const double low = std::min(atLower, atUpper);
        const double high = std::max(atLower, atUpper);
        const bool below = low < dependentLower;
        const bool above = high > dependentUpper;
        if (!below && !above)
            return CoupledFreeInterval{lower, upper, offset};
        if (below && above)
        {
            // Only possible while the interval spans the whole dependent range.
            if (!(lower < upper))
                return std::nullopt;
            lower = std::nextafter(lower, upper);
            upper = std::nextafter(upper, lower);
            continue;
        }
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
inline constexpr double kMaxOrientationCorrection = 2e-8;

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
