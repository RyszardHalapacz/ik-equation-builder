#pragma once

// IK math shared by the general AnalyticIkSolver and the generated runtime.
//
// Each piece exists exactly once. The two solvers differ only in how they get
// their geometry (a KinematicGeometry vs baked numeric parameters) and how
// they verify FK; everything that decides WHICH joint values come out is here.
// Two parallel copies used to drift apart -- one of the 2026-11-16 audit
// findings was exactly such a divergence (a status reported by one solver and
// not the other).
//
// Header-only and free of model types, because generated code includes it.

#include "ik_equations/model/IkTolerances.hpp"
#include "ik_equations/model/PositionalFamily.hpp"
#include "ik_equations/solver/detail/AnalyticIkNumerics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <numbers>
#include <numeric>
#include <vector>

namespace kinemaforge::ik::detail {

// Angle in [-pi, pi). Non-finite input is returned unchanged.
inline double normalizeAngle(double angle) noexcept
{
    if (!std::isfinite(angle))
        return angle;
    double result = std::remainder(angle, kTwoPi);
    if (result >= std::numbers::pi_v<double>)
        result -= kTwoPi;
    return result;
}

inline void appendDistinctAngle(std::vector<double>& values, double value, double tolerance)
{
    const double normalized = normalizeAngle(value);
    for (double existing : values)
        if (std::abs(normalizeAngle(existing - normalized)) <= tolerance)
            return;
    values.push_back(normalized);
}

// --- a*sin(x) + b*cos(x) = c ------------------------------------------------

enum class SinCosKind
{
    None,
    Finite,
    Infinite // a = b = c = 0 within tolerance: every x solves it
};

struct SinCosSolutions
{
    SinCosKind kind{SinCosKind::None};
    std::vector<double> values; // normalised, distinct; only for Finite
};

inline SinCosSolutions solveSinCos(double a, double b, double c, double tolerance)
{
    if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c) ||
        tolerance < 0.0 || !std::isfinite(tolerance))
        return {};

    const double amplitude = std::hypot(a, b);
    if (amplitude <= tolerance)
        return {std::abs(c) <= tolerance ? SinCosKind::Infinite : SinCosKind::None, {}};

    const double ratio = c / amplitude;
    if (ratio < -1.0 - tolerance || ratio > 1.0 + tolerance)
        return {};

    // a*sin(x)+b*cos(x) = r*sin(x+phi), phi = atan2(b, a).
    const double phase = std::atan2(b, a);
    const double first = std::asin(std::clamp(ratio, -1.0, 1.0));
    std::vector<double> shifted;
    appendDistinctAngle(shifted, first, tolerance);
    appendDistinctAngle(shifted, std::numbers::pi_v<double> - first, tolerance);

    SinCosSolutions result{SinCosKind::Finite, {}};
    for (double value : shifted)
        appendDistinctAngle(result.values, value - phase, tolerance);
    return result;
}

// --- Planar 2R arm (joints 2 and 3) -------------------------------------------

struct ArmBranch
{
    double shoulder{};     // joint 2
    double elbow{};        // joint 3, normalised
    bool positiveElbow{};  // internal elbow angle >= 0
    bool singular{};       // elbow straight or folded
};

struct PlanarArmSolutions
{
    std::vector<ArmBranch> branches;
    // Equal links and the target on the shoulder axis (elbow fully folded):
    // the shoulder angle is free. `branches` then holds one arbitrary member;
    // a solver must report the case instead of returning it.
    bool foldedFamily{};
};

// Target (x, y) in the arm plane, link lengths, the elbow's home angle and
// direction sign. With elbowLimit enabled, an ill-conditioned elbow angle just
// outside it (see kSingularLimitSnapMargin) is put on the limit and the
// shoulder re-solved for that elbow; the caller's FK check decides.
inline PlanarArmSolutions solvePlanarArm(double x, double y, double link1, double link2,
                                         double homeAngle, double elbowSign,
                                         const LimitRange& elbowLimit, double tolerance)
{
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(link1) || !std::isfinite(link2) ||
        !std::isfinite(homeAngle) || !std::isfinite(elbowSign) || !std::isfinite(tolerance) ||
        tolerance < 0.0 || link1 <= tolerance || link2 <= tolerance ||
        std::abs(std::abs(elbowSign) - 1.0) > tolerance)
        return {};

    PlanarArmSolutions result;
    result.foldedFamily = std::hypot(x, y) <= tolerance && std::abs(link1 - link2) <= tolerance;

    const double cosine = (x * x + y * y - link1 * link1 - link2 * link2) / (2.0 * link1 * link2);
    if (cosine < -1.0 - tolerance || cosine > 1.0 + tolerance)
        return result;

    const double magnitude = std::acos(std::clamp(cosine, -1.0, 1.0));
    const double targetAngle = std::atan2(y, x);
    for (double internal : {magnitude, -magnitude})
    {
        const bool positive = internal >= 0.0;
        double elbow = (internal - homeAngle) / elbowSign;
        if (elbowLimit.enabled)
            if (const auto snapped = nearbyLimit(elbow, elbowLimit.lower, elbowLimit.upper,
                                                 kSingularLimitSnapMargin))
            {
                elbow = *snapped;
                internal = homeAngle + elbowSign * elbow;
            }
        const double vx = link1 + link2 * std::cos(internal);
        const double vy = link2 * std::sin(internal);
        const double shoulder = normalizeAngle(targetAngle - std::atan2(vy, vx));
        const double normalizedElbow = normalizeAngle(elbow);

        const bool duplicate = std::any_of(result.branches.begin(), result.branches.end(),
                                           [&](const ArmBranch& branch) {
            return std::abs(normalizeAngle(branch.shoulder - shoulder)) <= tolerance &&
                   std::abs(normalizeAngle(branch.elbow - normalizedElbow)) <= tolerance;
        });
        if (!duplicate)
            result.branches.push_back({shoulder, normalizedElbow, positive,
                                       std::abs(std::sin(internal)) <= tolerance});
    }
    return result;
}

// --- Spherical wrist (joints 4, 5, 6) --------------------------------------

struct WristSolutions
{
    bool singular{};
    // Regular: the primary and the flipped branch, each (q4, q5, q6).
    std::array<std::array<double, 3>, 2> regular{};
    // Singular (|sin q5| below tolerance): q5 fixed, q4 free and
    //   q6 = offset + multiplier * q4   (modulo 2*pi).
    bool aligned{}; // q5 ~ 0 (true) or q5 ~ -pi (false)
    double q5{};
    double multiplier{};
    double offset{};
};

// local = Rx(q4) Ry(q5) Rx(axis6Sign * q6) in the wrist basis. Limits enable
// the snaps for ill-conditioned angles next to the singularity (see
// snapWristNearLimits / snapSingularWristPitch); disabled ranges skip them.
inline WristSolutions solveWrist(const std::array<std::array<double, 3>, 3>& local,
                                 double axis6Sign, const LimitRange& q4, const LimitRange& q5,
                                 const LimitRange& q6, double singularTolerance) noexcept
{
    const auto angles = decomposeXyx(local, singularTolerance);
    WristSolutions result;
    if (!angles.singular)
    {
        constexpr double pi = std::numbers::pi_v<double>;
        result.regular[0] = snapWristNearLimits(
            local,
            {normalizeAngle(angles.alpha), normalizeAngle(angles.beta),
             normalizeAngle(angles.gamma / axis6Sign)},
            axis6Sign, q4, q6);
        result.regular[1] = snapWristNearLimits(
            local,
            {normalizeAngle(angles.alpha + pi), normalizeAngle(-angles.beta),
             normalizeAngle((angles.gamma + pi) / axis6Sign)},
            axis6Sign, q4, q6);
        return result;
    }
    result.singular = true;
    result.aligned = local[0][0] >= 0.0;
    result.q5 = snapSingularWristPitch(result.aligned ? 0.0 : -std::numbers::pi_v<double>, q5);
    result.multiplier = (result.aligned ? -1.0 : 1.0) / axis6Sign;
    result.offset = angles.gamma / axis6Sign;
    return result;
}

// --- Full turns of a joint value within its limits ---------------------------

// The values base + 2*pi*k, k = first..last, that lie within a joint's limits
// (admitted kLimitTolerance outside and snapped onto the limit). This is the
// compact form of a joint's alternatives: its size does not depend on how wide
// the limits are. Unlimited joints have exactly one value, the normalised one.
struct TurnSet
{
    double base{};
    std::int64_t first{0};
    std::int64_t last{-1};
    LimitRange limit{};
    // Turn numbers beyond int64_t: not empty, but not representable either.
    bool saturated{};

    [[nodiscard]] std::size_t count() const noexcept
    {
        if (saturated)
            return std::numeric_limits<std::size_t>::max();
        return turnCount(TurnRange{first, last});
    }

    [[nodiscard]] double value(std::int64_t turn) const noexcept
    {
        const double candidate = base + kTwoPi * static_cast<double>(turn);
        return limit.enabled ? snapToLimit(candidate, limit.lower, limit.upper) : candidate;
    }
};

inline TurnSet turnSet(const LimitRange& limit, double value) noexcept
{
    TurnSet set;
    set.limit = limit;
    if (!std::isfinite(value))
        return set; // empty
    set.base = normalizeAngle(value);
    if (!limit.enabled)
    {
        set.first = set.last = 0;
        return set;
    }
    const auto turns = turnsWithin(limit.lower - kLimitTolerance, limit.upper + kLimitTolerance, set.base);
    if (turns.saturated)
    {
        set.saturated = true;
        return set;
    }
    set.first = turns.first;
    set.last = turns.last;
    // turnsWithin carries round-off slack; only admitted values belong here.
    while (set.first <= set.last &&
           !withinLimit(set.base + kTwoPi * static_cast<double>(set.first), limit.lower, limit.upper))
        ++set.first;
    while (set.last >= set.first &&
           !withinLimit(set.base + kTwoPi * static_cast<double>(set.last), limit.lower, limit.upper))
        --set.last;
    return set;
}

// Number of values equivalents() returns, without building them; O(1).
inline std::size_t equivalentCount(const LimitRange& limit, double value) noexcept
{
    return turnSet(limit, value).count();
}

// The TurnSet materialised. Precondition for a limited joint: its count was
// checked against the caller's branch budget, so the vector is small.
inline std::vector<double> equivalents(const LimitRange& limit, double value)
{
    const TurnSet set = turnSet(limit, value);
    std::vector<double> result;
    if (set.saturated)
        return result;
    for (auto k = set.first; k <= set.last; ++k)
        result.push_back(set.value(k));
    return result;
}

// --- Singular wrist family: dependent = offset + multiplier * free -----------

// The free joint's admissible range; an unbounded free axis needs one turn.
inline LimitRange freeParameterRange(const LimitRange& free) noexcept
{
    if (free.enabled)
        return free;
    return {true, -std::numbers::pi_v<double>, std::numbers::pi_v<double>};
}

// Integers k for which offset + multiplier*free + 2*pi*k can lie within the
// dependent joint's limits for some admissible free value. An unbounded
// dependent axis needs exactly one modulo relation.
inline TurnRange dependentTurns(double multiplier, double offset, const LimitRange& free,
                                const LimitRange& dependent) noexcept
{
    if (!dependent.enabled)
        return {0, 0};
    const LimitRange range = freeParameterRange(free);
    const double a = offset + multiplier * range.lower;
    const double b = offset + multiplier * range.upper;
    return turnsWithin(dependent.lower - std::max(a, b), dependent.upper - std::min(a, b));
}

// The dependent turns k of a singular family whose relation admits a
// non-empty free interval, in compact form: free interval and relation offset
// per k come from interval(k). Ends whose interval is empty are trimmed.
struct FamilyTurns
{
    double multiplier{};
    double offset{};
    std::int64_t first{0};
    std::int64_t last{-1};
    LimitRange free{};      // effective range of the free joint
    LimitRange dependent{};
    bool saturated{};

    [[nodiscard]] std::size_t count() const noexcept
    {
        if (saturated)
            return std::numeric_limits<std::size_t>::max();
        return turnCount(TurnRange{first, last});
    }

    [[nodiscard]] std::optional<CoupledFreeInterval> interval(std::int64_t turn) const noexcept
    {
        return coupledFreeInterval(free.lower, free.upper, dependent.enabled, dependent.lower,
                                   dependent.upper, offset + kTwoPi * static_cast<double>(turn),
                                   multiplier, tolerance::kRoundOff);
    }
};

inline FamilyTurns familyTurns(double multiplier, double offset, const LimitRange& freeJoint,
                               const LimitRange& dependent) noexcept
{
    FamilyTurns family;
    family.multiplier = multiplier;
    family.offset = offset;
    family.free = freeParameterRange(freeJoint);
    family.dependent = dependent;
    const TurnRange turns = dependentTurns(multiplier, offset, freeJoint, dependent);
    if (turns.saturated)
    {
        family.saturated = true;
        return family;
    }
    family.first = turns.first;
    family.last = turns.last;
    while (family.first <= family.last && !family.interval(family.first))
        ++family.first;
    while (family.last >= family.first && !family.interval(family.last))
        --family.last;
    return family;
}

// --- Positional singular families (q1 or q2 free) -----------------------------

inline LimitRange limitRange(const PositionalLimit& limit) noexcept
{
    return {limit.enabled, limit.lower, limit.upper};
}

// Rotation by angle q about the unit axis a (Rodrigues).
inline std::array<std::array<double, 3>, 3> axisRotation(const std::array<double, 3>& a, double q) noexcept
{
    const double c = std::cos(q);
    const double s = std::sin(q);
    const double t = 1.0 - c;
    return {{{c + a[0] * a[0] * t, a[0] * a[1] * t - a[2] * s, a[0] * a[2] * t + a[1] * s},
             {a[1] * a[0] * t + a[2] * s, c + a[1] * a[1] * t, a[1] * a[2] * t - a[0] * s},
             {a[2] * a[0] * t - a[1] * s, a[2] * a[1] * t + a[0] * s, c + a[2] * a[2] * t}}};
}

inline std::array<std::array<double, 3>, 3> multiply3(const std::array<std::array<double, 3>, 3>& a,
                                                      const std::array<std::array<double, 3>, 3>& b) noexcept
{
    std::array<std::array<double, 3>, 3> r{};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            for (std::size_t k = 0; k < 3; ++k)
                r[i][j] += a[i][k] * b[k][j];
    return r;
}

inline std::array<std::array<double, 3>, 3> transpose3(const std::array<std::array<double, 3>, 3>& m) noexcept
{
    std::array<std::array<double, 3>, 3> r{};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            r[i][j] = m[j][i];
    return r;
}

// The member of a positional family at the given free value, every joint on
// an in-limit turn; nothing if the value is outside the free interval or the
// member leaves a joint limit. A wrist singular at this member returns the
// family member with q4 in the middle of its admissible interval.
inline std::optional<std::array<double, 6>> evaluate(const PositionalFamily& family, double value)
{
    if (!std::isfinite(value) || value < family.freeLower || value > family.freeUpper ||
        family.freeJoint > 1U || family.wristBranch > 1U)
        return std::nullopt;

    std::array<double, 3> arm = family.arm;
    arm[family.freeJoint] = value;
    const auto r123 = multiply3(multiply3(axisRotation(family.axisDirections[0], arm[0]),
                                          axisRotation(family.axisDirections[1], arm[1])),
                                axisRotation(family.axisDirections[2], arm[2]));
    const auto wristRotation = multiply3(transpose3(r123), family.targetInHome);
    const auto local = multiply3(multiply3(transpose3(family.wristBasis), wristRotation), family.wristBasis);

    const auto& l = family.limits;
    const auto wrist = solveWrist(local, family.axis6Sign, limitRange(l[3]), limitRange(l[4]),
                                  limitRange(l[5]), tolerance::kSingularAngular);
    std::array<double, 6> q{arm[0], arm[1], arm[2], 0.0, 0.0, 0.0};
    bool wristSet = false;
    if (wrist.singular)
    {
        const auto turns = familyTurns(wrist.multiplier, wrist.offset, limitRange(l[3]), limitRange(l[5]));
        if (turns.saturated || turns.count() == 0U)
            return std::nullopt;
        const auto interval = turns.interval(turns.first);
        if (!interval)
            return std::nullopt;
        q[3] = std::midpoint(interval->lower, interval->upper);
        q[4] = wrist.q5;
        q[5] = interval->offset + wrist.multiplier * q[3];
        wristSet = true;
    }
    else
    {
        const auto& w = wrist.regular[family.wristBranch];
        q[3] = w[0];
        q[4] = w[1];
        q[5] = w[2];
    }

    // Every joint on an in-limit turn; the free value is used as given.
    for (std::size_t i = 0; i < 6; ++i)
    {
        if (i == family.freeJoint)
        {
            if (l[i].enabled && !withinLimit(value, l[i].lower, l[i].upper))
                return std::nullopt;
            continue;
        }
        if (wristSet && (i == 3 || i == 5))
            continue;
        const TurnSet set = turnSet(limitRange(l[i]), q[i]);
        if (set.saturated || set.count() == 0U)
            return std::nullopt;
        q[i] = set.value(set.first);
    }
    return q;
}

} // namespace kinemaforge::ik::detail
