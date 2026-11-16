#include "ik_equations/solver/AnalyticIkSolver.hpp"

#include "ik_equations/builders/KinematicGeometryBuilder.hpp"
#include "ik_equations/model/IkPattern.hpp"
#include "ik_equations/solver/IkPatternDetector.hpp"
#include "ik_equations/solver/PrimitiveSolvers.hpp"
#include "ik_equations/solver/detail/AnalyticIkNumerics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace kinemaforge::ik {
namespace {

using Matrix3 = std::array<std::array<double, 3>, 3>;

struct NumericTransform
{
    Matrix3 rotation{};
    Vector3 translation{};
};

struct PositionCandidate
{
    std::array<double, 3> q{};
    ShoulderBranch shoulder{ShoulderBranch::Primary};
    ElbowBranch elbow{ElbowBranch::Positive};
    std::optional<IkSingularityKind> singularity;
};

struct WristCandidate
{
    std::array<double, 3> q{};
    WristBranch wrist{WristBranch::Primary};
    std::optional<IkSingularityKind> singularity;
    std::optional<CoupledFreeParameter> coupled;
};

Vector3 add(const Vector3& a, const Vector3& b) noexcept
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vector3 subtract(const Vector3& a, const Vector3& b) noexcept
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vector3 scale(const Vector3& value, double factor) noexcept
{
    return {value.x * factor, value.y * factor, value.z * factor};
}

double dot(const Vector3& a, const Vector3& b) noexcept
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector3 cross(const Vector3& a, const Vector3& b) noexcept
{
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

double norm(const Vector3& value) noexcept
{
    return std::hypot(value.x, value.y, value.z);
}

std::optional<Vector3> normalized(const Vector3& value) noexcept
{
    const double length = norm(value);
    if (!std::isfinite(length) || length == 0.0)
        return std::nullopt;
    return scale(value, 1.0 / length);
}

Matrix3 identity() noexcept
{
    return {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
}

Matrix3 transpose(const Matrix3& value) noexcept
{
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            result[row][column] = value[column][row];
    return result;
}

Matrix3 multiply(const Matrix3& lhs, const Matrix3& rhs) noexcept
{
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            for (std::size_t k = 0; k < 3; ++k)
                result[row][column] += lhs[row][k] * rhs[k][column];
    return result;
}

Vector3 multiply(const Matrix3& matrix, const Vector3& vector) noexcept
{
    return {matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z,
            matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z,
            matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z};
}

Matrix3 rpyRotation(const Vector3& rpy) noexcept
{
    const double cr = std::cos(rpy.x);
    const double sr = std::sin(rpy.x);
    const double cp = std::cos(rpy.y);
    const double sp = std::sin(rpy.y);
    const double cy = std::cos(rpy.z);
    const double sy = std::sin(rpy.z);

    return {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
             {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
             {-sp, cp * sr, cp * cr}}};
}

Matrix3 axisRotation(const Vector3& axis, double angle) noexcept
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double oneMinusC = 1.0 - c;
    const double x = axis.x;
    const double y = axis.y;
    const double z = axis.z;

    return {{{c + x * x * oneMinusC,
              x * y * oneMinusC - z * s,
              x * z * oneMinusC + y * s},
             {y * x * oneMinusC + z * s,
              c + y * y * oneMinusC,
              y * z * oneMinusC - x * s},
             {z * x * oneMinusC - y * s,
              z * y * oneMinusC + x * s,
              c + z * z * oneMinusC}}};
}

Vector3 rotatePointAroundLine(const Vector3& point, const AxisLine& line, double angle) noexcept
{
    return add(line.point, multiply(axisRotation(line.direction, angle), subtract(point, line.point)));
}

Vector3 projectPerpendicular(const Vector3& value, const Vector3& axis) noexcept
{
    return subtract(value, scale(axis, dot(value, axis)));
}

Matrix3 targetRotation(const RotationMatrix3& rotation) noexcept
{
    return rotation.values;
}

NumericTransform compose(const NumericTransform& lhs, const NumericTransform& rhs) noexcept
{
    return {multiply(lhs.rotation, rhs.rotation),
            add(lhs.translation, multiply(lhs.rotation, rhs.translation))};
}

NumericTransform fixedTransform(const FixedRigidTransform& transform) noexcept
{
    return {rpyRotation(transform.rpy), transform.translation};
}

NumericTransform numericForward(const KinematicChain& chain,
                                const std::array<double, 6>& q,
                                const std::optional<FixedRigidTransform>& tcp)
{
    NumericTransform result{identity(), {}};
    std::size_t actuated = 0;

    for (const KinematicJoint& joint : chain.joints)
    {
        result = compose(result, NumericTransform{rpyRotation(joint.origin.rpy),
                                                  joint.origin.translation});

        switch (joint.type)
        {
        case JointType::Fixed:
            break;
        case JointType::Revolute:
        case JointType::Continuous:
        {
            const auto axis = normalized(joint.axis);
            if (!axis || actuated >= q.size())
                return {};
            result.rotation = multiply(result.rotation, axisRotation(*axis, q[actuated++]));
            break;
        }
        case JointType::Prismatic:
            if (actuated >= q.size())
                return {};
            result.translation = add(result.translation,
                                     multiply(result.rotation, scale(joint.axis, q[actuated++])));
            break;
        }
    }

    if (tcp)
        result = compose(result, fixedTransform(*tcp));
    return result;
}

NumericTransform homeTransform(const KinematicChain& chain,
                               const std::optional<FixedRigidTransform>& tcp)
{
    return numericForward(chain, {}, tcp);
}

bool finite(const Matrix3& matrix) noexcept
{
    for (const auto& row : matrix)
        for (double value : row)
            if (!std::isfinite(value))
                return false;
    return true;
}

bool finite(const Vector3& vector) noexcept
{
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

std::unexpected<AnalyticIkSolverError> error(AnalyticIkSolverErrorCode code,
                                             std::string message)
{
    return std::unexpected(AnalyticIkSolverError{code, std::move(message), std::nullopt, std::nullopt});
}

bool rotary(JointType type) noexcept
{
    return type == JointType::Revolute || type == JointType::Continuous;
}

std::vector<const KinematicJoint*> actuatedJoints(const KinematicChain& chain)
{
    std::vector<const KinematicJoint*> result;
    for (const auto& joint : chain.joints)
        if (joint.type != JointType::Fixed)
            result.push_back(&joint);
    return result;
}

const DetectedIkPattern* supportedPattern(const std::vector<DetectedIkPattern>& patterns)
{
    const auto it = std::find_if(patterns.begin(), patterns.end(), [](const auto& pattern) {
        return pattern.kind == IkPatternKind::SixRotarySphericalWrist;
    });
    return it == patterns.end() ? nullptr : &*it;
}

std::expected<std::vector<PositionCandidate>, AnalyticIkSolverError>
solvePosition(const KinematicGeometry& geometry, const Vector3& targetWrist,
              const Vector3& homeWrist, const KinematicJoint& elbowJoint)
{
    const auto& joints = geometry.actuatedJoints;
    const AxisLine& axis1 = joints[0].axis;
    const AxisLine& axis2 = joints[1].axis;
    const AxisLine& axis3 = joints[2].axis;

    const Vector3 v = subtract(targetWrist, axis1.point);
    const double d1d2 = dot(axis1.direction, axis2.direction);
    const double d1v = dot(axis1.direction, v);
    const double cosCoefficient = dot(v, axis2.direction) - d1d2 * d1v;
    const double sinCoefficient = -dot(cross(axis1.direction, v), axis2.direction);
    const double constant = dot(axis1.point, axis2.direction) + d1d2 * d1v -
                            dot(homeWrist, axis2.direction);

    const ScalarSolutions q1Solutions = solveSinCos(sinCoefficient, cosCoefficient, -constant,
                                                     kGeometryLinearTolerance);
    if (q1Solutions.kind == ScalarSolutionKind::Infinite)
        return error(AnalyticIkSolverErrorCode::UnsupportedKinematicStructure,
                     "target lies on a shoulder-axis singular family; the first solver does not "
                     "parameterise q1-dependent position branches");
    if (q1Solutions.kind == ScalarSolutionKind::None)
        return std::vector<PositionCandidate>{};

    const Vector3 d = axis2.direction;
    const Vector3 delta23 = subtract(axis3.point, axis2.point);
    const Vector3 v1 = projectPerpendicular(delta23, d);
    const double link1 = norm(v1);
    if (link1 <= kGeometryLinearTolerance)
        return error(AnalyticIkSolverErrorCode::UnsupportedKinematicStructure,
                     "joint 2 and joint 3 axes are coincident; planar 2R link 1 is zero");

    const Vector3 p3 = add(axis2.point, v1);
    const Vector3 v2 = projectPerpendicular(subtract(homeWrist, p3), d);
    const double link2 = norm(v2);
    if (link2 <= kGeometryLinearTolerance)
        return error(AnalyticIkSolverErrorCode::UnsupportedKinematicStructure,
                     "joint 3 axis passes through the wrist centre; planar 2R link 2 is zero");

    const Vector3 ex = scale(v1, 1.0 / link1);
    const Vector3 ey = cross(d, ex);
    const double homeAngle = std::atan2(dot(v2, ey), dot(v2, ex));
    const double axis3Sign = dot(axis3.direction, d) >= 0.0 ? 1.0 : -1.0;

    std::vector<PositionCandidate> result;
    for (std::size_t shoulderIndex = 0; shoulderIndex < q1Solutions.values.size(); ++shoulderIndex)
    {
        const double q1 = q1Solutions.values[shoulderIndex];
        const Vector3 unrotated = rotatePointAroundLine(targetWrist, axis1, -q1);
        const Vector3 planarTarget = projectPerpendicular(subtract(unrotated, axis2.point), d);
        const double tx = dot(planarTarget, ex);
        const double ty = dot(planarTarget, ey);

        // Equal links and the wrist centre on the joint-2 axis (elbow fully
        // folded): q2 is free. solvePlanar2R would pick one q2 from
        // atan2(0, 0), silently dropping the rest of the family.
        if (std::hypot(tx, ty) <= kGeometryLinearTolerance &&
            std::abs(link1 - link2) <= kGeometryLinearTolerance)
            return error(AnalyticIkSolverErrorCode::UnsupportedKinematicStructure,
                         "target lies on an elbow-folded singular family (equal links, wrist "
                         "centre on the joint-2 axis); the first solver does not parameterise "
                         "q2-dependent position branches");

        const auto planar = solvePlanar2R(tx, ty, link1, link2, homeAngle, axis3Sign,
                                          kGeometryLinearTolerance);
        const bool elbowLimited = elbowJoint.type != JointType::Continuous &&
                                  elbowJoint.limits.hasPositionLimits;
        for (auto branch : planar)
        {
            // An ill-conditioned elbow angle just outside its limit: put it on
            // the limit and re-solve the shoulder angle for it. See
            // detail::kElbowLimitSnapMargin; the FK check decides.
            if (elbowLimited)
                if (const auto snapped = detail::nearbyLimit(
                        branch.secondJoint, elbowJoint.limits.lower, elbowJoint.limits.upper,
                        detail::kElbowLimitSnapMargin))
                {
                    const double internal = homeAngle + axis3Sign * *snapped;
                    branch.secondJoint = *snapped;
                    branch.firstJoint = normalizeAngle(
                        std::atan2(ty, tx) - std::atan2(link2 * std::sin(internal),
                                                        link1 + link2 * std::cos(internal)));
                }
            PositionCandidate candidate;
            candidate.q = {q1, branch.firstJoint, branch.secondJoint};
            candidate.shoulder = shoulderIndex == 0 ? ShoulderBranch::Primary
                                                    : ShoulderBranch::Alternate;
            candidate.elbow = branch.elbow;
            if (branch.singular)
            {
                const double internal = homeAngle + axis3Sign * branch.secondJoint;
                candidate.singularity = std::cos(internal) >= 0.0
                    ? IkSingularityKind::ElbowExtended
                    : IkSingularityKind::ElbowFolded;
            }
            result.push_back(candidate);
        }
    }
    return result;
}

Matrix3 basisMatrix(const Vector3& x, const Vector3& y, const Vector3& z) noexcept
{
    // Columns are basis vectors.
    return {{{x.x, y.x, z.x}, {x.y, y.y, z.y}, {x.z, y.z, z.z}}};
}

std::expected<std::vector<WristCandidate>, AnalyticIkSolverError>
solveWrist(const KinematicGeometry& geometry,
           const Matrix3& homeRotation,
           const Matrix3& target,
           const std::array<double, 3>& q123,
           const std::vector<const KinematicJoint*>& joints)
{
    const auto& axes = geometry.actuatedJoints;
    const Vector3 a = axes[3].axis.direction;
    const Vector3 b = axes[4].axis.direction;
    const Vector3 c = axes[5].axis.direction;

    if (!axesParallel(axes[3].axis, axes[5].axis) ||
        !axesPerpendicular(axes[3].axis, axes[4].axis))
        return error(AnalyticIkSolverErrorCode::UnsupportedKinematicStructure,
                     "the first wrist solver requires axes 4/6 parallel and axis 5 perpendicular");

    const auto third = normalized(cross(a, b));
    if (!third)
        return error(AnalyticIkSolverErrorCode::UnsupportedKinematicStructure,
                     "wrist basis is degenerate");

    const Matrix3 r123 = multiply(multiply(axisRotation(axes[0].axis.direction, q123[0]),
                                           axisRotation(axes[1].axis.direction, q123[1])),
                                  axisRotation(axes[2].axis.direction, q123[2]));
    const Matrix3 wrist = multiply(multiply(transpose(r123), target), transpose(homeRotation));
    const Matrix3 basis = basisMatrix(a, b, *third);
    const Matrix3 local = multiply(multiply(transpose(basis), wrist), basis);

    const double axis6Sign = dot(c, a) >= 0.0 ? 1.0 : -1.0;
    const auto angles = detail::decomposeXyx(local, kGeometryAngularTolerance);

    std::vector<WristCandidate> result;
    if (!angles.singular)
    {
        const double alpha = angles.alpha;
        const double beta = angles.beta;
        const double gamma = angles.gamma;
        result.push_back({{normalizeAngle(alpha), normalizeAngle(beta),
                           normalizeAngle(gamma / axis6Sign)},
                          WristBranch::Primary, std::nullopt, std::nullopt});
        result.push_back({{normalizeAngle(alpha + std::numbers::pi_v<double>),
                           normalizeAngle(-beta),
                           normalizeAngle((gamma + std::numbers::pi_v<double>) / axis6Sign)},
                          WristBranch::Flip, std::nullopt, std::nullopt});
        return result;
    }

    WristCandidate singular;
    singular.wrist = WristBranch::Singular;
    singular.q[1] = local[0][0] >= 0.0 ? 0.0 : -std::numbers::pi_v<double>;

    CoupledFreeParameter relation;
    relation.freeActuatedJointPosition = 3;
    relation.freeJointName = joints[3]->name;
    relation.dependentActuatedJointPosition = 5;
    relation.dependentJointName = joints[5]->name;

    if (local[0][0] >= 0.0)
    {
        relation.multiplier = -1.0 / axis6Sign;
        relation.offset = angles.gamma / axis6Sign;
        singular.singularity = IkSingularityKind::WristAligned;
    }
    else
    {
        relation.multiplier = 1.0 / axis6Sign;
        relation.offset = angles.gamma / axis6Sign;
        singular.singularity = IkSingularityKind::WristAntiAligned;
    }

    singular.coupled = relation;
    result.push_back(std::move(singular));
    return result;
}

bool limited(const KinematicJoint& joint) noexcept
{
    return joint.type != JointType::Continuous && joint.limits.hasPositionLimits;
}

bool withinPositionLimit(const KinematicJoint& joint, double value) noexcept
{
    if (!std::isfinite(value))
        return false;
    if (!limited(joint))
        return true;
    return detail::withinLimit(value, joint.limits.lower, joint.limits.upper);
}

// How many values equivalentsWithinLimits() would return, without building
// them. An upper estimate: a value on the very edge of the turn range can
// still fail the limit check.
std::size_t equivalentCount(const KinematicJoint& joint, double angle) noexcept
{
    if (!std::isfinite(angle))
        return 0U;
    if (!limited(joint))
        return 1U;
    return detail::turnCount(detail::turnsWithin(joint.limits.lower, joint.limits.upper,
                                                 normalizeAngle(angle)));
}

// Precondition for bounded joints: the caller has checked equivalentCount()
// against the branch budget, so the vector built here is small.
std::vector<double> equivalentsWithinLimits(const KinematicJoint& joint, double angle)
{
    if (!limited(joint))
        return {normalizeAngle(angle)};

    std::vector<double> result;
    constexpr double period = 2.0 * std::numbers::pi_v<double>;
    const double base = normalizeAngle(angle);
    const auto turns = detail::turnsWithin(joint.limits.lower, joint.limits.upper, base);
    for (auto k = turns.first; k <= turns.last; ++k)
    {
        const double value = base + period * static_cast<double>(k);
        // A value within the numerical margin of a limit IS that limit; snap it
        // so the caller never receives a number outside [lower, upper]. The
        // final FK check runs on the snapped value.
        if (withinPositionLimit(joint, value))
            result.push_back(detail::snapToLimit(value, joint.limits.lower, joint.limits.upper));
    }
    return result;
}

struct FreeRange
{
    double lower{};
    double upper{};
};

FreeRange freeRange(const KinematicJoint& joint) noexcept
{
    if (limited(joint))
        return {joint.limits.lower, joint.limits.upper};
    return {-std::numbers::pi_v<double>, std::numbers::pi_v<double>};
}

// Integers k for which dependent = offset + multiplier*free + 2*pi*k can fall
// inside the dependent joint's limits for some admissible free value. An
// unbounded dependent axis needs exactly one modulo relation.
detail::TurnRange dependentTurns(const CoupledFreeParameter& relation,
                                 const KinematicJoint& freeJoint,
                                 const KinematicJoint& dependentJoint) noexcept
{
    if (!limited(dependentJoint))
        return {0, 0};
    const FreeRange range = freeRange(freeJoint);
    const double a = relation.offset + relation.multiplier * range.lower;
    const double b = relation.offset + relation.multiplier * range.upper;
    return detail::turnsWithin(dependentJoint.limits.lower - std::max(a, b),
                               dependentJoint.limits.upper - std::min(a, b));
}

std::size_t regularBranchCount(const std::array<double, 6>& q,
                               const std::vector<const KinematicJoint*>& joints) noexcept
{
    std::size_t count = 1U;
    for (std::size_t index = 0; index < q.size(); ++index)
        count = detail::saturatingMultiply(count, equivalentCount(*joints[index], q[index]));
    return count;
}

std::size_t singularBranchCount(const PositionCandidate& position,
                                const WristCandidate& wrist,
                                const std::vector<const KinematicJoint*>& joints) noexcept
{
    if (!wrist.coupled)
        return 0U;
    std::size_t count = equivalentCount(*joints[0], position.q[0]);
    count = detail::saturatingMultiply(count, equivalentCount(*joints[1], position.q[1]));
    count = detail::saturatingMultiply(count, equivalentCount(*joints[2], position.q[2]));
    count = detail::saturatingMultiply(count, equivalentCount(*joints[4], wrist.q[1]));
    return detail::saturatingMultiply(
        count, detail::turnCount(dependentTurns(*wrist.coupled, *joints[3], *joints[5])));
}

void expandRegularBranch(const std::array<double, 6>& q,
                         const PositionCandidate& position,
                         const WristCandidate& wrist,
                         const std::vector<const KinematicJoint*>& joints,
                         std::vector<SolutionBranch>& output)
{
    std::array<std::vector<double>, 6> alternatives;
    for (std::size_t index = 0; index < q.size(); ++index)
    {
        alternatives[index] = equivalentsWithinLimits(*joints[index], q[index]);
        if (alternatives[index].empty())
            return;
    }

    std::array<std::size_t, 6> cursor{};
    while (true)
    {
        SolutionBranch branch;
        branch.shoulder = position.shoulder;
        branch.elbow = position.elbow;
        branch.wrist = wrist.wrist;
        branch.singularity = wrist.singularity ? wrist.singularity : position.singularity;
        for (std::size_t index = 0; index < q.size(); ++index)
            branch.joints.push_back({index, joints[index]->name, alternatives[index][cursor[index]]});
        output.push_back(std::move(branch));

        std::size_t index = q.size();
        while (index > 0)
        {
            --index;
            if (++cursor[index] < alternatives[index].size())
                break;
            cursor[index] = 0;
        }
        if (index == 0 && cursor[0] == 0)
            break;
    }
}

void expandSingularWrist(const PositionCandidate& position,
                         const WristCandidate& wrist,
                         const std::vector<const KinematicJoint*>& joints,
                         std::vector<SolutionBranch>& output)
{
    if (!wrist.coupled)
        return;

    const auto q1s = equivalentsWithinLimits(*joints[0], position.q[0]);
    const auto q2s = equivalentsWithinLimits(*joints[1], position.q[1]);
    const auto q3s = equivalentsWithinLimits(*joints[2], position.q[2]);
    const auto q5s = equivalentsWithinLimits(*joints[4], wrist.q[1]);
    if (q1s.empty() || q2s.empty() || q3s.empty() || q5s.empty())
        return;

    const KinematicJoint& freeJoint = *joints[3];
    const KinematicJoint& dependentJoint = *joints[5];
    const FreeRange free = freeRange(freeJoint);

    constexpr double period = 2.0 * std::numbers::pi_v<double>;
    const detail::TurnRange turns = dependentTurns(*wrist.coupled, freeJoint, dependentJoint);
    for (double q1 : q1s)
        for (double q2 : q2s)
            for (double q3 : q3s)
                for (double q5 : q5s)
                {
                    // dependent = offset + multiplier*free + 2*pi*k; the
                    // admissible turns follow from BOTH joints' limits.
                    for (auto k = turns.first; k <= turns.last; ++k)
                    {
                        CoupledFreeParameter relation = *wrist.coupled;
                        relation.offset += period * static_cast<double>(k);

                        const auto interval = detail::coupledFreeInterval(
                            free.lower, free.upper, limited(dependentJoint),
                            dependentJoint.limits.lower, dependentJoint.limits.upper,
                            relation.offset, relation.multiplier, kPrimitiveSolverTolerance);
                        if (!interval)
                            continue;
                        relation.offset = interval->offset;
                        const double lower = interval->lower;
                        const double upper = interval->upper;

                        SolutionBranch branch;
                        branch.shoulder = position.shoulder;
                        branch.elbow = position.elbow;
                        branch.wrist = WristBranch::Singular;
                        branch.singularity = wrist.singularity;
                        branch.joints = {{0, joints[0]->name, q1},
                                         {1, joints[1]->name, q2},
                                         {2, joints[2]->name, q3},
                                         {4, joints[4]->name, q5}};
                        branch.freeParameters.push_back({3, joints[3]->name, lower, upper});
                        branch.coupledFreeParameters.push_back(std::move(relation));
                        output.push_back(std::move(branch));
                    }
                }
}

std::array<double, 6> branchValues(const PositionCandidate& position,
                                   const WristCandidate& wrist) noexcept
{
    return {position.q[0], position.q[1], position.q[2],
            wrist.q[0], wrist.q[1], wrist.q[2]};
}

bool poseMatches(const NumericTransform& transform, const PoseTarget& target) noexcept
{
    constexpr double translationTolerance = 2e-9;
    constexpr double rotationTolerance = 2e-9;
    if (!finite(transform.translation) || !finite(transform.rotation))
        return false;
    if (norm(subtract(transform.translation, target.position)) > translationTolerance)
        return false;
    const Matrix3 expected = targetRotation(target.orientation);
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            if (std::abs(transform.rotation[row][column] - expected[row][column]) > rotationTolerance)
                return false;
    return true;
}

bool branchMatches(const SolutionBranch& branch, const KinematicChain& chain,
                   const PoseTarget& target, const std::optional<FixedRigidTransform>& tcp,
                   const std::vector<const KinematicJoint*>& joints)
{
    std::array<double, 6> q{};
    for (const auto& joint : branch.joints)
        q[joint.actuatedJointPosition] = joint.value;
    const auto matches = [&] {
        for (std::size_t i = 0; i < q.size(); ++i)
            if (!withinPositionLimit(*joints[i], q[i]))
                return false;
        return poseMatches(numericForward(chain, q, tcp), target);
    };
    if (branch.freeParameters.empty())
        return matches();

    const auto& free = branch.freeParameters.front();
    const auto& relation = branch.coupledFreeParameters.front();
    for (double value : {free.lower, std::midpoint(free.lower, free.upper), free.upper})
    {
        q[free.actuatedJointPosition] = value;
        q[relation.dependentActuatedJointPosition] = relation.offset + relation.multiplier * value;
        if (!matches())
            return false;
    }
    return true;
}

// Two geometric branches that agree modulo a full turn on every axis expand to
// the same set of in-limit configurations, so one of them is dropped here --
// before expansion, where there are at most eight of them.
bool sameGeometricBranch(const std::array<double, 6>& lhs,
                         const std::array<double, 6>& rhs) noexcept
{
    for (std::size_t i = 0; i < lhs.size(); ++i)
        if (std::abs(normalizeAngle(lhs[i] - rhs[i])) > 1e-9)
            return false;
    return true;
}

struct GeometricBranch
{
    PositionCandidate position;
    WristCandidate wrist;
    std::array<double, 6> q{}; // meaningful for regular branches only
    std::size_t count{};       // branches this one expands to (upper estimate)
};

static_assert(kMaxOrientationCorrection == detail::kMaxOrientationCorrection);

} // namespace

std::expected<AnalyticIkSolution, AnalyticIkSolverError>
AnalyticIkSolver::solve(const KinematicChain& chain,
                        const PoseTarget& requestedTarget,
                        const std::optional<FixedRigidTransform>& tcp,
                        const AnalyticIkSolverOptions& options) const
{
    if (const auto checked = validate(requestedTarget); !checked)
        return std::unexpected(AnalyticIkSolverError{
            AnalyticIkSolverErrorCode::InvalidTarget,
            checked.error().message,
            checked.error(),
            std::nullopt});

    // Angles can only ever reproduce an exact rotation. A matrix accepted at
    // the edge of the validation band is not one, so every branch would fail
    // the 2e-9 round-trip against it. Solve for the nearest rotation instead
    // and report how far it is from what the caller wrote.
    const auto projection = nearestRotation(requestedTarget.orientation);
    if (!projection)
        return std::unexpected(AnalyticIkSolverError{
            AnalyticIkSolverErrorCode::InvalidTarget,
            projection.error().message,
            projection.error(),
            std::nullopt});
    if (projection->correction > kMaxOrientationCorrection)
        return error(AnalyticIkSolverErrorCode::InvalidTarget,
                     "target orientation is further from a rotation than the solver may correct");
    const PoseTarget target{requestedTarget.position, projection->rotation};
    const double correction = projection->correction;

    const auto joints = actuatedJoints(chain);
    if (joints.size() != 6 ||
        !std::all_of(joints.begin(), joints.end(), [](const KinematicJoint* joint) {
            return rotary(joint->type);
        }))
        return error(AnalyticIkSolverErrorCode::UnsupportedKinematicStructure,
                     "analytic solver currently supports exactly six rotary joints");

    const auto geometryResult = KinematicGeometryBuilder{}.build(chain);
    if (!geometryResult)
        return error(AnalyticIkSolverErrorCode::InvalidKinematicGeometry,
                     geometryResult.error().message);
    const KinematicGeometry& geometry = *geometryResult;

    const auto patterns = IkPatternDetector{}.detect(geometry);
    const DetectedIkPattern* pattern = supportedPattern(patterns);
    if (pattern == nullptr || !pattern->evidence.wristCenter)
        return error(AnalyticIkSolverErrorCode::UnsupportedKinematicStructure,
                     "no supported six-rotary spherical-wrist pattern was detected");

    const NumericTransform home = homeTransform(chain, tcp);
    if (!finite(home.rotation) || !finite(home.translation))
        return error(AnalyticIkSolverErrorCode::NonFiniteComputation,
                     "home transform is not finite");

    const Vector3 homeWrist = *pattern->evidence.wristCenter;
    const Vector3 wristInTarget = multiply(transpose(home.rotation),
                                           subtract(homeWrist, home.translation));
    const Vector3 targetWrist = add(target.position,
                                    multiply(targetRotation(target.orientation), wristInTarget));
    if (!finite(targetWrist))
        return error(AnalyticIkSolverErrorCode::NonFiniteComputation,
                     "target wrist centre is not finite");

    const auto positionResult = solvePosition(geometry, targetWrist, homeWrist, *joints[2]);
    if (!positionResult)
        return std::unexpected(positionResult.error());
    if (positionResult->empty())
        return AnalyticIkSolution{AnalyticIkStatus::NoSolution, {},
                                  "target wrist centre is outside the position workspace",
                                  correction};

    // Phase A: geometric branches. At most eight regular ones plus the wrist
    // singular families; verification and deduplication happen here, where the
    // work is constant, not after the full-turn expansion.
    std::vector<GeometricBranch> geometric;
    for (const PositionCandidate& position : *positionResult)
    {
        const auto wristResult = solveWrist(geometry, home.rotation,
                                            targetRotation(target.orientation),
                                            position.q, joints);
        if (!wristResult)
            return std::unexpected(wristResult.error());

        for (const WristCandidate& wrist : *wristResult)
        {
            if (wrist.wrist == WristBranch::Singular)
            {
                geometric.push_back({position, wrist, {},
                                     singularBranchCount(position, wrist, joints)});
                continue;
            }

            const auto q = branchValues(position, wrist);
            // This verification is deliberately inside the solver.  Analytic
            // derivation bugs must fail closed: no branch is returned unless
            // the production kinematic convention maps it back to the target.
            if (!poseMatches(numericForward(chain, q, tcp), target))
                continue;
            if (std::any_of(geometric.begin(), geometric.end(), [&](const GeometricBranch& existing) {
                    return existing.wrist.wrist != WristBranch::Singular &&
                           sameGeometricBranch(existing.q, q);
                }))
                continue;
            geometric.push_back({position, wrist, q, regularBranchCount(q, joints)});
        }
    }

    // Phase B: count before allocating. The number of in-limit full turns is a
    // product over joints, so it is decided from the limits alone whether the
    // result fits the budget. Exceeding it is an error, never a shortened list.
    std::size_t required = 0U;
    for (const GeometricBranch& branch : geometric)
        required = detail::saturatingAdd(required, branch.count);
    if (detail::branchBudgetExceeded(required, options.maxBranches))
        return std::unexpected(AnalyticIkSolverError{
            AnalyticIkSolverErrorCode::ResourceLimitExceeded,
            "the solution needs " +
                (required == std::numeric_limits<std::size_t>::max()
                     ? std::string{"more branches than can be counted"}
                     : std::to_string(required) + " branches") +
                ", more than the allowed " +
                std::to_string(options.maxBranches) +
                "; narrow the joint limits or raise AnalyticIkSolverOptions::maxBranches",
            std::nullopt,
            required});

    // Phase C: expand, then verify every branch that will be returned.
    std::vector<SolutionBranch> expanded;
    expanded.reserve(required);
    for (const GeometricBranch& branch : geometric)
    {
        if (branch.count == 0U)
            continue;
        if (branch.wrist.wrist == WristBranch::Singular)
            expandSingularWrist(branch.position, branch.wrist, joints, expanded);
        else
            expandRegularBranch(branch.q, branch.position, branch.wrist, joints, expanded);
    }

    std::vector<SolutionBranch> verified;
    verified.reserve(expanded.size());
    for (auto& branch : expanded)
        if (branchMatches(branch, chain, target, tcp, joints))
            verified.push_back(std::move(branch));

    if (verified.empty())
        return AnalyticIkSolution{AnalyticIkStatus::NoSolution, {},
                                  "geometric branches exist but none satisfies joint limits and pose round-trip",
                                  correction};

    const bool infinite = std::any_of(verified.begin(), verified.end(), [](const auto& branch) {
        return !branch.freeParameters.empty() || !branch.coupledFreeParameters.empty();
    });
    return AnalyticIkSolution{infinite ? AnalyticIkStatus::InfiniteFamily
                                      : AnalyticIkStatus::Solutions,
                              std::move(verified), {}, correction};
}

} // namespace kinemaforge::ik
