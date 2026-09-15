#include "ik_equations/solver/AnalyticIkSolver.hpp"

#include "ik_equations/builders/KinematicGeometryBuilder.hpp"
#include "ik_equations/model/IkPattern.hpp"
#include "ik_equations/solver/IkPatternDetector.hpp"
#include "ik_equations/solver/PrimitiveSolvers.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
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
    return std::unexpected(AnalyticIkSolverError{code, std::move(message), std::nullopt});
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
              const Vector3& homeWrist)
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

        const auto planar = solvePlanar2R(tx, ty, link1, link2, homeAngle, axis3Sign,
                                          kGeometryLinearTolerance);
        for (const auto& branch : planar)
        {
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
    const double beta = std::acos(std::clamp(local[0][0], -1.0, 1.0));
    const double sineBeta = std::sin(beta);

    std::vector<WristCandidate> result;
    if (std::abs(sineBeta) > kGeometryAngularTolerance)
    {
        const double alpha = std::atan2(local[1][0], -local[2][0]);
        const double gamma = std::atan2(local[0][1], local[0][2]);
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
        const double theta = std::atan2(local[2][1], local[1][1]);
        relation.multiplier = -1.0 / axis6Sign;
        relation.offset = theta / axis6Sign;
        singular.singularity = IkSingularityKind::WristAligned;
    }
    else
    {
        const double theta = std::atan2(local[1][2], local[1][1]);
        relation.multiplier = 1.0 / axis6Sign;
        relation.offset = -theta / axis6Sign;
        singular.singularity = IkSingularityKind::WristAntiAligned;
    }

    singular.coupled = relation;
    result.push_back(std::move(singular));
    return result;
}

bool withinPositionLimit(const KinematicJoint& joint, double value) noexcept
{
    if (!std::isfinite(value))
        return false;
    if (joint.type == JointType::Continuous || !joint.limits.hasPositionLimits)
        return true;
    return value >= joint.limits.lower - kPrimitiveSolverTolerance &&
           value <= joint.limits.upper + kPrimitiveSolverTolerance;
}

std::vector<double> equivalentsWithinLimits(const KinematicJoint& joint, double angle)
{
    if (joint.type == JointType::Continuous || !joint.limits.hasPositionLimits)
        return {normalizeAngle(angle)};

    std::vector<double> result;
    constexpr double period = 2.0 * std::numbers::pi_v<double>;
    const double base = normalizeAngle(angle);
    const long first = static_cast<long>(std::ceil((joint.limits.lower - base) / period - 1e-12));
    const long last = static_cast<long>(std::floor((joint.limits.upper - base) / period + 1e-12));
    for (long k = first; k <= last; ++k)
    {
        const double value = base + period * static_cast<double>(k);
        if (withinPositionLimit(joint, value))
            result.push_back(value);
    }
    return result;
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
    const double freeLower = freeJoint.type == JointType::Continuous || !freeJoint.limits.hasPositionLimits
        ? -std::numbers::pi_v<double> : freeJoint.limits.lower;
    const double freeUpper = freeJoint.type == JointType::Continuous || !freeJoint.limits.hasPositionLimits
        ? std::numbers::pi_v<double> : freeJoint.limits.upper;

    constexpr double period = 2.0 * std::numbers::pi_v<double>;
    for (double q1 : q1s)
        for (double q2 : q2s)
            for (double q3 : q3s)
                for (double q5 : q5s)
                {
                    // dependent = offset + multiplier*free + 2*pi*k.  Search
                    // the small set of k that can intersect the dependent limits.
                    for (int k = -2; k <= 2; ++k)
                    {
                        CoupledFreeParameter relation = *wrist.coupled;
                        relation.offset += period * static_cast<double>(k);

                        double lower = freeLower;
                        double upper = freeUpper;
                        if (dependentJoint.type != JointType::Continuous &&
                            dependentJoint.limits.hasPositionLimits)
                        {
                            const double a = (dependentJoint.limits.lower - relation.offset) /
                                             relation.multiplier;
                            const double b = (dependentJoint.limits.upper - relation.offset) /
                                             relation.multiplier;
                            lower = std::max(lower, std::min(a, b));
                            upper = std::min(upper, std::max(a, b));
                        }
                        if (lower > upper + kPrimitiveSolverTolerance)
                            continue;

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
    if (norm(subtract(transform.translation, target.position)) > translationTolerance)
        return false;
    const Matrix3 expected = targetRotation(target.orientation);
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            if (std::abs(transform.rotation[row][column] - expected[row][column]) > rotationTolerance)
                return false;
    return true;
}

bool duplicateRegular(const SolutionBranch& lhs, const SolutionBranch& rhs) noexcept
{
    if (lhs.joints.size() != 6 || rhs.joints.size() != 6)
        return false;
    for (std::size_t i = 0; i < 6; ++i)
        if (std::abs(normalizeAngle(lhs.joints[i].value - rhs.joints[i].value)) > 1e-9)
            return false;
    return true;
}

} // namespace

std::expected<AnalyticIkSolution, AnalyticIkSolverError>
AnalyticIkSolver::solve(const KinematicChain& chain,
                        const PoseTarget& target,
                        const std::optional<FixedRigidTransform>& tcp) const
{
    if (const auto checked = validate(target); !checked)
        return std::unexpected(AnalyticIkSolverError{
            AnalyticIkSolverErrorCode::InvalidTarget,
            checked.error().message,
            checked.error()});

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

    const auto positionResult = solvePosition(geometry, targetWrist, homeWrist);
    if (!positionResult)
        return std::unexpected(positionResult.error());
    if (positionResult->empty())
        return AnalyticIkSolution{AnalyticIkStatus::NoSolution, {},
                                  "target wrist centre is outside the position workspace"};

    std::vector<SolutionBranch> branches;
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
                expandSingularWrist(position, wrist, joints, branches);
                continue;
            }

            const auto q = branchValues(position, wrist);
            // This verification is deliberately inside the solver.  Analytic
            // derivation bugs must fail closed: no branch is returned unless
            // the production kinematic convention maps it back to the target.
            if (!poseMatches(numericForward(chain, q, tcp), target))
                continue;
            expandRegularBranch(q, position, wrist, joints, branches);
        }
    }

    std::vector<SolutionBranch> deduplicated;
    for (auto& branch : branches)
    {
        if (branch.wrist != WristBranch::Singular &&
            std::any_of(deduplicated.begin(), deduplicated.end(), [&](const auto& existing) {
                return duplicateRegular(existing, branch);
            }))
            continue;
        deduplicated.push_back(std::move(branch));
    }

    if (deduplicated.empty())
        return AnalyticIkSolution{AnalyticIkStatus::NoSolution, {},
                                  "geometric branches exist but none satisfies joint limits and pose round-trip"};

    const bool infinite = std::any_of(deduplicated.begin(), deduplicated.end(), [](const auto& branch) {
        return !branch.freeParameters.empty() || !branch.coupledFreeParameters.empty();
    });
    return AnalyticIkSolution{infinite ? AnalyticIkStatus::InfiniteFamily
                                      : AnalyticIkStatus::Solutions,
                              std::move(deduplicated), {}};
}

} // namespace kinemaforge::ik
