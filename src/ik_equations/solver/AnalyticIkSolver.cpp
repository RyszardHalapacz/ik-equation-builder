#include "ik_equations/solver/AnalyticIkSolver.hpp"

#include "ik_equations/builders/KinematicGeometryBuilder.hpp"
#include "ik_equations/model/IkPattern.hpp"
#include "ik_equations/solver/IkPatternDetector.hpp"
#include "ik_equations/solver/PreparedAnalyticIk.hpp"
#include "ik_equations/solver/PrimitiveSolvers.hpp"
#include "ik_equations/solver/detail/AnalyticIkKernels.hpp"
#include "ik_equations/solver/detail/AnalyticIkNumerics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
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

// A joint's position limits in the form the shared kernels take.
detail::LimitRange limitRange(const KinematicJoint& joint) noexcept
{
    return {joint.type != JointType::Continuous && joint.limits.hasPositionLimits,
            joint.limits.lower, joint.limits.upper};
}

// Seed of a positional singular family: the arm joint `freeJoint` is free and
// the other two are fixed (the free entry of `arm` is unused).
struct PositionalSeed
{
    std::size_t freeJoint{};
    std::array<double, 3> arm{};
    ElbowBranch elbow{ElbowBranch::Positive};
    IkSingularityKind singularity{IkSingularityKind::ShoulderAxis};
};

struct PositionSolution
{
    std::vector<PositionCandidate> candidates;
    std::vector<PositionalSeed> seeds;
};

std::expected<PositionSolution, AnalyticIkSolverError>
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
    // Infinite: the wrist centre is on the joint-1 axis and q1 is free. The
    // planar target is then the same for every q1, so it is solved once (at
    // q1 = 0) and each arm solution seeds a q1-free family.
    const bool shoulderFamily = q1Solutions.kind == ScalarSolutionKind::Infinite;
    if (q1Solutions.kind == ScalarSolutionKind::None)
        return PositionSolution{};
    const std::vector<double> q1Values = shoulderFamily ? std::vector<double>{0.0} : q1Solutions.values;

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

    PositionSolution result;
    for (std::size_t shoulderIndex = 0; shoulderIndex < q1Values.size(); ++shoulderIndex)
    {
        const double q1 = q1Values[shoulderIndex];
        const Vector3 unrotated = rotatePointAroundLine(targetWrist, axis1, -q1);
        const Vector3 planarTarget = projectPerpendicular(subtract(unrotated, axis2.point), d);
        const double tx = dot(planarTarget, ex);
        const double ty = dot(planarTarget, ey);

        // Shared with the generated runtime: elbow snap onto its limit and
        // detection of the folded equal-link family happen inside.
        const auto arm = detail::solvePlanarArm(tx, ty, link1, link2, homeAngle, axis3Sign,
                                                limitRange(elbowJoint), kGeometryLinearTolerance);
        // Equal links, wrist centre on the joint-2 axis: q2 is free. Never
        // return the one arbitrary member the planar solution picked.
        if (arm.foldedFamily)
        {
            if (shoulderFamily)
                return error(AnalyticIkSolverErrorCode::UnsupportedKinematicStructure,
                             "target lies on the joint-1 axis AND folds equal links onto the "
                             "joint-2 axis; a two-parameter family is not supported");
            if (!arm.branches.empty())
                result.seeds.push_back({1, {q1, 0.0, arm.branches.front().elbow},
                                        arm.branches.front().positiveElbow ? ElbowBranch::Positive
                                                                           : ElbowBranch::Negative,
                                        IkSingularityKind::ElbowFolded});
            continue;
        }
        if (shoulderFamily)
        {
            for (const auto& branch : arm.branches)
                result.seeds.push_back({0, {0.0, branch.shoulder, branch.elbow},
                                        branch.positiveElbow ? ElbowBranch::Positive : ElbowBranch::Negative,
                                        IkSingularityKind::ShoulderAxis});
            continue;
        }
        for (const auto& branch : arm.branches)
        {
            PositionCandidate candidate;
            candidate.q = {q1, branch.shoulder, branch.elbow};
            candidate.shoulder = shoulderIndex == 0 ? ShoulderBranch::Primary
                                                    : ShoulderBranch::Alternate;
            candidate.elbow = branch.positiveElbow ? ElbowBranch::Positive : ElbowBranch::Negative;
            if (branch.singular)
            {
                const double internal = homeAngle + axis3Sign * branch.elbow;
                candidate.singularity = std::cos(internal) >= 0.0
                    ? IkSingularityKind::ElbowExtended
                    : IkSingularityKind::ElbowFolded;
            }
            result.candidates.push_back(candidate);
        }
    }
    return result;
}

Matrix3 basisMatrix(const Vector3& x, const Vector3& y, const Vector3& z) noexcept
{
    // Columns are basis vectors.
    return {{{x.x, y.x, z.x}, {x.y, y.y, z.y}, {x.z, y.z, z.z}}};
}

// The wrist basis (columns: axis 4, axis 5, their cross product) and the sign
// of axis 6 along axis 4.
struct WristFrame
{
    Matrix3 basis{};
    double axis6Sign{1.0};
};

std::expected<WristFrame, AnalyticIkSolverError> wristFrame(const KinematicGeometry& geometry)
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
    return WristFrame{basisMatrix(a, b, *third), dot(c, a) >= 0.0 ? 1.0 : -1.0};
}

std::expected<std::vector<WristCandidate>, AnalyticIkSolverError>
solveWrist(const KinematicGeometry& geometry,
           const Matrix3& homeRotation,
           const Matrix3& target,
           const std::array<double, 3>& q123,
           const std::vector<const KinematicJoint*>& joints)
{
    const auto& axes = geometry.actuatedJoints;
    const auto frame = wristFrame(geometry);
    if (!frame)
        return std::unexpected(frame.error());

    const Matrix3 r123 = multiply(multiply(axisRotation(axes[0].axis.direction, q123[0]),
                                           axisRotation(axes[1].axis.direction, q123[1])),
                                  axisRotation(axes[2].axis.direction, q123[2]));
    const Matrix3 wrist = multiply(multiply(transpose(r123), target), transpose(homeRotation));
    const Matrix3& basis = frame->basis;
    const Matrix3 local = multiply(multiply(transpose(basis), wrist), basis);
    const double axis6Sign = frame->axis6Sign;
    // Shared with the generated runtime, including the limit snaps for
    // ill-conditioned angles next to the singularity.
    const auto solved = detail::solveWrist(local, axis6Sign, limitRange(*joints[3]),
                                           limitRange(*joints[4]), limitRange(*joints[5]),
                                           kGeometryAngularTolerance);

    std::vector<WristCandidate> result;
    if (!solved.singular)
    {
        for (std::size_t index = 0; index < solved.regular.size(); ++index)
        {
            const auto& q = solved.regular[index];
            result.push_back({{q[0], q[1], q[2]},
                              index == 0 ? WristBranch::Primary : WristBranch::Flip,
                              std::nullopt, std::nullopt});
        }
        return result;
    }

    WristCandidate singular;
    singular.wrist = WristBranch::Singular;
    singular.q[1] = solved.q5;
    singular.singularity = solved.aligned ? IkSingularityKind::WristAligned
                                          : IkSingularityKind::WristAntiAligned;

    CoupledFreeParameter relation;
    relation.freeActuatedJointPosition = 3;
    relation.freeJointName = joints[3]->name;
    relation.dependentActuatedJointPosition = 5;
    relation.dependentJointName = joints[5]->name;
    relation.multiplier = solved.multiplier;
    relation.offset = solved.offset;

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

std::array<double, 6> branchValues(const PositionCandidate& position,
                                   const WristCandidate& wrist) noexcept
{
    return {position.q[0], position.q[1], position.q[2],
            wrist.q[0], wrist.q[1], wrist.q[2]};
}

bool poseMatches(const NumericTransform& transform, const PoseTarget& target) noexcept
{
    constexpr double translationTolerance = tolerance::kFkPosition;
    constexpr double rotationTolerance = tolerance::kFkOrientation;
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
        if (std::abs(normalizeAngle(lhs[i] - rhs[i])) > tolerance::kSameBranch)
            return false;
    return true;
}

struct GeometricBranch
{
    PositionCandidate position;
    WristCandidate wrist;
    std::array<double, 6> q{}; // meaningful for regular branches only
};

static_assert(kMaxOrientationCorrection == detail::kMaxOrientationCorrection);

// Phases shared by solve() and solveCompact(): validation, geometry, and the
// geometric branches -- at most eight regular ones plus the wrist singular
// families -- each with the compact set of its in-limit full turns. Nothing
// here grows with the width of the joint limits.
// Members of a positional family checked against FK when it is built.
constexpr int kPositionalFamilySamples = 16;

struct CoreResult
{
    CompactAnalyticIkSolution compact;
    PoseTarget target;
    std::vector<const KinematicJoint*> joints;
    bool saturated{}; // some turn numbers do not fit int64_t
};

JointTurns jointTurns(std::size_t position, const KinematicJoint& joint, double value,
                      bool& saturated)
{
    const auto set = detail::turnSet(limitRange(joint), value);
    saturated = saturated || set.saturated;
    return {position, joint.name, set.base, set.first, set.last,
            set.limit.enabled, set.limit.lower, set.limit.upper};
}

// The target after validation: the rotation nearest to what the caller wrote
// and the distance to it.
struct Projected
{
    PoseTarget target;
    double correction{};
};

std::expected<Projected, AnalyticIkSolverError> projectTarget(const PoseTarget& requestedTarget)
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
    return Projected{PoseTarget{requestedTarget.position, projection->rotation}, projection->correction};
}

// Everything that depends on the robot and not on the target: the validated
// geometry, the detected pattern's wrist centre, and the home transform. Built
// once by a PreparedAnalyticIk, per call by the one-shot solver. `chain` is not
// owned; whoever owns this keeps it alive.
struct Prepared
{
    const KinematicChain* chain{};
    std::optional<FixedRigidTransform> tcp;
    KinematicGeometry geometry;
    NumericTransform home;
    Vector3 homeWrist;
    Vector3 wristInTarget;
};

std::expected<Prepared, AnalyticIkSolverError>
prepareRobot(const KinematicChain& chain, const std::optional<FixedRigidTransform>& tcp)
{
    const auto joints = actuatedJoints(chain);
    if (joints.size() != 6 ||
        !std::all_of(joints.begin(), joints.end(), [](const KinematicJoint* joint) {
            return rotary(joint->type);
        }))
        return error(AnalyticIkSolverErrorCode::UnsupportedKinematicStructure,
                     "analytic solver currently supports exactly six rotary joints");

    auto geometryResult = KinematicGeometryBuilder{}.build(chain);
    if (!geometryResult)
        return error(AnalyticIkSolverErrorCode::InvalidKinematicGeometry,
                     geometryResult.error().message);

    const auto patterns = IkPatternDetector{}.detect(*geometryResult);
    const DetectedIkPattern* pattern = supportedPattern(patterns);
    if (pattern == nullptr || !pattern->evidence.wristCenter)
        return error(AnalyticIkSolverErrorCode::UnsupportedKinematicStructure,
                     "no supported six-rotary spherical-wrist pattern was detected");

    const NumericTransform home = homeTransform(chain, tcp);
    if (!finite(home.rotation) || !finite(home.translation))
        return error(AnalyticIkSolverErrorCode::NonFiniteComputation,
                     "home transform is not finite");

    const Vector3 homeWrist = *pattern->evidence.wristCenter;
    return Prepared{&chain, tcp, std::move(*geometryResult), home, homeWrist,
                    multiply(transpose(home.rotation), subtract(homeWrist, home.translation))};
}

std::expected<CoreResult, AnalyticIkSolverError>
solveCore(const Prepared& prepared, const Projected& projected)
{
    const KinematicChain& chain = *prepared.chain;
    const std::optional<FixedRigidTransform>& tcp = prepared.tcp;
    const KinematicGeometry& geometry = prepared.geometry;
    const NumericTransform& home = prepared.home;
    const Vector3& homeWrist = prepared.homeWrist;
    const PoseTarget& target = projected.target;

    CoreResult core;
    core.target = target;
    core.compact.orientationCorrection = projected.correction;
    core.joints = actuatedJoints(chain);
    const auto& joints = core.joints;

    const Vector3 targetWrist = add(target.position,
                                    multiply(targetRotation(target.orientation), prepared.wristInTarget));
    if (!finite(targetWrist))
        return error(AnalyticIkSolverErrorCode::NonFiniteComputation,
                     "target wrist centre is not finite");
    {
        // |axis x v|, not sqrt(|v|^2 - (axis.v)^2): the latter loses every digit
        // of a millimetre-scale distance next to a metre-scale v.
        const AxisLine& shoulderAxis = geometry.actuatedJoints[0].axis;
        const Vector3 offset = cross(shoulderAxis.direction, subtract(targetWrist, shoulderAxis.point));
        core.compact.shoulderAxisDistance = norm(offset);
    }

    const auto positionResult = solvePosition(geometry, targetWrist, homeWrist, *joints[2]);
    if (!positionResult)
        return std::unexpected(positionResult.error());
    if (positionResult->candidates.empty() && positionResult->seeds.empty())
    {
        core.compact.message = "target wrist centre is outside the position workspace";
        return core;
    }

    // Phase A: geometric branches. At most eight regular ones plus the wrist
    // singular families; verification and deduplication happen here, where the
    // work is constant, not after the full-turn expansion.
    std::vector<GeometricBranch> geometric;
    for (const PositionCandidate& position : positionResult->candidates)
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
                geometric.push_back({position, wrist, {}});
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
            geometric.push_back({position, wrist, q});
        }
    }


    // Positional families (q1 or q2 free): one per seed and wrist branch,
    // verified with this solver's own FK at sample members.
    if (!positionResult->seeds.empty())
    {
        const auto frame = wristFrame(geometry);
        if (!frame)
            return std::unexpected(frame.error());
        const Matrix3 targetInHome = multiply(targetRotation(target.orientation), transpose(home.rotation));
        for (const PositionalSeed& seed : positionResult->seeds)
            for (std::size_t wristBranch = 0; wristBranch < 2; ++wristBranch)
            {
                PositionalFamilyBranch branch;
                PositionalFamily& family = branch.family;
                family.freeJoint = seed.freeJoint;
                family.arm = seed.arm;
                family.wristBranch = wristBranch;
                for (std::size_t i = 0; i < 3; ++i)
                {
                    const Vector3 axis = geometry.actuatedJoints[i].axis.direction;
                    family.axisDirections[i] = {axis.x, axis.y, axis.z};
                }
                family.targetInHome = targetInHome;
                family.wristBasis = frame->basis;
                family.axis6Sign = frame->axis6Sign;
                for (std::size_t i = 0; i < 6; ++i)
                {
                    const auto range = limitRange(*joints[i]);
                    family.limits[i] = {range.enabled, range.lower, range.upper};
                }
                const auto freeRange = detail::freeParameterRange(limitRange(*joints[seed.freeJoint]));
                family.freeLower = freeRange.lower;
                family.freeUpper = freeRange.upper;

                int members = 0;
                bool verified = true;
                for (int step = 0; step <= kPositionalFamilySamples && verified; ++step)
                {
                    const double value = std::lerp(family.freeLower, family.freeUpper,
                                                   static_cast<double>(step) / kPositionalFamilySamples);
                    const auto member = detail::evaluate(family, value);
                    if (!member)
                        continue;
                    ++members;
                    verified = poseMatches(numericForward(chain, *member, tcp), target);
                }
                if (!verified || members == 0)
                    continue;
                branch.freeJointName = joints[seed.freeJoint]->name;
                branch.elbow = seed.elbow;
                branch.wrist = wristBranch == 0 ? WristBranch::Primary : WristBranch::Flip;
                branch.singularity = seed.singularity;
                core.compact.positionalFamilies.push_back(std::move(branch));
            }
    }

    // Compact form of every geometric branch.
    for (const GeometricBranch& branch : geometric)
    {
        CompactSolutionBranch compact;
        compact.shoulder = branch.position.shoulder;
        compact.elbow = branch.position.elbow;
        compact.wrist = branch.wrist.wrist;
        if (branch.wrist.wrist != WristBranch::Singular)
        {
            compact.singularity = branch.wrist.singularity ? branch.wrist.singularity
                                                           : branch.position.singularity;
            for (std::size_t i = 0; i < 6; ++i)
                compact.joints.push_back(jointTurns(i, *joints[i], branch.q[i], core.saturated));
        }
        else
        {
            if (!branch.wrist.coupled)
                continue;
            compact.singularity = branch.wrist.singularity;
            const std::array<double, 4> values{branch.position.q[0], branch.position.q[1],
                                               branch.position.q[2], branch.wrist.q[1]};
            const std::array<std::size_t, 4> positions{0, 1, 2, 4};
            for (std::size_t i = 0; i < 4; ++i)
                compact.joints.push_back(jointTurns(positions[i], *joints[positions[i]], values[i],
                                                    core.saturated));
            const CoupledFreeParameter& relation = *branch.wrist.coupled;
            const auto family = detail::familyTurns(relation.multiplier, relation.offset,
                                                    limitRange(*joints[3]), limitRange(*joints[5]));
            core.saturated = core.saturated || family.saturated;
            compact.family = CompactCoupledFamily{
                relation.freeActuatedJointPosition, relation.freeJointName,
                relation.dependentActuatedJointPosition, relation.dependentJointName,
                relation.multiplier, relation.offset, family.first, family.last,
                family.free.lower, family.free.upper, family.dependent.enabled,
                family.dependent.lower, family.dependent.upper};
        }
        if (compact.count() == 0U && !core.saturated)
            continue;
        core.compact.branches.push_back(std::move(compact));
    }

    if (core.compact.branches.empty() && core.compact.positionalFamilies.empty())
    {
        core.compact.message =
            "geometric branches exist but none satisfies joint limits and pose round-trip";
        return core;
    }
    const bool family = !core.compact.positionalFamilies.empty() ||
                        std::any_of(core.compact.branches.begin(), core.compact.branches.end(),
                                    [](const CompactSolutionBranch& branch) {
                                        return branch.family.has_value();
                                    });
    core.compact.status = family ? AnalyticIkStatus::InfiniteFamily : AnalyticIkStatus::Solutions;
    return core;
}

} // namespace

namespace {

std::expected<CoreResult, AnalyticIkSolverError>
solveOnce(const KinematicChain& chain, const PoseTarget& requestedTarget,
          const std::optional<FixedRigidTransform>& tcp)
{
    // Target errors come before chain errors, as they always did.
    const auto projected = projectTarget(requestedTarget);
    if (!projected)
        return std::unexpected(projected.error());
    const auto prepared = prepareRobot(chain, tcp);
    if (!prepared)
        return std::unexpected(prepared.error());
    return solveCore(*prepared, *projected);
}

std::expected<CompactAnalyticIkSolution, AnalyticIkSolverError> finishCompact(CoreResult core)
{
    if (core.saturated)
        return std::unexpected(AnalyticIkSolverError{
            AnalyticIkSolverErrorCode::ResourceLimitExceeded,
            "joint limits span more full turns than an int64_t can number",
            std::nullopt,
            std::numeric_limits<std::size_t>::max()});
    return std::move(core.compact);
}

std::expected<AnalyticIkSolution, AnalyticIkSolverError>
finishSolve(CoreResult core, const KinematicChain& chain, const std::optional<FixedRigidTransform>& tcp,
            const AnalyticIkSolverOptions& options)
{
    const double correction = core.compact.orientationCorrection;
    if (core.compact.branches.empty())
    {
        if (core.compact.positionalFamilies.empty())
            return AnalyticIkSolution{AnalyticIkStatus::NoSolution, {}, core.compact.message, correction, {},
                                      core.compact.shoulderAxisDistance};
        return AnalyticIkSolution{AnalyticIkStatus::InfiniteFamily, {}, {}, correction,
                                  std::move(core.compact.positionalFamilies),
                                  core.compact.shoulderAxisDistance};
    }

    // Count before allocating. The number of in-limit full turns is a product
    // over joints, so whether the result fits the budget is decided from the
    // compact form alone. Exceeding it is an error, never a shortened list.
    const std::size_t required = core.saturated ? std::numeric_limits<std::size_t>::max()
                                                 : core.compact.expandedBranchCount();
    if (detail::branchBudgetExceeded(required, options.maxBranches))
        return std::unexpected(AnalyticIkSolverError{
            AnalyticIkSolverErrorCode::ResourceLimitExceeded,
            "the solution needs " +
                (required == std::numeric_limits<std::size_t>::max()
                     ? std::string{"more branches than can be counted"}
                     : std::to_string(required) + " branches") +
                ", more than the allowed " + std::to_string(options.maxBranches) +
                "; narrow the joint limits, raise AnalyticIkSolverOptions::maxBranches, "
                "or use solveCompact()",
            std::nullopt,
            required});

    // Expand, then verify every branch that will be returned.
    std::vector<SolutionBranch> verified;
    verified.reserve(required);
    for (auto& branch : expand(core.compact))
        if (branchMatches(branch, chain, core.target, tcp, core.joints))
            verified.push_back(std::move(branch));

    auto& families = core.compact.positionalFamilies;
    if (verified.empty() && families.empty())
        return AnalyticIkSolution{AnalyticIkStatus::NoSolution, {},
                                  "geometric branches exist but none satisfies joint limits and pose round-trip",
                                  correction, {}, core.compact.shoulderAxisDistance};

    const bool infinite = !families.empty() ||
                          std::any_of(verified.begin(), verified.end(), [](const auto& branch) {
                              return !branch.freeParameters.empty() || !branch.coupledFreeParameters.empty();
                          });
    return AnalyticIkSolution{infinite ? AnalyticIkStatus::InfiniteFamily
                                      : AnalyticIkStatus::Solutions,
                              std::move(verified), {}, correction, std::move(families),
                              core.compact.shoulderAxisDistance};
}

} // namespace

std::expected<CompactAnalyticIkSolution, AnalyticIkSolverError>
AnalyticIkSolver::solveCompact(const KinematicChain& chain,
                               const PoseTarget& requestedTarget,
                               const std::optional<FixedRigidTransform>& tcp) const
{
    auto core = solveOnce(chain, requestedTarget, tcp);
    if (!core)
        return std::unexpected(core.error());
    return finishCompact(std::move(*core));
}

std::optional<std::array<double, 6>> evaluate(const PositionalFamily& family, double value)
{
    return detail::evaluate(family, value);
}

std::vector<SolutionBranch> expand(const CompactAnalyticIkSolution& solution)
{
    std::vector<SolutionBranch> output;
    for (const CompactSolutionBranch& branch : solution.branches)
    {
        if (branch.count() == 0U)
            continue;
        const auto& joints = branch.joints;

        SolutionBranch base;
        base.shoulder = branch.shoulder;
        base.elbow = branch.elbow;
        base.wrist = branch.wrist;
        base.singularity = branch.singularity;

        // One combination of joint turns, with every member of the family.
        const auto emit = [&](const std::vector<std::int64_t>& turn) {
            SolutionBranch member = base;
            for (std::size_t i = 0; i < joints.size(); ++i)
                member.joints.push_back({joints[i].actuatedJointPosition, joints[i].jointName,
                                         joints[i].value(turn[i])});
            if (!branch.family)
            {
                output.push_back(std::move(member));
                return;
            }
            const CompactCoupledFamily& family = *branch.family;
            detail::FamilyTurns turns;
            turns.multiplier = family.multiplier;
            turns.offset = family.baseOffset;
            turns.free = {true, family.freeLower, family.freeUpper};
            turns.dependent = {family.dependentLimited, family.dependentLower, family.dependentUpper};
            for (auto k = family.firstTurn; k <= family.lastTurn; ++k)
            {
                const auto interval = turns.interval(k);
                if (!interval)
                    continue;
                SolutionBranch withFamily = member;
                withFamily.freeParameters.push_back({family.freeActuatedJointPosition,
                                                     family.freeJointName, interval->lower,
                                                     interval->upper});
                withFamily.coupledFreeParameters.push_back(
                    {family.freeActuatedJointPosition, family.freeJointName,
                     family.dependentActuatedJointPosition, family.dependentJointName,
                     family.multiplier, interval->offset});
                output.push_back(std::move(withFamily));
            }
        };

        // Odometer over the joints' turn numbers.
        std::vector<std::int64_t> turn;
        for (const auto& joint : joints)
            turn.push_back(joint.firstTurn);
        while (true)
        {
            emit(turn);
            std::size_t index = joints.size();
            while (index > 0)
            {
                --index;
                if (++turn[index] <= joints[index].lastTurn)
                    break;
                turn[index] = joints[index].firstTurn;
            }
            if (index == 0 && turn[0] == joints[0].firstTurn)
                break;
        }
    }
    return output;
}

std::expected<AnalyticIkSolution, AnalyticIkSolverError>
AnalyticIkSolver::solve(const KinematicChain& chain,
                        const PoseTarget& requestedTarget,
                        const std::optional<FixedRigidTransform>& tcp,
                        const AnalyticIkSolverOptions& options) const
{
    auto core = solveOnce(chain, requestedTarget, tcp);
    if (!core)
        return std::unexpected(core.error());
    return finishSolve(std::move(*core), chain, tcp, options);
}

double wristPitchSine(const SolutionBranch& branch)
{
    if (branch.wrist == WristBranch::Singular)
        return 0.0;
    for (const JointSolutionValue& joint : branch.joints)
        if (joint.actuatedJointPosition == 4)
            return std::abs(std::sin(joint.value));
    return 0.0;
}

struct PreparedAnalyticIk::Impl
{
    KinematicChain chain; // Prepared::chain points here
    Prepared prepared;

    Impl() = default;
    Impl(const Impl& other) : chain(other.chain), prepared(other.prepared)
    {
        prepared.chain = &chain; // never the other object's chain
    }
};

PreparedAnalyticIk::PreparedAnalyticIk(const PreparedAnalyticIk& other)
    : impl_(other.impl_ ? std::make_unique<Impl>(*other.impl_) : nullptr)
{
}

PreparedAnalyticIk& PreparedAnalyticIk::operator=(const PreparedAnalyticIk& other)
{
    if (this != &other)
        impl_ = other.impl_ ? std::make_unique<Impl>(*other.impl_) : nullptr;
    return *this;
}

PreparedAnalyticIk::PreparedAnalyticIk(PreparedAnalyticIk&&) noexcept = default;
PreparedAnalyticIk& PreparedAnalyticIk::operator=(PreparedAnalyticIk&&) noexcept = default;
PreparedAnalyticIk::~PreparedAnalyticIk() = default;

std::expected<PreparedAnalyticIk, AnalyticIkSolverError>
PreparedAnalyticIk::create(const KinematicChain& chain, const std::optional<FixedRigidTransform>& tcp)
{
    auto impl = std::make_unique<Impl>();
    impl->chain = chain;
    auto prepared = prepareRobot(impl->chain, tcp);
    if (!prepared)
        return std::unexpected(prepared.error());
    impl->prepared = std::move(*prepared);
    PreparedAnalyticIk result;
    result.impl_ = std::move(impl);
    return result;
}

std::expected<AnalyticIkSolution, AnalyticIkSolverError>
PreparedAnalyticIk::solve(const PoseTarget& requestedTarget, const AnalyticIkSolverOptions& options) const
{
    const auto projected = projectTarget(requestedTarget);
    if (!projected)
        return std::unexpected(projected.error());
    auto core = solveCore(impl_->prepared, *projected);
    if (!core)
        return std::unexpected(core.error());
    return finishSolve(std::move(*core), impl_->chain, impl_->prepared.tcp, options);
}

std::expected<CompactAnalyticIkSolution, AnalyticIkSolverError>
PreparedAnalyticIk::solveCompact(const PoseTarget& requestedTarget) const
{
    const auto projected = projectTarget(requestedTarget);
    if (!projected)
        return std::unexpected(projected.error());
    auto core = solveCore(impl_->prepared, *projected);
    if (!core)
        return std::unexpected(core.error());
    return finishCompact(std::move(*core));
}

std::expected<JacobianAnalysis, AnalyticIkSolverError>
PreparedAnalyticIk::jacobian(const std::array<double, 6>& q) const
{
    for (const double value : q)
        if (!std::isfinite(value))
            return error(AnalyticIkSolverErrorCode::NonFiniteComputation,
                         "joint configuration is not finite");

    // Every joint rotates everything downstream of it about its own axis, which
    // earlier joints have already moved. Doing that in order puts each axis in
    // its place for this configuration; the tool point is carried along.
    const Prepared& prepared = impl_->prepared;
    std::array<AxisLine, 6> axes;
    for (std::size_t i = 0; i < 6; ++i)
        axes[i] = prepared.geometry.actuatedJoints[i].axis;
    Vector3 tool = prepared.home.translation;
    for (std::size_t i = 0; i < 6; ++i)
    {
        const Matrix3 rotation = axisRotation(axes[i].direction, q[i]);
        const AxisLine moving = axes[i];
        for (std::size_t j = i + 1; j < 6; ++j)
        {
            axes[j].point = rotatePointAroundLine(axes[j].point, moving, q[i]);
            axes[j].direction = multiply(rotation, axes[j].direction);
        }
        tool = rotatePointAroundLine(tool, moving, q[i]);
    }

    JacobianAnalysis analysis;
    for (std::size_t joint = 0; joint < 6; ++joint)
    {
        const Vector3 d = axes[joint].direction;
        const Vector3 linear = cross(d, subtract(tool, axes[joint].point));
        analysis.jacobian[0][joint] = linear.x;
        analysis.jacobian[1][joint] = linear.y;
        analysis.jacobian[2][joint] = linear.z;
        analysis.jacobian[3][joint] = d.x;
        analysis.jacobian[4][joint] = d.y;
        analysis.jacobian[5][joint] = d.z;
    }

    // One-sided Jacobi (Hestenes) SVD: rotate pairs of columns until all are
    // mutually orthogonal; the singular values are then the column norms.
    std::array<std::array<double, 6>, 6> u = analysis.jacobian;
    for (int sweep = 0; sweep < 60; ++sweep)
    {
        bool rotated = false;
        for (std::size_t p = 0; p + 1 < 6; ++p)
            for (std::size_t r = p + 1; r < 6; ++r)
            {
                double alpha = 0.0;
                double beta = 0.0;
                double gamma = 0.0;
                for (std::size_t row = 0; row < 6; ++row)
                {
                    alpha += u[row][p] * u[row][p];
                    beta += u[row][r] * u[row][r];
                    gamma += u[row][p] * u[row][r];
                }
                if (gamma == 0.0 || std::abs(gamma) <= 1e-15 * std::sqrt(alpha * beta))
                    continue;
                rotated = true;
                const double zeta = (beta - alpha) / (2.0 * gamma);
                const double t = std::copysign(1.0, zeta) / (std::abs(zeta) + std::sqrt(1.0 + zeta * zeta));
                const double c = 1.0 / std::sqrt(1.0 + t * t);
                const double s = c * t;
                for (std::size_t row = 0; row < 6; ++row)
                {
                    const double a = u[row][p];
                    const double b = u[row][r];
                    u[row][p] = c * a - s * b;
                    u[row][r] = s * a + c * b;
                }
            }
        if (!rotated)
            break;
    }
    for (std::size_t column = 0; column < 6; ++column)
    {
        double sum = 0.0;
        for (std::size_t row = 0; row < 6; ++row)
            sum += u[row][column] * u[row][column];
        analysis.singularValues[column] = std::sqrt(sum);
    }
    std::sort(analysis.singularValues.begin(), analysis.singularValues.end(), std::greater<>{});
    analysis.minSingularValue = analysis.singularValues.back();
    analysis.condition = analysis.minSingularValue == 0.0
                             ? std::numeric_limits<double>::infinity()
                             : analysis.singularValues.front() / analysis.minSingularValue;
    return analysis;
}

} // namespace kinemaforge::ik
