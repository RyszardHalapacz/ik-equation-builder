#include "ik_equations/solver/IkPatternDetector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace kinemaforge::ik {
namespace {

bool rotary(JointType type) noexcept
{
    return type == JointType::Revolute || type == JointType::Continuous;
}

struct WristMatch
{
    Vector3 center;
    double residual{};
};

std::optional<WristMatch> sphericalWrist(const AxisLine& a,
                                         const AxisLine& b,
                                         const AxisLine& c)
{
    const std::array<const AxisLine*, 3> axes{&a, &b, &c};
    double maximumResidual = 0.0;
    for (std::size_t i = 0; i < axes.size(); ++i)
        for (std::size_t j = i + 1; j < axes.size(); ++j)
        {
            const double distance = axisDistance(*axes[i], *axes[j]);
            maximumResidual = std::max(maximumResidual, distance);
            if (distance > kGeometryLinearTolerance)
                return std::nullopt;
        }

    std::optional<Vector3> candidate;
    for (std::size_t i = 0; i < axes.size() && !candidate; ++i)
        for (std::size_t j = i + 1; j < axes.size() && !candidate; ++j)
            candidate = axisIntersectionPoint(*axes[i], *axes[j]);

    // Three coincident axes have no unique center. That geometry is not useful
    // to the first wrist solver even though every pair has zero distance.
    if (!candidate)
        return std::nullopt;

    for (const AxisLine* axis : axes)
    {
        const double distance = pointToAxisDistance(*candidate, *axis);
        maximumResidual = std::max(maximumResidual, distance);
        if (distance > kGeometryLinearTolerance)
            return std::nullopt;
    }

    return WristMatch{*candidate, maximumResidual};
}

} // namespace

std::vector<DetectedIkPattern> IkPatternDetector::detect(const KinematicGeometry& geometry) const
{
    std::vector<DetectedIkPattern> patterns;
    const auto& joints = geometry.actuatedJoints;

    std::optional<WristMatch> wrist;
    if (joints.size() >= 3)
    {
        const std::size_t first = joints.size() - 3;
        if (rotary(joints[first].type) && rotary(joints[first + 1].type) &&
            rotary(joints[first + 2].type))
        {
            wrist = sphericalWrist(joints[first].axis, joints[first + 1].axis,
                                   joints[first + 2].axis);
            if (wrist)
                patterns.push_back(DetectedIkPattern{
                    IkPatternKind::SphericalWrist,
                    {first, first + 1, first + 2},
                    PatternEvidence{wrist->residual, wrist->center}});
        }
    }

    bool shoulderElbowParallel = false;
    double shoulderElbowResidual = 0.0;
    if (joints.size() >= 3 && rotary(joints[1].type) && rotary(joints[2].type))
    {
        const auto& lhs = joints[1].axis.direction;
        const auto& rhs = joints[2].axis.direction;
        const Vector3 cross{lhs.y * rhs.z - lhs.z * rhs.y,
                            lhs.z * rhs.x - lhs.x * rhs.z,
                            lhs.x * rhs.y - lhs.y * rhs.x};
        shoulderElbowResidual = std::hypot(cross.x, cross.y, cross.z);
        shoulderElbowParallel = shoulderElbowResidual <= kGeometryAngularTolerance;
        if (shoulderElbowParallel)
            patterns.push_back(DetectedIkPattern{
                IkPatternKind::ParallelShoulderElbow,
                {1, 2},
                PatternEvidence{shoulderElbowResidual, std::nullopt}});
    }

    if (joints.size() == 6 && wrist && shoulderElbowParallel &&
        std::all_of(joints.begin(), joints.end(),
                    [](const JointAxisGeometry& joint) { return rotary(joint.type); }))
    {
        patterns.push_back(DetectedIkPattern{
            IkPatternKind::SixRotarySphericalWrist,
            {0, 1, 2, 3, 4, 5},
            PatternEvidence{std::max(wrist->residual, shoulderElbowResidual), wrist->center}});
    }

    return patterns;
}

} // namespace kinemaforge::ik
