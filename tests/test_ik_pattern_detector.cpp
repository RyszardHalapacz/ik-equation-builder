#include <gtest/gtest.h>

#include "ik_equations/solver/IkPatternDetector.hpp"

#include <algorithm>
#include <string>

namespace ik = kinemaforge::ik;
namespace {

ik::JointAxisGeometry rotary(std::size_t index, ik::AxisLine axis)
{
    return {index, index, "j" + std::to_string(index + 1), ik::JointType::Revolute, axis};
}

bool has(const std::vector<ik::DetectedIkPattern>& patterns, ik::IkPatternKind kind)
{
    return std::any_of(patterns.begin(), patterns.end(),
                       [kind](const auto& pattern) { return pattern.kind == kind; });
}

ik::KinematicGeometry supportedGeometry()
{
    ik::KinematicGeometry geometry;
    geometry.actuatedJoints = {
        rotary(0, {{0, 0, 0}, {0, 0, 1}}),
        rotary(1, {{0, 0, 0}, {0, 1, 0}}),
        rotary(2, {{1, 0, 0}, {0, 1, 0}}),
        rotary(3, {{2, 0, 0}, {1, 0, 0}}),
        rotary(4, {{2, 0, 0}, {0, 1, 0}}),
        rotary(5, {{2, 0, 0}, {0, 0, 1}})};
    return geometry;
}

} // namespace

TEST(IkPatternDetector, DetectsSphericalWrist)
{
    const auto patterns = ik::IkPatternDetector{}.detect(supportedGeometry());
    EXPECT_TRUE(has(patterns, ik::IkPatternKind::SphericalWrist));
}

TEST(IkPatternDetector, DetectsParallelShoulderElbow)
{
    const auto patterns = ik::IkPatternDetector{}.detect(supportedGeometry());
    EXPECT_TRUE(has(patterns, ik::IkPatternKind::ParallelShoulderElbow));
}

TEST(IkPatternDetector, DetectsCombinedSupportedFamily)
{
    const auto patterns = ik::IkPatternDetector{}.detect(supportedGeometry());
    EXPECT_TRUE(has(patterns, ik::IkPatternKind::SixRotarySphericalWrist));
}

TEST(IkPatternDetector, ReportsWristCenterEvidence)
{
    const auto patterns = ik::IkPatternDetector{}.detect(supportedGeometry());
    const auto it = std::find_if(patterns.begin(), patterns.end(), [](const auto& pattern) {
        return pattern.kind == ik::IkPatternKind::SphericalWrist;
    });
    ASSERT_TRUE(it != patterns.end());
    ASSERT_TRUE(it->evidence.wristCenter);
    EXPECT_NEAR(it->evidence.wristCenter->x, 2.0, 1e-12);
}

TEST(IkPatternDetector, RejectsSkewWristAxes)
{
    auto geometry = supportedGeometry();
    geometry.actuatedJoints[5].axis.point.z = 0.01;
    const auto patterns = ik::IkPatternDetector{}.detect(geometry);
    EXPECT_FALSE(has(patterns, ik::IkPatternKind::SphericalWrist));
    EXPECT_FALSE(has(patterns, ik::IkPatternKind::SixRotarySphericalWrist));
}

TEST(IkPatternDetector, RejectsNonParallelShoulderElbow)
{
    auto geometry = supportedGeometry();
    geometry.actuatedJoints[2].axis.direction = {1, 0, 0};
    const auto patterns = ik::IkPatternDetector{}.detect(geometry);
    EXPECT_FALSE(has(patterns, ik::IkPatternKind::ParallelShoulderElbow));
    EXPECT_FALSE(has(patterns, ik::IkPatternKind::SixRotarySphericalWrist));
}

TEST(IkPatternDetector, RejectsPrismaticJointFromCombinedFamily)
{
    auto geometry = supportedGeometry();
    geometry.actuatedJoints[0].type = ik::JointType::Prismatic;
    const auto patterns = ik::IkPatternDetector{}.detect(geometry);
    EXPECT_FALSE(has(patterns, ik::IkPatternKind::SixRotarySphericalWrist));
}

TEST(IkPatternDetector, RequiresExactlySixActuatedJointsForCombinedFamily)
{
    auto geometry = supportedGeometry();
    geometry.actuatedJoints.pop_back();
    const auto patterns = ik::IkPatternDetector{}.detect(geometry);
    EXPECT_FALSE(has(patterns, ik::IkPatternKind::SixRotarySphericalWrist));
}

TEST(IkPatternDetector, ThreeCoincidentWristAxesHaveNoUniqueCenter)
{
    auto geometry = supportedGeometry();
    geometry.actuatedJoints[3].axis = {{2, 0, 0}, {1, 0, 0}};
    geometry.actuatedJoints[4].axis = {{2, 0, 0}, {1, 0, 0}};
    geometry.actuatedJoints[5].axis = {{2, 0, 0}, {-1, 0, 0}};
    const auto patterns = ik::IkPatternDetector{}.detect(geometry);
    EXPECT_FALSE(has(patterns, ik::IkPatternKind::SphericalWrist));
}

TEST(IkPatternDetector, EmptyGeometryProducesNoPatterns)
{
    EXPECT_TRUE(ik::IkPatternDetector{}.detect({}).empty());
}
