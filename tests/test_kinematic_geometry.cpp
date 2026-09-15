#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/builders/KinematicGeometryBuilder.hpp"

#include <cmath>
#include <filesystem>
#include <limits>
#include <numbers>
#include <string>

namespace ik = kinemaforge::ik;
namespace {

ik::KinematicJoint makeJoint(std::string name, ik::JointType type,
                             ik::Vector3 translation, ik::Vector3 rpy,
                             ik::Vector3 axis, std::size_t index)
{
    ik::KinematicJoint joint;
    joint.name = std::move(name);
    joint.type = type;
    joint.origin = {translation, rpy};
    joint.axis = axis;
    joint.index = index;
    if (type != ik::JointType::Fixed)
        joint.variable = ik::JointVariable{"q" + std::to_string(index + 1), index};
    return joint;
}

ik::KinematicGeometry loadGeometry(const char* fileName)
{
    ik::UrdfModelLoader loader;
    const auto robot = loader.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName);
    const auto chain = ik::KinematicChainBuilder{}.build(robot, "base_link", "tool0");
    if (!chain)
        return {};
    const auto geometry = ik::KinematicGeometryBuilder{}.build(*chain);
    return geometry ? *geometry : ik::KinematicGeometry{};
}

} // namespace

TEST(KinematicGeometry, EmptyChainBuildsEmptyGeometry)
{
    const auto result = ik::KinematicGeometryBuilder{}.build({"base", "base", {}});
    ASSERT_TRUE(result);
    EXPECT_TRUE(result->actuatedJoints.empty());
}

TEST(KinematicGeometry, FixedJointContributesTransformButNoAxis)
{
    ik::KinematicChain chain{"base", "tool",
        {makeJoint("fixed", ik::JointType::Fixed, {1, 0, 0}, {0, 0, std::numbers::pi / 2}, {}, 0),
         makeJoint("j", ik::JointType::Revolute, {0, 1, 0}, {}, {1, 0, 0}, 1)}};
    const auto result = ik::KinematicGeometryBuilder{}.build(chain);
    ASSERT_TRUE(result);
    ASSERT_EQ(result->actuatedJoints.size(), 1U);
    const auto& axis = result->actuatedJoints.front().axis;
    EXPECT_NEAR(axis.point.x, 0.0, 1e-12);
    EXPECT_NEAR(axis.point.y, 0.0, 1e-12);
    EXPECT_NEAR(axis.direction.x, 0.0, 1e-12);
    EXPECT_NEAR(axis.direction.y, 1.0, 1e-12);
}

TEST(KinematicGeometry, OriginRotationRotatesAxisIntoBaseFrame)
{
    ik::KinematicChain chain{"base", "tool",
        {makeJoint("j", ik::JointType::Revolute, {}, {0, 0, std::numbers::pi / 2}, {1, 0, 0}, 0)}};
    const auto result = ik::KinematicGeometryBuilder{}.build(chain);
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->actuatedJoints[0].axis.direction.x, 0.0, 1e-12);
    EXPECT_NEAR(result->actuatedJoints[0].axis.direction.y, 1.0, 1e-12);
}

TEST(KinematicGeometry, NormalizesHandBuiltActuatedAxis)
{
    ik::KinematicChain chain{"base", "tool",
        {makeJoint("j", ik::JointType::Prismatic, {}, {}, {0, 0, 4}, 0)}};
    const auto result = ik::KinematicGeometryBuilder{}.build(chain);
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->actuatedJoints[0].axis.direction.z, 1.0, 1e-12);
}

TEST(KinematicGeometry, RejectsZeroActuatedAxis)
{
    ik::KinematicChain chain{"base", "tool",
        {makeJoint("j", ik::JointType::Revolute, {}, {}, {}, 0)}};
    const auto result = ik::KinematicGeometryBuilder{}.build(chain);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, ik::KinematicGeometryErrorCode::InvalidActuatedAxis);
}

TEST(KinematicGeometry, RejectsNonFiniteOrigin)
{
    ik::KinematicChain chain{"base", "tool",
        {makeJoint("j", ik::JointType::Revolute,
                   {std::numeric_limits<double>::infinity(), 0, 0}, {}, {0, 0, 1}, 0)}};
    const auto result = ik::KinematicGeometryBuilder{}.build(chain);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, ik::KinematicGeometryErrorCode::NonFiniteOrigin);
}

TEST(KinematicGeometry, DetectsParallelPerpendicularAndCoincidentAxes)
{
    const ik::AxisLine x{{0, 0, 0}, {1, 0, 0}};
    const ik::AxisLine xSame{{2, 0, 0}, {-1, 0, 0}};
    const ik::AxisLine y{{0, 0, 0}, {0, 1, 0}};
    EXPECT_TRUE(ik::axesParallel(x, xSame));
    EXPECT_TRUE(ik::axesCoincident(x, xSame));
    EXPECT_TRUE(ik::axesPerpendicular(x, y));
}

TEST(KinematicGeometry, MeasuresSkewLineDistance)
{
    const ik::AxisLine x{{0, 0, 0}, {1, 0, 0}};
    const ik::AxisLine yAtZOne{{0, 0, 1}, {0, 1, 0}};
    EXPECT_NEAR(ik::axisDistance(x, yAtZOne), 1.0, 1e-12);
}

TEST(KinematicGeometry, FindsIntersectionPointForCrossingAxes)
{
    const ik::AxisLine x{{1, 2, 3}, {1, 0, 0}};
    const ik::AxisLine y{{4, 0, 3}, {0, 1, 0}};
    const auto point = ik::axisIntersectionPoint(x, y);
    ASSERT_TRUE(point);
    EXPECT_NEAR(point->x, 4.0, 1e-12);
    EXPECT_NEAR(point->y, 2.0, 1e-12);
    EXPECT_NEAR(point->z, 3.0, 1e-12);
}

TEST(KinematicGeometry, ParallelAxesHaveNoUniqueIntersectionPoint)
{
    const ik::AxisLine a{{0, 0, 0}, {1, 0, 0}};
    const ik::AxisLine b{{0, 1, 0}, {1, 0, 0}};
    EXPECT_FALSE(ik::axisIntersectionPoint(a, b));
}

TEST(KinematicGeometry, Kr4LastThreeAxesIntersectPairwise)
{
    const auto geometry = loadGeometry("kr4_r600.urdf");
    ASSERT_EQ(geometry.actuatedJoints.size(), 6U);
    for (std::size_t i = 3; i < 6; ++i)
        for (std::size_t j = i + 1; j < 6; ++j)
            EXPECT_LE(ik::axisDistance(geometry.actuatedJoints[i].axis,
                                       geometry.actuatedJoints[j].axis),
                      ik::kGeometryLinearTolerance);
}

TEST(KinematicGeometry, Kr640LastThreeAxesIntersectPairwise)
{
    const auto geometry = loadGeometry("kr640.urdf");
    ASSERT_EQ(geometry.actuatedJoints.size(), 6U);
    for (std::size_t i = 3; i < 6; ++i)
        for (std::size_t j = i + 1; j < 6; ++j)
            EXPECT_LE(ik::axisDistance(geometry.actuatedJoints[i].axis,
                                       geometry.actuatedJoints[j].axis),
                      ik::kGeometryLinearTolerance);
}
