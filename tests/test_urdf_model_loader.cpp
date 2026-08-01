#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"

#include <filesystem>
#include <numbers>

namespace {

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

constexpr double kPi = std::numbers::pi;

} // namespace

TEST(UrdfModelLoaderTest, LoadsKr640LinksAndJoints)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(urdfPath("kr640.urdf"));

    EXPECT_EQ(description.name, "kuka_kr640_r2800_2");
    EXPECT_EQ(description.links.size(), 8u);
    ASSERT_EQ(description.joints.size(), 7u);
}

TEST(UrdfModelLoaderTest, MapsRevoluteJointFields)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(urdfPath("kr640.urdf"));

    const auto& joint = description.joints.at(0);
    EXPECT_EQ(joint.name, "joint_a1");
    EXPECT_EQ(joint.parentLink, "base_link");
    EXPECT_EQ(joint.childLink, "link_1");
    EXPECT_EQ(joint.type, kinemaforge::ik::JointType::Revolute);

    EXPECT_DOUBLE_EQ(joint.origin.translation.z, 0.750);
    EXPECT_DOUBLE_EQ(joint.axis.z, 1.0);

    EXPECT_TRUE(joint.limits.hasPositionLimits);
    EXPECT_DOUBLE_EQ(joint.limits.lower, -3.2288);
    EXPECT_DOUBLE_EQ(joint.limits.upper, 3.2288);
}

TEST(UrdfModelLoaderTest, MapsFixedJointWithoutPositionLimits)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(urdfPath("kr640.urdf"));

    const auto& fixedJoint = description.joints.back();
    EXPECT_EQ(fixedJoint.name, "joint_a6_to_tool0");
    EXPECT_EQ(fixedJoint.type, kinemaforge::ik::JointType::Fixed);
    EXPECT_FALSE(fixedJoint.limits.hasPositionLimits);
}

TEST(UrdfModelLoaderTest, ThrowsOnMissingFile)
{
    kinemaforge::ik::UrdfModelLoader loader;
    EXPECT_THROW(loader.load(urdfPath("does_not_exist.urdf")), std::runtime_error);
}

TEST(UrdfModelLoaderTest, MapsAllKr640JointTypes)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(urdfPath("kr640.urdf"));
    ASSERT_EQ(description.joints.size(), 7u);

    struct Expected
    {
        const char* name;
        kinemaforge::ik::JointType type;
    };
    const Expected expected[] = {
        {"joint_a1", kinemaforge::ik::JointType::Revolute},
        {"joint_a2", kinemaforge::ik::JointType::Revolute},
        {"joint_a3", kinemaforge::ik::JointType::Revolute},
        {"joint_a4", kinemaforge::ik::JointType::Revolute},
        {"joint_a5", kinemaforge::ik::JointType::Revolute},
        {"joint_a6", kinemaforge::ik::JointType::Revolute},
        {"joint_a6_to_tool0", kinemaforge::ik::JointType::Fixed},
    };

    for (std::size_t i = 0; i < description.joints.size(); ++i)
    {
        SCOPED_TRACE(testing::Message() << "joint index " << i);
        EXPECT_EQ(description.joints[i].name, expected[i].name);
        EXPECT_EQ(description.joints[i].type, expected[i].type);
    }
}

TEST(UrdfModelLoaderTest, MapsKr4VelocityAndEffortLimits)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(urdfPath("kr4_r600.urdf"));
    ASSERT_EQ(description.joints.size(), 9u);

    struct Expected
    {
        const char* name;
        double velocity;
        double effort;
    };
    const Expected expected[] = {
        {"joint_1", 5.8643062867, 119.016},
        {"joint_2", 5.8643062867, 105.851},
        {"joint_3", 8.5084763635, 54.315},
        {"joint_4", 10.471962422, 11.812},
        {"joint_5", 9.2345347637, 12.328},
        {"joint_6", 13.9626340160, 6.916},
    };

    for (std::size_t i = 0; i < std::size(expected); ++i)
    {
        const auto& joint = description.joints.at(i + 1); // index 0 is the fixed base_link-base joint
        SCOPED_TRACE(testing::Message() << "joint " << expected[i].name);
        EXPECT_EQ(joint.name, expected[i].name);
        EXPECT_DOUBLE_EQ(joint.limits.velocity, expected[i].velocity);
        EXPECT_DOUBLE_EQ(joint.limits.effort, expected[i].effort);
    }
}

TEST(UrdfModelLoaderTest, LoadsKr4LinksAndJoints)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(urdfPath("kr4_r600.urdf"));

    EXPECT_EQ(description.name, "kuka_kr4_r600");
    EXPECT_EQ(description.links.size(), 10u);
    ASSERT_EQ(description.joints.size(), 9u);
}

TEST(UrdfModelLoaderTest, MapsKr4JointOriginRotation)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(urdfPath("kr4_r600.urdf"));

    // joint_4 has a non-zero rpy on two axes at once (x and z) — the
    // sturdiest case for catching an accidental sign/axis swap.
    const auto& joint = description.joints.at(4);
    ASSERT_EQ(joint.name, "joint_4");

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 0.1694);
    EXPECT_DOUBLE_EQ(joint.origin.translation.y, -0.02);
    EXPECT_DOUBLE_EQ(joint.origin.translation.z, -0.059);

    EXPECT_DOUBLE_EQ(joint.origin.rpy.x, kPi / 2.0);
    EXPECT_DOUBLE_EQ(joint.origin.rpy.y, 0.0);
    EXPECT_DOUBLE_EQ(joint.origin.rpy.z, -kPi / 2.0);
}

TEST(UrdfModelLoaderTest, MapsKr4JointAxis)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(urdfPath("kr4_r600.urdf"));

    // Same joint as MapsKr4JointOriginRotation: origin.rpy != 0 but axis
    // must still come through as the raw local Z from the URDF, not a
    // vector rotated by origin.rpy into some other frame.
    const auto& joint = description.joints.at(4);
    ASSERT_EQ(joint.name, "joint_4");

    EXPECT_DOUBLE_EQ(joint.axis.x, 0.0);
    EXPECT_DOUBLE_EQ(joint.axis.y, 0.0);
    EXPECT_DOUBLE_EQ(joint.axis.z, 1.0);
}
