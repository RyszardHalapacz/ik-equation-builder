#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"

#include <filesystem>

namespace {

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

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
