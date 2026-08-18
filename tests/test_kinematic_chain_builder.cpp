#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"

#include <filesystem>
#include <string>

namespace {

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

} // namespace

using kinemaforge::ik::JointType;
using kinemaforge::ik::KinematicChainBuilder;
using kinemaforge::ik::KinematicChainError;
using kinemaforge::ik::RobotDescription;
using kinemaforge::ik::UrdfJoint;
using kinemaforge::ik::UrdfLink;
using kinemaforge::ik::UrdfModelLoader;

TEST(KinematicChainBuilderTest, BuildsKr640BaseToTool0Chain)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->joints.size(), 7u);
}

TEST(KinematicChainBuilderTest, PreservesJointOrder)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->joints.size(), 7u);

    const char* expectedNames[] = {
        "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a6_to_tool0",
    };
    for (std::size_t i = 0; i < result->joints.size(); ++i)
    {
        SCOPED_TRACE(testing::Message() << "joint index " << i);
        EXPECT_EQ(result->joints[i].name, expectedNames[i]);
    }
}

TEST(KinematicChainBuilderTest, KeepsFixedJoints)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());

    const auto& fixedJoint = result->joints.back();
    EXPECT_EQ(fixedJoint.name, "joint_a6_to_tool0");
    EXPECT_EQ(fixedJoint.type, JointType::Fixed);
}

TEST(KinematicChainBuilderTest, AssignsSymbolsOnlyToActuatedJoints)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->joints.size(), 7u);

    for (std::size_t i = 0; i < 6; ++i)
    {
        SCOPED_TRACE(testing::Message() << "actuated joint index " << i);
        EXPECT_TRUE(result->joints[i].variable.has_value());
    }
    EXPECT_FALSE(result->joints.back().variable.has_value());
}

TEST(KinematicChainBuilderTest, NumbersSymbolsFromQ1)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());

    for (std::size_t i = 0; i < 6; ++i)
    {
        SCOPED_TRACE(testing::Message() << "actuated joint index " << i);
        ASSERT_TRUE(result->joints[i].variable.has_value());
        EXPECT_EQ(result->joints[i].variable->name, "q" + std::to_string(i + 1));
        EXPECT_EQ(result->joints[i].variable->index, i + 1);
    }
}

TEST(KinematicChainBuilderTest, BuildsKr4BaseToTool0Chain)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr4_r600.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");

    ASSERT_TRUE(result.has_value());
    // joint_1..joint_6 (6 actuated) + link6-tool0 (fixed). base_link's other
    // fixed joint (base_link-base -> "base") is a dead-end branch, correctly
    // excluded — mirrors the link_6 -> flange/tool0 branch at the other end.
    EXPECT_EQ(result->joints.size(), 7u);
}

TEST(KinematicChainBuilderTest, PicksCorrectBranchAtLinkSix)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr4_r600.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());

    for (auto const& joint : result->joints)
        EXPECT_NE(joint.name, "link6-flange");

    EXPECT_EQ(result->joints.back().name, "link6-tool0");
}

TEST(KinematicChainBuilderTest, ChainRecordsRequestedBaseAndToolLinkNames)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());

    EXPECT_EQ(result->baseLink, "base_link");
    EXPECT_EQ(result->toolLink, "tool0");
}

TEST(KinematicChainBuilderTest, JointIndexMatchesPositionInChain)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());

    for (std::size_t i = 0; i < result->joints.size(); ++i)
        EXPECT_EQ(result->joints[i].index, i);
}

TEST(KinematicChainBuilderTest, PreservesOriginAndAxisForFixedJoints)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(result.has_value());

    const auto& fixedJoint = result->joints.back();
    ASSERT_EQ(fixedJoint.name, "joint_a6_to_tool0");
    EXPECT_DOUBLE_EQ(fixedJoint.origin.translation.z, 0.290);
}

TEST(KinematicChainBuilderTest, ReturnsBaseLinkNotFoundWhenBaseLinkDoesNotExist)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "does_not_exist", "tool0");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), KinematicChainError::BaseLinkNotFound);
}

TEST(KinematicChainBuilderTest, ReturnsToolLinkNotFoundWhenToolLinkDoesNotExist)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "does_not_exist");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), KinematicChainError::ToolLinkNotFound);
}

TEST(KinematicChainBuilderTest, ReturnsNoPathFoundWhenNoPathExists)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr4_r600.urdf"));

    KinematicChainBuilder builder;
    // "flange" exists but is a dead-end branch off link_6; tool0 hangs off
    // the other branch, so there is no downward path from flange to tool0.
    const auto result = builder.build(robot, "flange", "tool0");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), KinematicChainError::NoPathFound);
}

TEST(KinematicChainBuilderTest, ReturnsEmptyChainWhenBaseEqualsTool)
{
    UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath("kr640.urdf"));

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base_link", "base_link");

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->baseLink, "base_link");
    EXPECT_EQ(result->toolLink, "base_link");
    EXPECT_TRUE(result->joints.empty());
}

TEST(KinematicChainBuilderTest, ReturnsInvalidRobotDescriptionOnCyclicInput)
{
    RobotDescription robot;
    robot.name = "cyclic";
    robot.links = {UrdfLink{"A"}, UrdfLink{"B"}, UrdfLink{"C"}};

    UrdfJoint j1;
    j1.name = "j1";
    j1.parentLink = "A";
    j1.childLink = "B";
    j1.type = JointType::Revolute;

    UrdfJoint j2;
    j2.name = "j2";
    j2.parentLink = "B";
    j2.childLink = "A";
    j2.type = JointType::Revolute;

    robot.joints = {j1, j2};

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "A", "C");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), KinematicChainError::InvalidRobotDescription);
}

TEST(KinematicChainBuilderTest, ReturnsInvalidRobotDescriptionOnDuplicateChildLink)
{
    RobotDescription robot;
    robot.name = "duplicate_parent";
    robot.links = {UrdfLink{"base"}, UrdfLink{"mid"}, UrdfLink{"tool"}};

    UrdfJoint j1;
    j1.name = "j1";
    j1.parentLink = "base";
    j1.childLink = "mid";
    j1.type = JointType::Fixed;

    UrdfJoint j2;
    j2.name = "j2";
    j2.parentLink = "tool";
    j2.childLink = "mid"; // "mid" already claimed as a child by j1
    j2.type = JointType::Fixed;

    robot.joints = {j1, j2};

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base", "mid");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), KinematicChainError::InvalidRobotDescription);
}

TEST(KinematicChainBuilderTest, AssignsVariableToContinuousJoint)
{
    // The loader doesn't parse type="continuous" yet (see
    // proposal-loader-test-coverage.md, "Znane luki"), so this is
    // constructed directly — regression guard for the sekcja 7/14
    // decision that Continuous counts as actuated.
    RobotDescription robot;
    robot.name = "continuous_test";
    robot.links = {UrdfLink{"base"}, UrdfLink{"tool"}};

    UrdfJoint joint;
    joint.name = "spin";
    joint.parentLink = "base";
    joint.childLink = "tool";
    joint.type = JointType::Continuous;

    robot.joints = {joint};

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base", "tool");

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->joints.size(), 1u);
    ASSERT_TRUE(result->joints[0].variable.has_value());
    EXPECT_EQ(result->joints[0].variable->name, "q1");
    EXPECT_EQ(result->joints[0].variable->index, 1u);
}

// ---------------------------------------------------------------------------
// Declared links are the only source of link existence (stage F2.3.R, R2)
//
// build() used to union robot.links with every joint parent/child reference,
// because the loader did not cross-check the two. That made the two errors below
// unreachable for any name mentioned in any joint: a link that exists only as a
// typo inside a <parent> element became a usable frame, and a transform could be
// built relative to a frame the URDF never declared.
// ---------------------------------------------------------------------------

TEST(KinematicChainBuilderTest, RejectsBaseLinkNotDeclaredAsLink)
{
    RobotDescription robot;
    robot.links = {UrdfLink{"declared"}};

    UrdfJoint joint;
    joint.name = "j";
    joint.parentLink = "phantom";   // referenced, never declared
    joint.childLink = "declared";
    joint.type = JointType::Fixed;
    robot.joints = {joint};

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "phantom", "declared");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), KinematicChainError::BaseLinkNotFound);
}

TEST(KinematicChainBuilderTest, RejectsToolLinkNotDeclaredAsLink)
{
    RobotDescription robot;
    robot.links = {UrdfLink{"base"}};

    UrdfJoint joint;
    joint.name = "j";
    joint.parentLink = "base";
    joint.childLink = "phantom";    // referenced, never declared
    joint.type = JointType::Revolute;
    robot.joints = {joint};

    KinematicChainBuilder builder;
    const auto result = builder.build(robot, "base", "phantom");

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), KinematicChainError::ToolLinkNotFound);
}
