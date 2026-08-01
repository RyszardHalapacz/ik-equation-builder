#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"

#include <filesystem>
#include <numbers>
#include "kinematics/robot_model_loader.hpp"   // dla testów kodu błędu

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

constexpr double kPi = std::numbers::pi;

// Writes a URDF to a unique temporary file and removes it on destruction.
// Keeping the malformed XML inline in each test is what makes those tests
// readable; a directory of invalid_axis_07.urdf files would not be.
class TemporaryUrdf
{
public:
    explicit TemporaryUrdf(std::string_view contents)
        : path_(std::filesystem::temp_directory_path() /
                ("kinemaforge_test_" + uniqueSuffix() + ".urdf"))
    {
        std::ofstream out(path_, std::ios::binary);
        if (!out)
            throw std::runtime_error("cannot create temporary URDF: " + path_.string());

        out << contents;
        out.close();
        if (!out)
            throw std::runtime_error("cannot write temporary URDF: " + path_.string());
    }

    ~TemporaryUrdf()
    {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    TemporaryUrdf(const TemporaryUrdf&) = delete;
    TemporaryUrdf& operator=(const TemporaryUrdf&) = delete;

    const std::filesystem::path& path() const noexcept { return path_; }

private:
    // gtest_discover_tests runs each test as its own process, and ctest may
    // run those in parallel. A per-process counter starting at zero would
    // make several processes fight over the same file — one deleting
    // another's input mid-test. The random salt is drawn once per process.
    static std::string uniqueSuffix()
    {
        static const std::uint64_t salt = [] {
            std::random_device device;
            return (static_cast<std::uint64_t>(device()) << 32) ^ device();
        }();
        static std::atomic<unsigned> counter{0};
        return std::to_string(salt) + "_" + std::to_string(counter++);
    }

    std::filesystem::path path_;
};

// Minimal two-link robot; the caller supplies the joint element.
std::string urdfWithJoint(std::string_view jointXml)
{
    return std::string{R"(<?xml version="1.0"?><robot name="t">)"}
         + R"(<link name="base"/><link name="tool"/>)"
         + std::string{jointXml}
         + "</robot>";
}

std::string revoluteJoint(std::string_view originXml, std::string_view axisXml)
{
    return std::string{R"(<joint name="j" type="revolute">)"}
         + R"(<parent link="base"/><child link="tool"/>)"
         + std::string{originXml} + std::string{axisXml}
         + R"(<limit lower="-1" upper="1" effort="1" velocity="1"/>)"
         + "</joint>";
}

// Loads a URDF given inline; returns the single joint, or throws.
kinemaforge::ik::UrdfJoint loadSingleJoint(std::string_view urdf)
{
    const TemporaryUrdf file{urdf};
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(file.path());
    return description.joints.at(0);
}

// Asserts on the actual reason, not merely "something threw". A bare
// EXPECT_THROW would go green for a file-not-found, a temp-file collision
// or a write failure just as happily as for the malformed input under test.
void expectRejectedWith(std::string_view urdf, mt::kinematics::LoadErrorCode expected)
{
    const TemporaryUrdf file{urdf};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, expected);
}

void expectMalformed(std::string_view urdf)
{
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::malformed_vector);
}

// The smallest positive double. Written as a literal on purpose:
// std::to_string uses %f with six decimals and would render it "0.000000",
// i.e. a zero axis — the test would then exercise degenerate-axis
// rejection instead of subnormal normalization.
constexpr std::string_view kDenormMin = "4.9406564584124654e-324";

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

TEST(UrdfModelLoaderTest, ParsesValidVector3)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 2 3"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 1.0);
    EXPECT_DOUBLE_EQ(joint.origin.translation.y, 2.0);
    EXPECT_DOUBLE_EQ(joint.origin.translation.z, 3.0);
}

TEST(UrdfModelLoaderTest, AcceptsLeadingAndTrailingWhitespace)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="  1 2 3  "/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 1.0);
    EXPECT_DOUBLE_EQ(joint.origin.translation.z, 3.0);
}

TEST(UrdfModelLoaderTest, AcceptsCharacterReferenceTabSeparators)
{
    // A literal tab would be normalized to a space by XML itself, so this
    // test would pass on the old parser too and guard nothing. A character
    // reference survives normalization and reaches the parser as a tab.
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1&#x9;2&#x9;3"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.y, 2.0);
}

TEST(UrdfModelLoaderTest, AcceptsLeadingPlusSign)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="+1 +2 +3"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 1.0);
}

TEST(UrdfModelLoaderTest, AcceptsScientificNotation)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1.5e2 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 150.0);
}

TEST(UrdfModelLoaderTest, AcceptsNegativeZero)
{
    // Accepting the spelling is the contract; the sign bit is not, since
    // -0.0 == 0.0 and later stages must not depend on it.
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="-0 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 0.0);
}

TEST(UrdfModelLoaderTest, RejectsMalformedVectorText)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 abc 3"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsMissingVectorComponent)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 2"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsExtraVectorComponent)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 2 3 4"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsTrailingGarbage)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 2 3 abc"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsEmptyVectorAttribute)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz=""/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsOverflowingVectorComponent)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1e400 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsNaNVectorComponent)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="nan 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsPositiveInfinityVectorComponent)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="inf 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsNegativeInfinityVectorComponent)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="-inf 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsMultipleLeadingSigns)
{
    // Naively skipping '+' before from_chars would let this through as -1;
    // it is the only sign combination that slips past that mistake.
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="+-1 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));
}
TEST(UrdfModelLoaderTest, DefaultsMissingOriginToZero)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(joint.origin.rpy.z, 0.0);
}

TEST(UrdfModelLoaderTest, DefaultsMissingOriginXyzToZero)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin rpy="0 0 1"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(joint.origin.rpy.z, 1.0);
}

TEST(UrdfModelLoaderTest, DefaultsMissingOriginRpyToZero)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 1.0);
    EXPECT_DOUBLE_EQ(joint.origin.rpy.x, 0.0);
}

TEST(UrdfModelLoaderTest, KeepsRpyInRadians)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin rpy="3.14159 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.rpy.x, 3.14159);
}
TEST(UrdfModelLoaderTest, RejectsMalformedOriginRpy)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin rpy="1 abc 3"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsMalformedAxis)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="0 0 0"/>)", R"(<axis xyz="1 abc 3"/>)")));
}
TEST(UrdfModelLoaderTest, DefaultsMissingAxisToXForActuatedJoint)
{
    const auto joint = loadSingleJoint(urdfWithJoint(revoluteJoint("", "")));

    EXPECT_DOUBLE_EQ(joint.axis.x, 1.0);
    EXPECT_DOUBLE_EQ(joint.axis.y, 0.0);
    EXPECT_DOUBLE_EQ(joint.axis.z, 0.0);
}

TEST(UrdfModelLoaderTest, DefaultsEmptyAxisElementToXForActuatedJoint)
{
    // <axis/> is a missing attribute, not a bad value: same default as no
    // element at all.
    const auto joint = loadSingleJoint(urdfWithJoint(revoluteJoint("", "<axis/>")));

    EXPECT_DOUBLE_EQ(joint.axis.x, 1.0);
}

TEST(UrdfModelLoaderTest, RejectsEmptyAxisAttribute)
{
    // Present but empty is malformed, unlike a missing attribute.
    expectMalformed(urdfWithJoint(revoluteJoint("", R"(<axis xyz=""/>)")));
}

TEST(UrdfModelLoaderTest, RejectsZeroAxisForRevoluteJoint)
{
    expectRejectedWith(urdfWithJoint(revoluteJoint("", R"(<axis xyz="0 0 0"/>)")),
                       mt::kinematics::LoadErrorCode::degenerate_axis);
}

TEST(UrdfModelLoaderTest, RejectsZeroAxisForPrismaticJoint)
{
    const std::string joint =
        std::string{R"(<joint name="j" type="prismatic">)"}
        + R"(<parent link="base"/><child link="tool"/>)"
        + R"(<axis xyz="0 0 0"/>)"
        + R"(<limit lower="-1" upper="1" effort="1" velocity="1"/>)"
        + "</joint>";
    expectRejectedWith(urdfWithJoint(joint),
                       mt::kinematics::LoadErrorCode::degenerate_axis);
}

TEST(UrdfModelLoaderTest, IgnoresZeroAxisForFixedJoint)
{
    // A fixed joint's axis takes no part in the kinematics, so a degenerate
    // value is not an error.
    //
    // The chain needs an actuated joint as well: trace_chain skips fixed
    // joints and rejects an empty chain, so a robot made of one fixed joint
    // would fail with incomplete_kinematic_chain and never reach the axis
    // check this test is about.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="middle"/><link name="tool"/>)"
        R"(<joint name="fixed_j" type="fixed">)"
        R"(<parent link="base"/><child link="middle"/>)"
        R"(<axis xyz="0 0 0"/>)"
        R"(</joint>)"
        R"(<joint name="active_j" type="revolute">)"
        R"(<parent link="middle"/><child link="tool"/>)"
        R"(<axis xyz="0 0 1"/>)"
        R"(<limit lower="-1" upper="1" effort="1" velocity="1"/>)"
        R"(</joint></robot>)";

    const TemporaryUrdf file{urdf};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->model.joints().size(), 2u);

    // The degenerate axis is kept verbatim, not normalized or rejected.
    const auto& fixed = result->model.joints()[0];
    ASSERT_EQ(fixed.type, mt::kinematics::JointType::fixed);
    EXPECT_DOUBLE_EQ(fixed.axis.x, 0.0);
    EXPECT_DOUBLE_EQ(fixed.axis.y, 0.0);
    EXPECT_DOUBLE_EQ(fixed.axis.z, 0.0);
}

TEST(UrdfModelLoaderTest, RejectsMalformedAxisForFixedJoint)
{
    // Semantics are skipped for fixed joints; syntax never is.
    const std::string joint =
        std::string{R"(<joint name="j" type="fixed">)"}
        + R"(<parent link="base"/><child link="tool"/>)"
        + R"(<axis xyz="1 abc 3"/>)"
        + "</joint>";
    expectMalformed(urdfWithJoint(joint));
}
TEST(UrdfModelLoaderTest, NormalizesNonUnitXAxis)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="2 0 0"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.x, 1.0);
    EXPECT_DOUBLE_EQ(joint.axis.y, 0.0);
}

TEST(UrdfModelLoaderTest, NormalizesNonUnitYAxis)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="0 3 0"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.y, 1.0);
}

TEST(UrdfModelLoaderTest, NormalizesNonUnitZAxis)
{
    // Axis-aligned input normalizes exactly: hypot returns |component| and
    // dividing a value by itself gives 1.0 bit for bit.
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="0 0 5"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.z, 1.0);
    EXPECT_DOUBLE_EQ(joint.axis.x, 0.0);
}

TEST(UrdfModelLoaderTest, NormalizesNegativeAxis)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="0 0 -5"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.z, -1.0);
}

TEST(UrdfModelLoaderTest, KeepsAlreadyUnitAxisBitExact)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.z, 1.0);
}

TEST(UrdfModelLoaderTest, NormalizesArbitraryAxis)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="1 2 3"/>)")));

    // All three components, not just the length: a wrong direction of the
    // right length would otherwise pass.
    const double scale = 1.0 / std::sqrt(14.0);
    EXPECT_NEAR(joint.axis.x, 1.0 * scale, 1e-15);
    EXPECT_NEAR(joint.axis.y, 2.0 * scale, 1e-15);
    EXPECT_NEAR(joint.axis.z, 3.0 * scale, 1e-15);
    EXPECT_NEAR(std::hypot(joint.axis.x, joint.axis.y, joint.axis.z), 1.0, 1e-15);
}

TEST(UrdfModelLoaderTest, NormalizesVeryLargeAxis)
{
    // sqrt(x*x + ...) would overflow to inf here and yield a zero axis.
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="1e200 0 0"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.x, 1.0);
}

TEST(UrdfModelLoaderTest, NormalizesVerySmallAxis)
{
    // sqrt(x*x + ...) would underflow to zero here and divide by it.
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="1e-200 0 0"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.x, 1.0);
}

TEST(UrdfModelLoaderTest, NormalizesMultiComponentSubnormalAxis)
{
    // std::hypot alone returns denorm_min here, giving [1, 1, 0] — length
    // sqrt(2). Only scaling by the largest component first survives this.
    // A single-component test cannot catch it: that value divides by itself.
    const std::string axis = std::string{R"(<axis xyz=")"}
        + std::string{kDenormMin} + " " + std::string{kDenormMin} + R"( 0"/>)";

    const auto joint = loadSingleJoint(urdfWithJoint(revoluteJoint("", axis)));

    const double expected = 1.0 / std::sqrt(2.0);
    EXPECT_NEAR(joint.axis.x, expected, 1e-15);
    EXPECT_NEAR(joint.axis.y, expected, 1e-15);
    EXPECT_DOUBLE_EQ(joint.axis.z, 0.0);
    EXPECT_NEAR(std::hypot(joint.axis.x, joint.axis.y, joint.axis.z), 1.0, 1e-15);
}

TEST(UrdfModelLoaderTest, NormalizesFullySubnormalAxis)
{
    const std::string value{kDenormMin};
    const std::string axis = std::string{R"(<axis xyz=")"}
        + value + " " + value + " " + value + R"("/>)";

    const auto joint = loadSingleJoint(urdfWithJoint(revoluteJoint("", axis)));

    // Length alone would pass for any unit vector; the direction matters.
    const double expected = 1.0 / std::sqrt(3.0);
    EXPECT_NEAR(joint.axis.x, expected, 1e-15);
    EXPECT_NEAR(joint.axis.y, expected, 1e-15);
    EXPECT_NEAR(joint.axis.z, expected, 1e-15);
}
TEST(UrdfModelLoaderTest, KeepsKr640GeometryValid)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(urdfPath("kr640.urdf"));

    // Expected axes per joint, not just unit length: a loader that turned
    // every axis into [1,0,0] would pass a length-only check.
    struct Expected { const char* name; double x, y, z; };
    const Expected expected[] = {
        {"joint_a1", 0.0, 0.0, 1.0},
        {"joint_a2", 0.0, 1.0, 0.0},
        {"joint_a3", 0.0, 1.0, 0.0},
        {"joint_a4", 1.0, 0.0, 0.0},
        {"joint_a5", 0.0, 1.0, 0.0},
        {"joint_a6", 1.0, 0.0, 0.0},
    };

    for (const auto& e : expected)
    {
        SCOPED_TRACE(e.name);
        const auto it = std::ranges::find(description.joints, e.name,
                                          &kinemaforge::ik::UrdfJoint::name);
        ASSERT_NE(it, description.joints.end());
        EXPECT_DOUBLE_EQ(it->axis.x, e.x);
        EXPECT_DOUBLE_EQ(it->axis.y, e.y);
        EXPECT_DOUBLE_EQ(it->axis.z, e.z);
    }
}

TEST(UrdfModelLoaderTest, KeepsKr4GeometryValid)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(urdfPath("kr4_r600.urdf"));

    // Every actuated joint in this robot spins about local Z.
    for (const auto& joint : description.joints)
    {
        if (joint.type == kinemaforge::ik::JointType::Fixed)
            continue;
        SCOPED_TRACE(joint.name);
        EXPECT_DOUBLE_EQ(joint.axis.x, 0.0);
        EXPECT_DOUBLE_EQ(joint.axis.y, 0.0);
        EXPECT_DOUBLE_EQ(joint.axis.z, 1.0);
    }
}

TEST(UrdfModelLoaderTest, ReturnsMalformedVectorContext)
{
    const TemporaryUrdf file{urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 abc 3"/>)", R"(<axis xyz="0 0 1"/>)"))};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, mt::kinematics::LoadErrorCode::malformed_vector);
    EXPECT_EQ(result.error().jointName, "j");
    EXPECT_EQ(result.error().attribute, "origin/xyz");
    ASSERT_TRUE(result.error().rawValue.has_value());
    EXPECT_EQ(*result.error().rawValue, "1 abc 3");
}

TEST(UrdfModelLoaderTest, ReturnsEmptyRawValueAsPresentButEmpty)
{
    // xyz="" is a reportable value, not an absent one — the optional must
    // be engaged so the message can quote it.
    const TemporaryUrdf file{urdfWithJoint(
        revoluteJoint(R"(<origin xyz=""/>)", R"(<axis xyz="0 0 1"/>)"))};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_FALSE(result.has_value());
    ASSERT_TRUE(result.error().rawValue.has_value());
    EXPECT_TRUE(result.error().rawValue->empty());
}

TEST(UrdfModelLoaderTest, ReturnsDegenerateAxisContext)
{
    const TemporaryUrdf file{urdfWithJoint(revoluteJoint("", R"(<axis xyz="0 0 0"/>)"))};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, mt::kinematics::LoadErrorCode::degenerate_axis);
    EXPECT_EQ(result.error().jointName, "j");
    EXPECT_EQ(result.error().attribute, "axis/xyz");
    ASSERT_TRUE(result.error().rawValue.has_value());
    EXPECT_EQ(*result.error().rawValue, "0 0 0");
}

TEST(UrdfModelLoaderTest, TruncatesOverlongRawValue)
{
    const std::string overlong(400, '9');
    const std::string origin = R"(<origin xyz=")" + overlong + R"("/>)";
    const TemporaryUrdf file{urdfWithJoint(revoluteJoint(origin, R"(<axis xyz="0 0 1"/>)"))};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_FALSE(result.has_value());
    ASSERT_TRUE(result.error().rawValue.has_value());
    EXPECT_EQ(result.error().rawValue->size(), 256u + 3u);   // 256 znaków + "..."
    EXPECT_TRUE(result.error().rawValue->ends_with("..."));
}

TEST(UrdfModelLoaderTest, IncludesLoadErrorContextInException)
{
    // The ik layer keeps throwing, but the message must now carry the
    // context the lower layer collected.
    const TemporaryUrdf file{urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 abc 3"/>)", R"(<axis xyz="0 0 1"/>)"))};
    kinemaforge::ik::UrdfModelLoader loader;

    try
    {
        loader.load(file.path());
        FAIL() << "expected UrdfModelLoader::load to throw";
    }
    catch (const std::runtime_error& error)
    {
        const std::string message = error.what();
        EXPECT_NE(message.find("joint 'j'"), std::string::npos) << message;
        EXPECT_NE(message.find("origin/xyz"), std::string::npos) << message;
        EXPECT_NE(message.find("1 abc 3"), std::string::npos) << message;
    }
}

TEST(UrdfModelLoaderTest, ReportsContinuousAsUnsupportedUntilImplemented)
{
    // TODO: remove together with the proposal adding Continuous support.
    // This pins a known gap as deliberate, so it cannot quietly be taken
    // for intended behaviour.
    const std::string joint =
        std::string{R"(<joint name="j" type="continuous">)"}
        + R"(<parent link="base"/><child link="tool"/>)"
        + R"(<axis xyz="0 0 1"/><limit effort="1" velocity="1"/>)"
        + "</joint>";
    expectRejectedWith(urdfWithJoint(joint),
                       mt::kinematics::LoadErrorCode::unsupported_joint_type);
}
