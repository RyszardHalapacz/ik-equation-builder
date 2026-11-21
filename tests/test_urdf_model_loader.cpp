#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"

#include <filesystem>
#include <numbers>
#include "legacy/kinematics/robot_model_loader.hpp"   // dla testów kodu błędu

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
    // The second joint is now only scenery. It used to be required: trace_chain
    // skipped fixed joints and rejected an empty chain, so a robot made of one
    // fixed joint failed with incomplete_kinematic_chain before ever reaching
    // the axis check this test is about. Since R2 the loader no longer traces a
    // chain, a single fixed joint is a valid two-node tree, and the extra joint
    // is kept only to avoid rewriting a test that already passes.
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

// --- Continuous joints (follow-up 2026-11-16 #3) ----------------------------
//
// URDF: a continuous joint is a revolute joint without position limits. Its
// <limit> is optional; when present, this project's strict profile still
// requires velocity and effort, and lower/upper carry no meaning and are
// ignored.

namespace {
std::string continuousJoint(std::string_view limitXml)
{
    return std::string{R"(<joint name="j" type="continuous">)"}
         + R"(<parent link="base"/><child link="tool"/>)"
         + R"(<axis xyz="0 0 1"/>)" + std::string{limitXml} + "</joint>";
}
} // namespace

TEST(UrdfModelLoaderTest, LoadsContinuousJointWithoutPositionLimits)
{
    const TemporaryUrdf file{urdfWithJoint(continuousJoint(R"(<limit effort="1" velocity="2"/>)"))};
    const auto loaded = mt::kinematics::load_urdf(file.path());
    ASSERT_TRUE(loaded.has_value());
    ASSERT_EQ(loaded->model.joints().size(), 1U);
    EXPECT_EQ(loaded->model.joints()[0].type, mt::kinematics::JointType::continuous);
    EXPECT_EQ(loaded->model.joints()[0].limits.velocity, 2.0);

    const auto description = kinemaforge::ik::UrdfModelLoader{}.load(file.path());
    ASSERT_EQ(description.joints.size(), 1U);
    EXPECT_EQ(description.joints[0].type, kinemaforge::ik::JointType::Continuous);
    EXPECT_FALSE(description.joints[0].limits.hasPositionLimits);
    EXPECT_EQ(description.joints[0].limits.velocity, 2.0);
}

TEST(UrdfModelLoaderTest, ContinuousJointNeedsNoLimitElement)
{
    const TemporaryUrdf file{urdfWithJoint(continuousJoint(""))};
    const auto description = kinemaforge::ik::UrdfModelLoader{}.load(file.path());
    ASSERT_EQ(description.joints.size(), 1U);
    EXPECT_EQ(description.joints[0].type, kinemaforge::ik::JointType::Continuous);
    EXPECT_FALSE(description.joints[0].limits.hasPositionLimits);
}

TEST(UrdfModelLoaderTest, ContinuousJointIgnoresLowerAndUpper)
{
    const TemporaryUrdf file{urdfWithJoint(
        continuousJoint(R"(<limit lower="-1" upper="1" effort="1" velocity="1"/>)"))};
    const auto description = kinemaforge::ik::UrdfModelLoader{}.load(file.path());
    EXPECT_FALSE(description.joints.at(0).limits.hasPositionLimits);
}

TEST(UrdfModelLoaderTest, ContinuousJointLimitStillFollowsTheStrictProfile)
{
    expectRejectedWith(urdfWithJoint(continuousJoint(R"(<limit effort="1"/>)")),
                       mt::kinematics::LoadErrorCode::missing_limit_attribute);
    expectRejectedWith(urdfWithJoint(continuousJoint(R"(<limit effort="1" velocity="-1"/>)")),
                       mt::kinematics::LoadErrorCode::invalid_limit_constraint);
    expectRejectedWith(urdfWithJoint(continuousJoint(
                           R"(<limit effort="1" velocity="1"/><limit effort="1" velocity="1"/>)")),
                       mt::kinematics::LoadErrorCode::duplicate_limit_element);
}

TEST(UrdfModelLoaderTest, ContinuousJointStillNeedsANonZeroAxis)
{
    const std::string joint = std::string{R"(<joint name="j" type="continuous">)"}
                            + R"(<parent link="base"/><child link="tool"/>)"
                            + R"(<axis xyz="0 0 0"/></joint>)";
    expectRejectedWith(urdfWithJoint(joint), mt::kinematics::LoadErrorCode::degenerate_axis);
}

TEST(UrdfModelLoaderTest, Kr640WithContinuousWristLoadsIntoAChainTheSolverTreatsAsUnbounded)
{
    // End to end: the same robot with joint_a6 continuous.
    std::ifstream in(urdfPath("kr640.urdf"));
    std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    const std::string from = R"(<joint name="joint_a6" type="revolute">)";
    const auto at = text.find(from);
    ASSERT_NE(at, std::string::npos);
    text.replace(at, from.size(), R"(<joint name="joint_a6" type="continuous">)");

    const TemporaryUrdf file{text};
    const auto robot = kinemaforge::ik::UrdfModelLoader{}.load(file.path());
    const auto chain = kinemaforge::ik::KinematicChainBuilder{}.build(robot, "base_link", "tool0");
    ASSERT_TRUE(chain);
    const auto wrist = std::find_if(chain->joints.begin(), chain->joints.end(),
                                    [](const auto& joint) { return joint.name == "joint_a6"; });
    ASSERT_NE(wrist, chain->joints.end());
    EXPECT_EQ(wrist->type, kinemaforge::ik::JointType::Continuous);
    EXPECT_FALSE(wrist->limits.hasPositionLimits);
}

// ---------------------------------------------------------------------------
// Topology (stage F2.3.R, R2)
//
// load_urdf used to trace a single linear chain, which assumed one parent has
// one child. URDF describes a tree, and both shipped robots branch. These tests
// pin the replacement: the loader validates the whole tree and selects nothing.
// ---------------------------------------------------------------------------

namespace {

// A branching robot shaped like the real KR4 around base_link: one fixed branch
// and one actuated branch sharing a parent. `fixedFirst` swaps the declaration
// order of those two joints and changes nothing else.
std::string branchingRobot(bool fixedFirst)
{
    const std::string fixedJoint =
        R"(<joint name="side" type="fixed">)"
        R"(<parent link="base"/><child link="pedestal"/>)"
        R"(<origin xyz="0 0 0" rpy="0 0 0"/>)"
        R"(</joint>)";
    const std::string actuatedJoint =
        R"(<joint name="j1" type="revolute">)"
        R"(<parent link="base"/><child link="tool"/>)"
        R"(<origin xyz="0 0 0.5" rpy="0 0 0"/><axis xyz="0 0 1"/>)"
        R"(<limit lower="-1" upper="1" effort="1" velocity="1"/>)"
        R"(</joint>)";

    return std::string{R"(<?xml version="1.0"?><robot name="branch">)"}
         + R"(<link name="base"/><link name="pedestal"/><link name="tool"/>)"
         + (fixedFirst ? fixedJoint + actuatedJoint : actuatedJoint + fixedJoint)
         + "</robot>";
}

} // namespace

TEST(UrdfModelLoaderTest, AcceptsBranchingTree)
{
    // One link parenting two joints is a valid URDF tree, not an error. The
    // previous implementation silently kept only one of the two branches.
    const TemporaryUrdf file{branchingRobot(true)};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->model.links().size(), 3u);
    EXPECT_EQ(result->model.joints().size(), 2u);
}

TEST(UrdfModelLoaderTest, LoadsEquivalentModelRegardlessOfJointOrderInXml)
{
    // The bug this whole stage exists for. Reordering two <joint> elements that
    // share a parent changes no geometry, so it must change no outcome. Before
    // R2 one order loaded and the other returned incomplete_kinematic_chain.
    const TemporaryUrdf fixedFirst{branchingRobot(true)};
    const TemporaryUrdf actuatedFirst{branchingRobot(false)};

    const auto a = mt::kinematics::load_urdf(fixedFirst.path());
    const auto b = mt::kinematics::load_urdf(actuatedFirst.path());

    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());

    // Compared as sets keyed by name: the joint vector legitimately preserves
    // source order, so comparing index by index would test the wrong thing.
    const auto right = b->model.joints();
    ASSERT_EQ(a->model.joints().size(), right.size());
    for (auto const& left : a->model.joints())
    {
        const auto it = std::find_if(right.begin(), right.end(),
                                     [&](auto const& r) { return r.name == left.name; });
        ASSERT_NE(it, right.end()) << "joint missing after reorder: " << left.name;
        EXPECT_EQ(it->parent_link, left.parent_link);
        EXPECT_EQ(it->child_link, left.child_link);
        EXPECT_EQ(static_cast<int>(it->type), static_cast<int>(left.type));
        EXPECT_DOUBLE_EQ(it->origin_xyz.z, left.origin_xyz.z);
        EXPECT_DOUBLE_EQ(it->axis.z, left.axis.z);
    }
}

TEST(UrdfModelLoaderTest, BuildsSameChainRegardlessOfJointOrderInXml)
{
    // Stronger than the previous test and closer to what matters: the selected
    // kinematic chain must be identical, not merely the input equivalent.
    const kinemaforge::ik::UrdfModelLoader loader;
    const kinemaforge::ik::KinematicChainBuilder chainBuilder;

    const TemporaryUrdf fixedFirst{branchingRobot(true)};
    const TemporaryUrdf actuatedFirst{branchingRobot(false)};

    const auto a = chainBuilder.build(loader.load(fixedFirst.path()), "base", "tool");
    const auto b = chainBuilder.build(loader.load(actuatedFirst.path()), "base", "tool");

    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    ASSERT_EQ(a->joints.size(), b->joints.size());
    for (std::size_t i = 0; i < a->joints.size(); ++i)
    {
        EXPECT_EQ(a->joints[i].name, b->joints[i].name);
        EXPECT_EQ(a->joints[i].variable.has_value(), b->joints[i].variable.has_value());
    }
}

TEST(UrdfModelLoaderTest, RejectsEmptyLinkName)
{
    // T0a. link.name is node identity here, so an unnamed node is not a node.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name=""/><link name="B"/>)"
        R"(<joint name="j" type="fixed"><parent link=""/><child link="B"/></joint>)"
        R"(</robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::invalid_link_declaration);
}

TEST(UrdfModelLoaderTest, RejectsDuplicateLinkName)
{
    // T0b. Two <link> elements sharing a name collapse into one node in any set
    // keyed by name, so every later invariant would judge a graph that does not
    // match the file.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="A"/><link name="A"/>)"
        R"(</robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::invalid_link_declaration);
}

namespace {

// Two unit links; j2 is declared as a mimic of j1. Treated as an independent
// axis, j2 gives the arm a two-dimensional workspace, while the coupling leaves
// it a single curve -- IK would return configurations that violate the
// coupling. Audit 2026-11-16 #1.
std::string mimicArmUrdf(std::string_view mimicElement)
{
    return std::string{
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="l1"/><link name="tool"/>)"
        R"(<joint name="j1" type="revolute">)"
        R"(<parent link="base"/><child link="l1"/>)"
        R"(<axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/>)"
        R"(</joint>)"
        R"(<joint name="j2" type="revolute">)"
        R"(<origin xyz="1 0 0"/><parent link="l1"/><child link="tool"/>)"
        R"(<axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/>)"}
        + std::string{mimicElement} +
        R"(</joint></robot>)";
}

} // namespace

// --- Mimic joints (follow-up 2026-11-16 #2) ----------------------------------
//
// A mimic joint is loaded with its coupling (joint = multiplier * other +
// offset) and validated. It is rejected only where it would be misread: inside
// a kinematic chain, where every consumer treats each joint as an independent
// axis (KinematicChainBuilder -> MimicJointInChain). The common case -- gripper
// fingers mimicking each other beside the arm -- now loads and works.

TEST(UrdfModelLoaderTest, LoadsMimicJointWithItsCoupling)
{
    const TemporaryUrdf file{mimicArmUrdf(R"(<mimic joint="j1" multiplier="-2" offset="0.5"/>)")};
    const auto result = mt::kinematics::load_urdf(file.path());
    ASSERT_TRUE(result.has_value());
    const auto& joints = result->model.joints();
    ASSERT_EQ(joints.size(), 2U);
    EXPECT_FALSE(joints[0].mimic.has_value());
    ASSERT_TRUE(joints[1].mimic.has_value());
    EXPECT_EQ(joints[1].mimic->joint, "j1");
    EXPECT_EQ(joints[1].mimic->multiplier, -2.0);
    EXPECT_EQ(joints[1].mimic->offset, 0.5);

    const auto description = kinemaforge::ik::UrdfModelLoader{}.load(file.path());
    ASSERT_TRUE(description.joints.at(1).mimic.has_value());
    EXPECT_EQ(description.joints[1].mimic->joint, "j1");
    EXPECT_EQ(description.joints[1].mimic->multiplier, -2.0);
    EXPECT_EQ(description.joints[1].mimic->offset, 0.5);
}

TEST(UrdfModelLoaderTest, MimicMultiplierAndOffsetDefaultToOneAndZero)
{
    const TemporaryUrdf file{mimicArmUrdf(R"(<mimic joint="j1"/>)")};
    const auto result = mt::kinematics::load_urdf(file.path());
    ASSERT_TRUE(result.has_value());
    ASSERT_TRUE(result->model.joints()[1].mimic.has_value());
    EXPECT_EQ(result->model.joints()[1].mimic->multiplier, 1.0);
    EXPECT_EQ(result->model.joints()[1].mimic->offset, 0.0);
}

TEST(UrdfModelLoaderTest, RejectsInvalidMimicReferences)
{
    using mt::kinematics::LoadErrorCode;
    expectRejectedWith(mimicArmUrdf(R"(<mimic joint="ghost"/>)"), LoadErrorCode::invalid_mimic_reference);
    expectRejectedWith(mimicArmUrdf(R"(<mimic joint="j2"/>)"), LoadErrorCode::invalid_mimic_reference);
    expectRejectedWith(mimicArmUrdf(R"(<mimic joint=""/>)"), LoadErrorCode::invalid_mimic_reference);
    expectRejectedWith(mimicArmUrdf(R"(<mimic/>)"), LoadErrorCode::invalid_mimic_reference);
    expectRejectedWith(mimicArmUrdf(R"(<mimic joint="j1" multiplier="abc"/>)"),
                       LoadErrorCode::invalid_mimic_reference);
    expectRejectedWith(mimicArmUrdf(R"(<mimic joint="j1" offset="nan"/>)"),
                       LoadErrorCode::invalid_mimic_reference);
    expectRejectedWith(mimicArmUrdf(R"(<mimic joint="j1"/><mimic joint="j1"/>)"),
                       LoadErrorCode::invalid_mimic_reference);
}

TEST(UrdfModelLoaderTest, RejectsMimicOfAMimicJoint)
{
    // Chains of couplings are not resolved; the referenced joint must be
    // independent.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="a"/><link name="b"/><link name="c"/><link name="d"/>)"
        R"(<joint name="j1" type="revolute"><parent link="a"/><child link="b"/>)"
        R"(<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>)"
        R"(<joint name="j2" type="revolute"><parent link="b"/><child link="c"/>)"
        R"(<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/><mimic joint="j1"/></joint>)"
        R"(<joint name="j3" type="revolute"><parent link="c"/><child link="d"/>)"
        R"(<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/><mimic joint="j2"/></joint>)"
        R"(</robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::invalid_mimic_reference);
}

TEST(UrdfModelLoaderTest, RejectsMimicOnAFixedJoint)
{
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="a"/><link name="b"/><link name="c"/>)"
        R"(<joint name="j1" type="revolute"><parent link="a"/><child link="b"/>)"
        R"(<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>)"
        R"(<joint name="f" type="fixed"><parent link="b"/><child link="c"/><mimic joint="j1"/></joint>)"
        R"(</robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::unsupported_mimic_joint);
}

TEST(UrdfModelLoaderTest, ChainThroughAMimicJointIsRejected)
{
    // Inside a chain the coupling would be read as an independent axis.
    const TemporaryUrdf file{mimicArmUrdf(R"(<mimic joint="j1"/>)")};
    const auto robot = kinemaforge::ik::UrdfModelLoader{}.load(file.path());
    const auto chain = kinemaforge::ik::KinematicChainBuilder{}.build(robot, "base", "tool");
    ASSERT_FALSE(chain.has_value());
    EXPECT_EQ(chain.error(), kinemaforge::ik::KinematicChainError::MimicJointInChain);
    // The part of the chain before the mimic joint is unaffected.
    EXPECT_TRUE(kinemaforge::ik::KinematicChainBuilder{}.build(robot, "base", "l1").has_value());
}

TEST(UrdfModelLoaderTest, ArmWithMimicGripperFingersBuildsTheArmChain)
{
    // The usual reason a URDF has <mimic>: two gripper fingers, one mimicking
    // the other, beside the arm. Previously the whole model was refused.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="tool"/><link name="finger_l"/><link name="finger_r"/>)"
        R"(<joint name="arm" type="revolute"><parent link="base"/><child link="tool"/>)"
        R"(<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>)"
        R"(<joint name="left" type="prismatic"><parent link="tool"/><child link="finger_l"/>)"
        R"(<axis xyz="0 1 0"/><limit lower="0" upper="0.04" effort="1" velocity="1"/></joint>)"
        R"(<joint name="right" type="prismatic"><parent link="tool"/><child link="finger_r"/>)"
        R"(<axis xyz="0 -1 0"/><limit lower="0" upper="0.04" effort="1" velocity="1"/>)"
        R"(<mimic joint="left"/></joint>)"
        R"(</robot>)";
    const TemporaryUrdf file{urdf};
    const auto robot = kinemaforge::ik::UrdfModelLoader{}.load(file.path());
    const auto chain = kinemaforge::ik::KinematicChainBuilder{}.build(robot, "base", "tool");
    ASSERT_TRUE(chain.has_value());
    EXPECT_EQ(chain->joints.size(), 1U);
}

TEST(UrdfModelLoaderTest, RejectsEmptyJointName)
{
    // Joint names identify axes in IK results (JointValue::name), so an
    // unnamed joint cannot be reported back unambiguously.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="tool"/>)"
        R"(<joint name="" type="revolute">)"
        R"(<parent link="base"/><child link="tool"/>)"
        R"(<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>)"
        R"(</joint></robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::invalid_joint_declaration);
}

TEST(UrdfModelLoaderTest, RejectsDuplicateMovableJointName)
{
    // Audit 2026-11-16 #6: two movable joints called "same" loaded fine and made
    // identification of axes by name in IK results ambiguous.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="mid"/><link name="tool"/>)"
        R"(<joint name="same" type="revolute">)"
        R"(<parent link="base"/><child link="mid"/>)"
        R"(<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>)"
        R"(</joint>)"
        R"(<joint name="same" type="revolute">)"
        R"(<parent link="mid"/><child link="tool"/>)"
        R"(<axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/>)"
        R"(</joint></robot>)";
    const TemporaryUrdf file{urdf};
    const auto result = mt::kinematics::load_urdf(file.path());
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, mt::kinematics::LoadErrorCode::invalid_joint_declaration);
    EXPECT_EQ(result.error().jointName, "same");
}

TEST(UrdfModelLoaderTest, RejectsDuplicateJointNameAcrossJointTypes)
{
    // URDF requires joint names to be unique regardless of type.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="mid"/><link name="tool"/>)"
        R"(<joint name="j" type="fixed"><parent link="base"/><child link="mid"/></joint>)"
        R"(<joint name="j" type="revolute">)"
        R"(<parent link="mid"/><child link="tool"/>)"
        R"(<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>)"
        R"(</joint></robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::invalid_joint_declaration);
}

TEST(UrdfModelLoaderTest, RejectsUndeclaredParentLink)
{
    // T1. A typo used to conjure a usable frame out of thin air.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="tool"/>)"
        R"(<joint name="j" type="revolute">)"
        R"(<parent link="bas"/><child link="tool"/>)"
        R"(<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>)"
        R"(</joint></robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::undeclared_parent_link);
}

TEST(UrdfModelLoaderTest, RejectsUndeclaredChildLink)
{
    // T2.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="tool"/>)"
        R"(<joint name="j" type="revolute">)"
        R"(<parent link="base"/><child link="ghost"/>)"
        R"(<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>)"
        R"(</joint></robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::undeclared_child_link);
}

TEST(UrdfModelLoaderTest, ReportsUndeclaredLinkWithJointContext)
{
    // The code alone is not actionable in a file with thirty joints.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="tool"/>)"
        R"(<joint name="shoulder" type="revolute">)"
        R"(<parent link="typo"/><child link="tool"/>)"
        R"(<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>)"
        R"(</joint></robot>)";

    const TemporaryUrdf file{urdf};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, mt::kinematics::LoadErrorCode::undeclared_parent_link);
    EXPECT_EQ(result.error().jointName, "shoulder");
    EXPECT_EQ(result.error().attribute, "parent/link");
    ASSERT_TRUE(result.error().rawValue.has_value());
    EXPECT_EQ(*result.error().rawValue, "typo");
}

TEST(UrdfModelLoaderTest, RejectsLinkClaimedByTwoJoints)
{
    // T3 -- what makes the graph a tree rather than a DAG. Without it
    // T_parent_child is not well defined for the twice-claimed link.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="mid"/><link name="tool"/>)"
        R"(<joint name="a" type="fixed"><parent link="base"/><child link="tool"/></joint>)"
        R"(<joint name="b" type="fixed"><parent link="mid"/><child link="tool"/></joint>)"
        R"(</robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::duplicate_child_link);
}

TEST(UrdfModelLoaderTest, RejectsCyclicTopologyWithNoRoot)
{
    // T4, branch roots == 0. Every link has a parent, so walking up parents in a
    // finite graph cannot terminate: a cycle is certain.
    //
    // The point of this test is that load_urdf RETURNS. The previous
    // implementation looped forever while growing memory, so it would have hung
    // ctest rather than failed it.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="ring">)"
        R"(<link name="A"/><link name="B"/><link name="C"/>)"
        R"(<joint name="ab" type="fixed"><parent link="A"/><child link="B"/></joint>)"
        R"(<joint name="bc" type="fixed"><parent link="B"/><child link="C"/></joint>)"
        R"(<joint name="ca" type="fixed"><parent link="C"/><child link="A"/></joint>)"
        R"(</robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::cyclic_topology);
}

TEST(UrdfModelLoaderTest, RejectsCyclicTopologyUnreachableFromRoot)
{
    // T4, branch roots == 1 but visited < links. The unvisited component cannot
    // hold a root -- there is exactly one and it was visited -- so every node
    // there has a parent and, by the same argument, the component has a cycle.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="rpr">)"
        R"(<link name="root"/><link name="X"/><link name="A"/><link name="B"/>)"
        R"(<joint name="rx" type="fixed"><parent link="root"/><child link="X"/></joint>)"
        R"(<joint name="ab" type="fixed"><parent link="A"/><child link="B"/></joint>)"
        R"(<joint name="ba" type="fixed"><parent link="B"/><child link="A"/></joint>)"
        R"(</robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::cyclic_topology);
}

TEST(UrdfModelLoaderTest, RejectsDisconnectedForest)
{
    // T5. This used to be accepted, and one whole subtree disappeared without a
    // diagnostic -- a silent wrong answer rather than a loud failure. Which
    // subtree survived depended on unordered_map iteration order.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="forest">)"
        R"(<link name="rootA"/><link name="a1"/><link name="rootB"/><link name="b1"/>)"
        R"(<joint name="ja" type="fixed"><parent link="rootA"/><child link="a1"/></joint>)"
        R"(<joint name="jb" type="fixed"><parent link="rootB"/><child link="b1"/></joint>)"
        R"(</robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::disconnected_topology);
}

TEST(UrdfModelLoaderTest, AcceptsSingleLinkWithNoJoints)
{
    // A one-node tree is legal. It is also what makes baseLink == toolLink ->
    // empty KinematicChain reachable from an actual file.
    const TemporaryUrdf file{
        R"(<?xml version="1.0"?><robot name="o"><link name="only"/></robot>)"};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->model.links().size(), 1u);
    EXPECT_TRUE(result->model.joints().empty());
}

TEST(UrdfModelLoaderTest, RejectsSeveralIsolatedLinksWithNoJoints)
{
    // Two links and no joints are two isolated nodes, which is a forest. An
    // earlier draft of this stage would have accepted it, contradicting T5.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t"><link name="A"/><link name="B"/></robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::disconnected_topology);
}

// ---------------------------------------------------------------------------
// Joint limits — the KinemaForge strict profile (stage F2.3.R, R3)
//
// <limit> used to be read with pugixml's as_double(), which answers 0.0 both to
// a missing attribute and to lower="banana". A joint with no lower/upper loaded
// as [0, 0] -- actuated, frozen, and passing the old lower > upper check because
// 0 > 0 is false. velocity and effort were never checked at all, so "nan" and
// "inf" reached the domain model.
// ---------------------------------------------------------------------------

namespace {

// A revolute joint whose <limit> is exactly the given XML, so each test varies
// one thing. urdfWithJoint supplies <link name="base"/> and <link name="tool"/>,
// which the topology invariants from R2 require.
std::string jointWithLimit(std::string_view limitXml)
{
    return std::string{R"(<joint name="j" type="revolute">)"}
         + R"(<parent link="base"/><child link="tool"/>)"
         + R"(<axis xyz="0 0 1"/>)"
         + std::string{limitXml}
         + "</joint>";
}

void expectLimitRejected(std::string_view limitXml, mt::kinematics::LoadErrorCode expected)
{
    expectRejectedWith(urdfWithJoint(jointWithLimit(limitXml)), expected);
}

} // namespace

// --- element cardinality ---------------------------------------------------

TEST(UrdfModelLoaderTest, RejectsMissingLimitElement)
{
    const std::string joint =
        std::string{R"(<joint name="j" type="revolute">)"}
        + R"(<parent link="base"/><child link="tool"/><axis xyz="0 0 1"/>)"
        + "</joint>";
    expectRejectedWith(urdfWithJoint(joint),
                       mt::kinematics::LoadErrorCode::missing_limit_element);
}

TEST(UrdfModelLoaderTest, RejectsDuplicateLimitElement)
{
    // Two <limit> elements mean two different things. node.child() would take
    // the first and say nothing, which is the defect class this stage removes.
    expectLimitRejected(R"(<limit lower="-1" upper="1" velocity="2" effort="10"/>)"
                        R"(<limit lower="-3" upper="3" velocity="5" effort="50"/>)",
                        mt::kinematics::LoadErrorCode::duplicate_limit_element);
}

// --- attribute presence ----------------------------------------------------

TEST(UrdfModelLoaderTest, RejectsMissingLowerLimit)
{
    // Regression: this used to load with limits [0, 0] -- an actuated joint
    // frozen in place and indistinguishable from a modelling mistake.
    expectLimitRejected(R"(<limit upper="1" velocity="1" effort="1"/>)",
                        mt::kinematics::LoadErrorCode::missing_limit_attribute);
}

TEST(UrdfModelLoaderTest, RejectsMissingUpperLimit)
{
    expectLimitRejected(R"(<limit lower="-1" velocity="1" effort="1"/>)",
                        mt::kinematics::LoadErrorCode::missing_limit_attribute);
}

TEST(UrdfModelLoaderTest, RejectsMissingVelocity)
{
    expectLimitRejected(R"(<limit lower="-1" upper="1" effort="1"/>)",
                        mt::kinematics::LoadErrorCode::missing_limit_attribute);
}

TEST(UrdfModelLoaderTest, RejectsMissingEffort)
{
    expectLimitRejected(R"(<limit lower="-1" upper="1" velocity="1"/>)",
                        mt::kinematics::LoadErrorCode::missing_limit_attribute);
}

TEST(UrdfModelLoaderTest, ReportsFirstMissingLimitAttributeInContractOrder)
{
    // The four tests above cannot pin the checking order: with one fault per
    // input, any order produces the same result. Only a colliding input can, and
    // the order is a contract -- a caller fixing faults one at a time gets a
    // deterministic sequence rather than one that depends on how the validator
    // happens to be written today.

    // lower, upper AND velocity missing -> lower wins over both.
    {
        const TemporaryUrdf file{urdfWithJoint(jointWithLimit(R"(<limit effort="1"/>)"))};
        const auto result = mt::kinematics::load_urdf(file.path());

        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code,
                  mt::kinematics::LoadErrorCode::missing_limit_attribute);
        EXPECT_EQ(result.error().attribute, "limit/lower");
    }

    // Second half of the chain: velocity wins over effort.
    {
        const TemporaryUrdf file{
            urdfWithJoint(jointWithLimit(R"(<limit lower="-1" upper="1"/>)"))};
        const auto result = mt::kinematics::load_urdf(file.path());

        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code,
                  mt::kinematics::LoadErrorCode::missing_limit_attribute);
        EXPECT_EQ(result.error().attribute, "limit/velocity");
    }
}

// --- value form ------------------------------------------------------------

TEST(UrdfModelLoaderTest, RejectsMalformedLowerLimit)
{
    expectLimitRejected(R"(<limit lower="banana" upper="1" velocity="1" effort="1"/>)",
                        mt::kinematics::LoadErrorCode::malformed_limit_value);
}

TEST(UrdfModelLoaderTest, RejectsMalformedUpperLimit)
{
    // Two numbers where one is required. This is the test that pins the layer
    // boundary: if the END rule leaked into parse_finite_number, this one alone
    // would fail while the rest still passed.
    expectLimitRejected(R"(<limit lower="-1" upper="1 2" velocity="1" effort="1"/>)",
                        mt::kinematics::LoadErrorCode::malformed_limit_value);
}

TEST(UrdfModelLoaderTest, RejectsEmptyLimitAttribute)
{
    expectLimitRejected(R"(<limit lower="" upper="1" velocity="1" effort="1"/>)",
                        mt::kinematics::LoadErrorCode::malformed_limit_value);
}

TEST(UrdfModelLoaderTest, RejectsNonFiniteVelocity)
{
    // Regression: strtod parses "nan", and the old isfinite check covered only
    // lower and upper, so this reached the domain model.
    expectLimitRejected(R"(<limit lower="-1" upper="1" velocity="nan" effort="1"/>)",
                        mt::kinematics::LoadErrorCode::malformed_limit_value);
}

TEST(UrdfModelLoaderTest, RejectsNonFiniteEffort)
{
    expectLimitRejected(R"(<limit lower="-1" upper="1" velocity="1" effort="inf"/>)",
                        mt::kinematics::LoadErrorCode::malformed_limit_value);
}

TEST(UrdfModelLoaderTest, RejectsOverflowingLimitValue)
{
    // Distinct from "not a number": 1e400 is valid syntax that from_chars
    // rejects with result_out_of_range, a different branch.
    expectLimitRejected(R"(<limit lower="-1" upper="1e400" velocity="1" effort="1"/>)",
                        mt::kinematics::LoadErrorCode::malformed_limit_value);
}

// --- relations -------------------------------------------------------------

TEST(UrdfModelLoaderTest, RejectsInvertedLimitRange)
{
    expectLimitRejected(R"(<limit lower="1" upper="-1" velocity="1" effort="1"/>)",
                        mt::kinematics::LoadErrorCode::invalid_limit_constraint);
}

TEST(UrdfModelLoaderTest, RejectsNegativeVelocity)
{
    expectLimitRejected(R"(<limit lower="-1" upper="1" velocity="-2" effort="1"/>)",
                        mt::kinematics::LoadErrorCode::invalid_limit_constraint);
}

TEST(UrdfModelLoaderTest, RejectsNegativeEffort)
{
    expectLimitRejected(R"(<limit lower="-1" upper="1" velocity="1" effort="-5"/>)",
                        mt::kinematics::LoadErrorCode::invalid_limit_constraint);
}

// --- the edge of strictness: what must still be accepted -------------------

TEST(UrdfModelLoaderTest, AcceptsZeroWidthLimitRange)
{
    // Explicit input, written twice by the author -- not a value the parser
    // invented from silence. That distinction is the whole reason lower and
    // upper are required.
    const auto joint = loadSingleJoint(urdfWithJoint(jointWithLimit(
        R"(<limit lower="1.5" upper="1.5" velocity="1" effort="1"/>)")));

    EXPECT_DOUBLE_EQ(joint.limits.lower, 1.5);
    EXPECT_DOUBLE_EQ(joint.limits.upper, 1.5);
}

TEST(UrdfModelLoaderTest, AcceptsZeroVelocityAndEffort)
{
    // Zero is finite and non-negative. This validates data; it does not judge
    // whether a robot with those numbers is useful.
    const auto joint = loadSingleJoint(urdfWithJoint(jointWithLimit(
        R"(<limit lower="-1" upper="1" velocity="0" effort="0"/>)")));

    EXPECT_DOUBLE_EQ(joint.limits.velocity, 0.0);
    EXPECT_DOUBLE_EQ(joint.limits.effort, 0.0);
}

TEST(UrdfModelLoaderTest, AcceptsLimitValuesInScientificNotation)
{
    const auto joint = loadSingleJoint(urdfWithJoint(jointWithLimit(
        R"(<limit lower="-1.5e-3" upper="2.5e2" velocity="1e1" effort="1e2"/>)")));

    EXPECT_DOUBLE_EQ(joint.limits.lower, -1.5e-3);
    EXPECT_DOUBLE_EQ(joint.limits.upper, 2.5e2);
}

TEST(UrdfModelLoaderTest, AcceptsLimitValuesWithSurroundingWhitespace)
{
    // Pins the full parse_scalar contract, not just its END rule: leading and
    // trailing whitespace are allowed around a limit value. The '+' on upper
    // also exercises the leading-sign branch through the scalar path -- the
    // existing whitespace and plus-sign tests both go through parse_vector3,
    // which is a different layer since parse_finite_number was extracted.
    const auto joint = loadSingleJoint(urdfWithJoint(jointWithLimit(
        R"(<limit lower="  -1.5  " upper="  +1.5  ")"
        R"( velocity="  2.0  " effort="  10.0  "/>)")));

    EXPECT_DOUBLE_EQ(joint.limits.lower, -1.5);
    EXPECT_DOUBLE_EQ(joint.limits.upper, 1.5);
    EXPECT_DOUBLE_EQ(joint.limits.velocity, 2.0);
    EXPECT_DOUBLE_EQ(joint.limits.effort, 10.0);
}

TEST(UrdfModelLoaderTest, IgnoresLimitForFixedJoint)
{
    // A limit on a joint that cannot move has no meaning, so its contents are
    // not even parsed. Without this test the decision lives only in a comment.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="tool"/>)"
        R"(<joint name="j" type="fixed">)"
        R"(<parent link="base"/><child link="tool"/>)"
        R"(<limit lower="banana" upper="also nonsense"/>)"
        R"(</joint></robot>)";

    const TemporaryUrdf file{urdf};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->model.joints().size(), 1u);
}

// --- diagnostics -----------------------------------------------------------

TEST(UrdfModelLoaderTest, ReportsMalformedLimitWithAttributeContext)
{
    // The code alone is not actionable: it must name the joint and the attribute
    // and quote back what was actually there.
    const TemporaryUrdf file{urdfWithJoint(jointWithLimit(
        R"(<limit lower="banana" upper="1" velocity="1" effort="1"/>)"))};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, mt::kinematics::LoadErrorCode::malformed_limit_value);
    EXPECT_EQ(result.error().jointName, "j");
    EXPECT_EQ(result.error().attribute, "limit/lower");
    ASSERT_TRUE(result.error().rawValue.has_value());
    EXPECT_EQ(*result.error().rawValue, "banana");
}

TEST(UrdfModelLoaderTest, ReportsGeometryFaultBeforeTopologyFault)
{
    // The T0..T5 ordering is a contract WITHIN topology validation. Joint
    // parsing runs first, so a file broken both ways reports the geometry
    // fault. Pinned so the header's promise is not read as a global priority
    // over syntax and geometry errors.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="tool"/>)"
        R"(<joint name="j" type="revolute">)"
        R"(<parent link="undeclared"/><child link="tool"/>)"
        R"(<axis xyz="abc"/><limit lower="-1" upper="1" effort="1" velocity="1"/>)"
        R"(</joint></robot>)";
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::malformed_vector);
}
