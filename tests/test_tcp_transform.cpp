#include <gtest/gtest.h>

#include "ik_equations/IkEquationBuilder.hpp"
#include "ik_equations/model/FixedRigidTransform.hpp"
#include "support/NumericForwardKinematics.hpp"
#include "support/TransformComparison.hpp"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace ik = kinemaforge::ik;
namespace support = kinemaforge::testsupport;

using ik::FixedRigidTransform;
using ik::IkEquationBuilder;
using ik::IkEquationBuilderErrorCode;
using ik::sameNode;
using support::JointConfiguration;
using support::Matrix4;

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

// Drives the facade up to a built T_base_tip.
void prepare(IkEquationBuilder& builder, const char* fileName)
{
    ASSERT_TRUE(builder.loadRobotModel(urdfPath(fileName)).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.buildForwardKinematics().has_value());
}

FixedRigidTransform translationTcp(double x, double y, double z)
{
    return FixedRigidTransform{{x, y, z}, {}};
}

// The same nine configurations Phase 1 validated FK against: zero, six
// single-joint poses, mixed, and near-limits. Checking only zero and mixed
// would leave the claim ("the same configurations as Phase 1") wider than what
// the tests actually cover.
std::vector<JointConfiguration> tcpValidationConfigurations(const ik::KinematicChain& chain)
{
    std::vector<JointConfiguration> configurations;

    configurations.emplace_back(6, 0.0);

    for (std::size_t index = 0; index < 6; ++index)
    {
        JointConfiguration single(6, 0.0);
        single[index] = (index % 2 == 0) ? 0.25 : -0.25;
        configurations.push_back(std::move(single));
    }

    configurations.push_back({0.35, -0.45, 0.55, -0.65, 0.40, -0.30});
    configurations.push_back(support::nearLimitConfiguration(chain));

    return configurations;
}

// Nine configurations against the quaternion reference, for one robot.
void expectTcpMatchesReference(IkEquationBuilder& builder, const char* fileName,
                               const FixedRigidTransform& tcp)
{
    prepare(builder, fileName);
    ASSERT_TRUE(builder.setTcp(tcp).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto* chain = builder.kinematicChain();
    ASSERT_NE(chain, nullptr);
    const auto* symbolic = builder.tcpForwardKinematics();
    ASSERT_NE(symbolic, nullptr);

    std::size_t index = 0;
    for (const auto& configuration : tcpValidationConfigurations(*chain))
    {
        SCOPED_TRACE(testing::Message() << fileName << " configuration " << index);

        const auto numeric =
            support::evaluateSymbolic(*symbolic, support::makeSymbolValues(*chain, configuration));
        ASSERT_TRUE(numeric.has_value());

        const Matrix4 reference = support::toMatrix4(
            support::numericTcpForwardKinematics(*chain, configuration, tcp));

        support::expectMatrixMatches(
            *numeric, reference,
            std::string(fileName) + " tcp cfg " + std::to_string(index));
        ++index;
    }
}

} // namespace

// --- composition ----------------------------------------------------

TEST(TcpTransformTest, IdentityTcpLeavesForwardKinematicsUnchanged)
{
    // "adds no nodes" is a claim about node identity, not about shape: a tree
    // rebuilt from scratch in the same shape would satisfy structurallyEqual.
    // multiplyTransforms literally returns lhs for an identity rhs, so
    // sameNode is the assertion that actually pins it.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    ASSERT_TRUE(builder.setTcp(FixedRigidTransform{}).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto* tip = builder.forwardKinematics();
    const auto* tcp = builder.tcpForwardKinematics();
    ASSERT_NE(tip, nullptr);
    ASSERT_NE(tcp, nullptr);

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            EXPECT_TRUE(sameNode((*tip)(row, column), (*tcp)(row, column)));
        }
}

TEST(TcpTransformTest, AppliesTcpTranslationInToolFrame)
{
    // The single most important test of this stage.
    //
    // kr640 at q1 = pi/2 has R_base_tip = Rz(pi/2) and p_base_tip =
    // (0, 1.600, 2.335), both hand-computed and already pinned by the Phase 1
    // validation. A TCP of (0.1, 0, 0) is expressed in the TOOL frame, so:
    //
    //   p_base_tcp = (0, 1.600, 2.335) + Rz(pi/2) * (0.1, 0, 0)
    //              = (0, 1.700, 2.335)
    //
    // Adding the offset in the base frame instead would give
    // (0.1, 1.600, 2.335) -- a difference in a different axis, so this cannot
    // pass by accident. The expectation depends on neither FK implementation.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    ASSERT_TRUE(builder.setTcp(translationTcp(0.1, 0.0, 0.0)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    JointConfiguration configuration(6, 0.0);
    configuration[0] = kPi / 2.0;

    const auto* chain = builder.kinematicChain();
    ASSERT_NE(chain, nullptr);

    const auto numeric = support::evaluateSymbolic(
        *builder.tcpForwardKinematics(), support::makeSymbolValues(*chain, configuration));
    ASSERT_TRUE(numeric.has_value());

    Matrix4 expected{};
    expected[0][0] = 0.0; expected[0][1] = -1.0; expected[0][2] = 0.0; expected[0][3] = 0.000;
    expected[1][0] = 1.0; expected[1][1] =  0.0; expected[1][2] = 0.0; expected[1][3] = 1.700;
    expected[2][0] = 0.0; expected[2][1] =  0.0; expected[2][2] = 1.0; expected[2][3] = 2.335;
    expected[3][3] = 1.0;

    support::expectMatrixMatches(*numeric, expected, "kr640 q1=pi/2 + tcp x0.1 (hand computed)");
}

TEST(TcpTransformTest, AppliesTranslationOnlyTcp)
{
    // At q = 0 every kr640 transform is a pure translation, so a tool-frame
    // offset adds directly: (1.600, 0, 2.335) + (0, 0, 0.05).
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto numeric = support::evaluateSymbolic(
        *builder.tcpForwardKinematics(),
        support::makeSymbolValues(*builder.kinematicChain(), JointConfiguration(6, 0.0)));
    ASSERT_TRUE(numeric.has_value());

    Matrix4 expected{};
    expected[0][0] = expected[1][1] = expected[2][2] = expected[3][3] = 1.0;
    expected[0][3] = 1.600;
    expected[2][3] = 2.385;

    support::expectMatrixMatches(*numeric, expected, "kr640 zero + tcp z0.05");
}

TEST(TcpTransformTest, AppliesRotationOnlyTcp)
{
    // Rotation about Z by pi/2 at q = 0, where R_base_tip is identity, so the
    // result is exactly the TCP rotation and the position is unchanged.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    ASSERT_TRUE(builder.setTcp(FixedRigidTransform{{}, {0.0, 0.0, kPi / 2.0}}).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto numeric = support::evaluateSymbolic(
        *builder.tcpForwardKinematics(),
        support::makeSymbolValues(*builder.kinematicChain(), JointConfiguration(6, 0.0)));
    ASSERT_TRUE(numeric.has_value());

    Matrix4 expected{};
    expected[0][1] = -1.0;
    expected[1][0] =  1.0;
    expected[2][2] =  1.0;
    expected[0][3] = 1.600;
    expected[2][3] = 2.335;
    expected[3][3] = 1.0;

    support::expectMatrixMatches(*numeric, expected, "kr640 zero + tcp Rz(pi/2)");
}

TEST(TcpTransformTest, AppliesCombinedTcp)
{
    // Both at once, checked against the quaternion reference rather than by
    // hand -- the point here is that translation and rotation compose, which
    // the two tests above verify separately.
    const FixedRigidTransform tcp{{0.05, -0.02, 0.13}, {0.2, -0.3, 0.4}};
    const JointConfiguration configuration{0.35, -0.45, 0.55, -0.65, 0.40, -0.30};

    IkEquationBuilder builder;
    prepare(builder, "kr4_r600.urdf");
    ASSERT_TRUE(builder.setTcp(tcp).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto numeric = support::evaluateSymbolic(
        *builder.tcpForwardKinematics(),
        support::makeSymbolValues(*builder.kinematicChain(), configuration));
    ASSERT_TRUE(numeric.has_value());

    const Matrix4 reference = support::toMatrix4(
        support::numericTcpForwardKinematics(*builder.kinematicChain(), configuration, tcp));

    support::expectMatrixMatches(*numeric, reference, "kr4 mixed + combined tcp");
}

TEST(TcpTransformTest, PreservesCanonicalHomogeneousLastRow)
{
    IkEquationBuilder builder;
    prepare(builder, "kr4_r600.urdf");
    ASSERT_TRUE(builder.setTcp(FixedRigidTransform{{0.05, -0.02, 0.13},
                                                   {0.2, -0.3, 0.4}}).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto* result = builder.tcpForwardKinematics();
    ASSERT_NE(result, nullptr);
    EXPECT_TRUE(ik::hasCanonicalHomogeneousLastRow(*result));
}

// --- state graph ----------------------------------------------------

TEST(TcpTransformTest, ChangingTcpInvalidatesTcpForwardKinematics)
{
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());
    ASSERT_NE(builder.tcpForwardKinematics(), nullptr);

    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.09)).has_value());

    EXPECT_EQ(builder.tcpForwardKinematics(), nullptr);
}

TEST(TcpTransformTest, ClearingTcpInvalidatesTcpForwardKinematics)
{
    // Valid in any state, including before a chain is selected -- the contract
    // says clearTcp must not depend on the chain, and void/noexcept alone does
    // not guarantee that.
    {
        IkEquationBuilder fresh;
        fresh.clearTcp();
        EXPECT_EQ(fresh.tcp(), nullptr);
        EXPECT_EQ(fresh.tcpForwardKinematics(), nullptr);
    }

    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    builder.clearTcp();

    EXPECT_EQ(builder.tcp(), nullptr);
    EXPECT_EQ(builder.tcpForwardKinematics(), nullptr);

    // Idempotent: a second call, with nothing set, also succeeds.
    builder.clearTcp();
    EXPECT_EQ(builder.tcp(), nullptr);
}

TEST(TcpTransformTest, RebuildingForwardKinematicsInvalidatesTcpForwardKinematicsButPreservesTcp)
{
    // The TCP is a sibling of the transform in the dependency graph, not a
    // descendant.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    ASSERT_TRUE(builder.buildForwardKinematics().has_value());

    EXPECT_NE(builder.tcp(), nullptr);
    EXPECT_EQ(builder.tcpForwardKinematics(), nullptr);
}

TEST(TcpTransformTest, SettingTcpPreservesForwardKinematics)
{
    // The other side of the graph: T_base_tip does not depend on the TCP, so
    // its pointer stays valid AND keeps pointing at the same result.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    const auto* fkBefore = builder.forwardKinematics();
    ASSERT_NE(fkBefore, nullptr);

    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());

    EXPECT_EQ(builder.forwardKinematics(), fkBefore);
}

TEST(TcpTransformTest, ChangingChainInvalidatesTcpAndTcpForwardKinematics)
{
    // The same three numbers would mean an offset from a DIFFERENT physical
    // frame after the tip changes, so keeping them would be a plausible wrong
    // answer.
    IkEquationBuilder builder;
    prepare(builder, "kr4_r600.urdf");
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    ASSERT_TRUE(builder.selectChain("base_link", "base").has_value());

    EXPECT_EQ(builder.tcp(), nullptr);
    EXPECT_EQ(builder.tcpForwardKinematics(), nullptr);
}

TEST(TcpTransformTest, LoadingNewRobotInvalidatesTcpAndTcpForwardKinematics)
{
    IkEquationBuilder builder;
    prepare(builder, "kr4_r600.urdf");
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());

    EXPECT_EQ(builder.tcp(), nullptr);
    EXPECT_EQ(builder.tcpForwardKinematics(), nullptr);
}

// --- errors ---------------------------------------------------------

TEST(TcpTransformTest, RejectsTcpBeforeChainSelection)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());

    const auto result = builder.setTcp(translationTcp(0.0, 0.0, 0.05));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::KinematicChainNotSelected);
    EXPECT_EQ(builder.tcp(), nullptr);
}

TEST(TcpTransformTest, RejectsTcpForwardKinematicsBeforeChainSelection)
{
    // Pins the prerequisite ORDER, not just the failure: an implementation
    // checking the transform first would return ForwardKinematicsNotBuilt here
    // and still pass every other test in this file.
    IkEquationBuilder builder;

    const auto result = builder.buildTcpForwardKinematics();

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::KinematicChainNotSelected);
}

TEST(TcpTransformTest, RejectsTcpForwardKinematicsBeforeForwardKinematics)
{
    IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(urdfPath("kr640.urdf")).has_value());
    ASSERT_TRUE(builder.selectChain("base_link", "tool0").has_value());
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());

    const auto result = builder.buildTcpForwardKinematics();

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::ForwardKinematicsNotBuilt);
}

TEST(TcpTransformTest, RejectsTcpForwardKinematicsWithoutTcp)
{
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    const auto result = builder.buildTcpForwardKinematics();

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::TcpNotSet);
}

TEST(TcpTransformTest, RejectsNonFiniteTcpTranslation)
{
    // All three components, all three non-finite values. The implementation
    // has one branch per component, so checking only z would let a dropped
    // check on x through unnoticed.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    for (const double value : {kNaN, kInfinity, -kInfinity})
        for (const auto& tcp : {FixedRigidTransform{{value, 0.0, 0.0}, {}},
                                FixedRigidTransform{{0.0, value, 0.0}, {}},
                                FixedRigidTransform{{0.0, 0.0, value}, {}}})
        {
            SCOPED_TRACE(testing::Message()
                         << "translation (" << tcp.translation.x << ", "
                         << tcp.translation.y << ", " << tcp.translation.z << ")");

            const auto result = builder.setTcp(tcp);

            ASSERT_FALSE(result.has_value());
            EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::InvalidTcpTransform);
            EXPECT_FALSE(result.error().message.empty());
        }
}

TEST(TcpTransformTest, RejectsNonFiniteTcpRotation)
{
    // Roll, pitch and yaw, all three non-finite values -- same reasoning.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");

    for (const double value : {kNaN, kInfinity, -kInfinity})
        for (const auto& tcp : {FixedRigidTransform{{}, {value, 0.0, 0.0}},
                                FixedRigidTransform{{}, {0.0, value, 0.0}},
                                FixedRigidTransform{{}, {0.0, 0.0, value}}})
        {
            SCOPED_TRACE(testing::Message() << "rpy (" << tcp.rpy.x << ", " << tcp.rpy.y
                                            << ", " << tcp.rpy.z << ")");

            const auto result = builder.setTcp(tcp);

            ASSERT_FALSE(result.has_value());
            EXPECT_EQ(result.error().code, IkEquationBuilderErrorCode::InvalidTcpTransform);
            EXPECT_FALSE(result.error().message.empty());
        }
}

TEST(TcpTransformTest, FailedTcpUpdatePreservesPreviousState)
{
    // Validation and the transactional guarantee are different contracts, so
    // this is a separate test: pointer identity proves the object is
    // untouched, not merely still populated.
    IkEquationBuilder builder;
    prepare(builder, "kr640.urdf");
    ASSERT_TRUE(builder.setTcp(translationTcp(0.0, 0.0, 0.05)).has_value());
    ASSERT_TRUE(builder.buildTcpForwardKinematics().has_value());

    const auto* tcpBefore = builder.tcp();
    const auto* resultBefore = builder.tcpForwardKinematics();
    ASSERT_NE(tcpBefore, nullptr);
    ASSERT_NE(resultBefore, nullptr);

    const auto result = builder.setTcp(translationTcp(0.0, 0.0, kNaN));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(builder.tcp(), tcpBefore);
    EXPECT_EQ(builder.tcpForwardKinematics(), resultBefore);
    EXPECT_DOUBLE_EQ(builder.tcp()->translation.z, 0.05);
}

// --- real robots against the quaternion reference -------------------

TEST(TcpTransformTest, BuildsKr4TcpForwardKinematics)
{
    IkEquationBuilder builder;
    expectTcpMatchesReference(builder, "kr4_r600.urdf",
                              FixedRigidTransform{{0.05, -0.02, 0.13}, {0.2, -0.3, 0.4}});
}

TEST(TcpTransformTest, BuildsKr640TcpForwardKinematics)
{
    IkEquationBuilder builder;
    expectTcpMatchesReference(builder, "kr640.urdf",
                              FixedRigidTransform{{0.05, -0.02, 0.13}, {0.2, -0.3, 0.4}});
}
