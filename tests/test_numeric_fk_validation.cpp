#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/symbolic/ExpressionEvaluator.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ik = kinemaforge::ik;
namespace support = kinemaforge::testsupport;

using support::JointConfiguration;
using support::Matrix3;
using support::Quaternion;
using support::RigidTransform;

namespace {

constexpr double kPi = std::numbers::pi;

// Candidate tolerance, approved as a candidate only. The implementation report
// must state the measured worst-case error; exceeding this bound is a finding
// for review, NOT a licence to raise the number.
constexpr double kAbsoluteTolerance = 1e-12;
constexpr double kRelativeTolerance = 1e-12;

using Matrix4 = std::array<std::array<double, 4>, 4>;

std::filesystem::path urdfPath(const char* fileName)
{
    return std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName;
}

bool withinTolerance(double actual, double expected)
{
    return std::abs(actual - expected)
           <= kAbsoluteTolerance + kRelativeTolerance * std::abs(expected);
}

Matrix4 toMatrix4(const RigidTransform& transform)
{
    const Matrix3 rotation = support::toRotationMatrix(transform.rotation);

    Matrix4 result{};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            result[row][column] = rotation[row][column];

    result[0][3] = transform.translation.x;
    result[1][3] = transform.translation.y;
    result[2][3] = transform.translation.z;
    result[3][3] = 1.0;
    return result;
}

// R_error = R_expected^T * R_actual ; angle = acos((trace - 1) / 2).
//
// Diagnostic only -- the pass/fail condition stays per-cell. This answers the
// question a single cell difference cannot: is the discrepancy a real
// orientation error, or noise in one entry?
double orientationAngleError(const Matrix4& actual, const Matrix4& expected)
{
    // trace(R_expected^T * R_actual) = sum_i sum_k R_expected(k,i) * R_actual(k,i)
    double trace = 0.0;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t k = 0; k < 3; ++k)
            trace += expected[k][i] * actual[k][i];

    // clamp: with errors near 1e-16 the argument can leave [-1, 1].
    return std::acos(std::clamp(0.5 * (trace - 1.0), -1.0, 1.0));
}

// One evaluator for all sixteen cells -- that is the whole reason
// ExpressionEvaluator is a session. A per-cell evaluator would drop the cache
// between roots.
std::optional<Matrix4> evaluateSymbolic(const ik::SymbolicTransform& fk,
                                        const ik::SymbolValues& values)
{
    ik::ExpressionEvaluator evaluator{values};

    Matrix4 numeric{};
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            const auto value = evaluator.evaluate(fk(row, column));
            if (!value)
            {
                ADD_FAILURE() << "evaluation failed at cell (" << row << ", " << column
                              << "), code " << static_cast<int>(value.error().code)
                              << " symbol '" << value.error().symbolName << "'";
                return std::nullopt;
            }
            numeric[row][column] = *value;
        }
    return numeric;
}

void expectMatrixMatches(const Matrix4& actual, const Matrix4& expected,
                         std::string_view label)
{
    double maxRotationError = 0.0;
    double maxTranslationError = 0.0;

    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
        {
            SCOPED_TRACE(testing::Message()
                         << label << " cell (" << row << ", " << column << ")");

            const double error = std::abs(actual[row][column] - expected[row][column]);
            if (row < 3 && column < 3)
                maxRotationError = std::max(maxRotationError, error);
            else if (row < 3)
                maxTranslationError = std::max(maxTranslationError, error);

            EXPECT_TRUE(withinTolerance(actual[row][column], expected[row][column]))
                << "actual=" << actual[row][column]
                << " expected=" << expected[row][column]
                << " error=" << error;
        }

    // Printed unconditionally: the implementation report quotes these.
    std::cout << "[ MEASURE  ] " << label
              << "  rotation=" << maxRotationError
              << "  translation=" << maxTranslationError
              << "  angle=" << orientationAngleError(actual, expected) << "\n";
}

struct LoadedRobot
{
    ik::KinematicChain chain;
    ik::SymbolicTransform fk;
};

std::optional<LoadedRobot> loadRobot(const char* fileName)
{
    ik::UrdfModelLoader loader;
    const auto robot = loader.load(urdfPath(fileName));

    ik::KinematicChainBuilder chainBuilder;
    auto chain = chainBuilder.build(robot, "base_link", "tool0");
    if (!chain)
    {
        ADD_FAILURE() << "chain build failed for " << fileName;
        return std::nullopt;
    }

    const ik::JointTransformBuilder transformBuilder;
    const ik::ForwardKinematicsBuilder fkBuilder;
    auto fk = fkBuilder.build(*chain, transformBuilder);

    return LoadedRobot{std::move(*chain), std::move(fk)};
}

std::size_t actuatedJointCount(const ik::KinematicChain& chain)
{
    std::size_t count = 0;
    for (const auto& joint : chain.joints)
        if (joint.variable) ++count;
    return count;
}

// Alternating lower + 5% of span, upper - 5% of span. Derived from the loaded
// model rather than hard-coded, so it stays correct if the URDF changes.
// Deliberately not the exact limits: this validates FK, not boundary handling.
JointConfiguration nearLimitConfiguration(const ik::KinematicChain& chain)
{
    JointConfiguration configuration;
    bool useLower = true;

    for (const auto& joint : chain.joints)
    {
        if (!joint.variable) continue;

        const double span = joint.limits.upper - joint.limits.lower;
        configuration.push_back(useLower ? joint.limits.lower + 0.05 * span
                                         : joint.limits.upper - 0.05 * span);
        useLower = !useLower;
    }
    return configuration;
}

void expectConfigurationMatchesReference(const LoadedRobot& robot,
                                         const JointConfiguration& configuration,
                                         std::string_view label)
{
    const auto symbolic =
        evaluateSymbolic(robot.fk, support::makeSymbolValues(robot.chain, configuration));
    if (!symbolic) return;

    const Matrix4 reference =
        toMatrix4(support::numericForwardKinematics(robot.chain, configuration));

    expectMatrixMatches(*symbolic, reference, label);
}

// --- synthetic chains -----------------------------------------------

ik::KinematicJoint makeJoint(ik::JointType type, ik::Vector3 axis,
                             ik::Vector3 translation, ik::Vector3 rpy,
                             std::string variableName = "q1")
{
    ik::KinematicJoint joint;
    joint.name = "j";
    joint.type = type;
    joint.axis = axis;
    joint.origin.translation = translation;
    joint.origin.rpy = rpy;
    if (type != ik::JointType::Fixed)
        joint.variable = ik::JointVariable{std::move(variableName), 1};
    return joint;
}

ik::KinematicChain makeChain(std::vector<ik::KinematicJoint> joints)
{
    ik::KinematicChain chain;
    chain.baseLink = "base";
    chain.toolLink = "tool";
    chain.joints = std::move(joints);
    return chain;
}

LoadedRobot buildSynthetic(std::vector<ik::KinematicJoint> joints)
{
    auto chain = makeChain(std::move(joints));
    const ik::JointTransformBuilder transformBuilder;
    const ik::ForwardKinematicsBuilder fkBuilder;
    auto fk = fkBuilder.build(chain, transformBuilder);
    return LoadedRobot{std::move(chain), std::move(fk)};
}

} // namespace

// --- KR4 against the quaternion reference ---------------------------

TEST(NumericFkValidationTest, EvaluatesKr4ZeroConfigurationAgainstNumericReference)
{
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());
    ASSERT_EQ(actuatedJointCount(robot->chain), 6u);

    expectConfigurationMatchesReference(*robot, JointConfiguration(6, 0.0), "kr4 zero");
}

TEST(NumericFkValidationTest, EvaluatesKr4SingleJointConfigurationsAgainstNumericReference)
{
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    // One joint at a time, so a fault in a single joint cannot be masked by
    // the others sitting at zero. +-0.25 is inside every limit of both robots.
    for (std::size_t index = 0; index < 6; ++index)
    {
        JointConfiguration configuration(6, 0.0);
        configuration[index] = (index % 2 == 0) ? 0.25 : -0.25;
        expectConfigurationMatchesReference(
            *robot, configuration, "kr4 single joint " + std::to_string(index + 1));
    }
}

TEST(NumericFkValidationTest, EvaluatesKr4MixedConfigurationAgainstNumericReference)
{
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    const JointConfiguration configuration{0.35, -0.45, 0.55, -0.65, 0.40, -0.30};
    expectConfigurationMatchesReference(*robot, configuration, "kr4 mixed");
}

TEST(NumericFkValidationTest, EvaluatesKr4NearLimitsAgainstNumericReference)
{
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    expectConfigurationMatchesReference(*robot, nearLimitConfiguration(robot->chain),
                                        "kr4 near limits");
}

// --- KR640 against the quaternion reference -------------------------

TEST(NumericFkValidationTest, EvaluatesKr640ZeroConfigurationAgainstNumericReference)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());
    ASSERT_EQ(actuatedJointCount(robot->chain), 6u);

    expectConfigurationMatchesReference(*robot, JointConfiguration(6, 0.0), "kr640 zero");
}

TEST(NumericFkValidationTest, EvaluatesKr640SingleJointConfigurationsAgainstNumericReference)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    for (std::size_t index = 0; index < 6; ++index)
    {
        JointConfiguration configuration(6, 0.0);
        configuration[index] = (index % 2 == 0) ? 0.25 : -0.25;
        expectConfigurationMatchesReference(
            *robot, configuration, "kr640 single joint " + std::to_string(index + 1));
    }
}

TEST(NumericFkValidationTest, EvaluatesKr640MixedConfigurationAgainstNumericReference)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    const JointConfiguration configuration{0.35, -0.45, 0.55, -0.65, 0.40, -0.30};
    expectConfigurationMatchesReference(*robot, configuration, "kr640 mixed");
}

TEST(NumericFkValidationTest, EvaluatesKr640NearLimitsAgainstNumericReference)
{
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    expectConfigurationMatchesReference(*robot, nearLimitConfiguration(robot->chain),
                                        "kr640 near limits");
}

// --- hand-computed oracles ------------------------------------------

TEST(NumericFkValidationTest, Kr640ZeroConfigurationMatchesHandComputedPose)
{
    // Every kr640 joint has rpy="0 0 0", so at q = 0 every transform is a pure
    // translation and the tool position is a plain sum read off the URDF:
    //
    //   x = 0.350 + 1.250                  = 1.600
    //   z = 0.750 + 1.150 + 0.145 + 0.290  = 2.335
    //
    // Independent of BOTH implementations -- this is the only kind of check
    // that can catch a shared misunderstanding rather than a coding slip.
    //
    // It does NOT test composition order: pure translations commute, so
    // reversing the whole chain would give the same answer. See the quarter
    // turn test below for that.
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    const auto symbolic = evaluateSymbolic(
        robot->fk, support::makeSymbolValues(robot->chain, JointConfiguration(6, 0.0)));
    ASSERT_TRUE(symbolic.has_value());

    Matrix4 expected{};
    expected[0][0] = expected[1][1] = expected[2][2] = expected[3][3] = 1.0;
    expected[0][3] = 1.600;
    expected[1][3] = 0.000;
    expected[2][3] = 2.335;

    expectMatrixMatches(*symbolic, expected, "kr640 zero (hand computed)");
}

TEST(NumericFkValidationTest, Kr640Joint1QuarterTurnMatchesHandComputedPose)
{
    // q1 = pi/2 turns everything downstream about +Z. Translations past a1 sum
    // to x = 0.350 + 1.250 = 1.600 and z = 1.150 + 0.145 + 0.290 = 1.585, so
    //
    //   p = (0, 0, 0.750) + Rz(pi/2) * (1.600, 0, 1.585) = (0, 1.600, 2.335)
    //
    // Unlike the zero pose this DOES pin composition order, the sign of the a1
    // axis, and translation propagation through an earlier rotation.
    const auto robot = loadRobot("kr640.urdf");
    ASSERT_TRUE(robot.has_value());

    JointConfiguration configuration(6, 0.0);
    configuration[0] = kPi / 2.0;

    const auto symbolic =
        evaluateSymbolic(robot->fk, support::makeSymbolValues(robot->chain, configuration));
    ASSERT_TRUE(symbolic.has_value());

    Matrix4 expected{};
    expected[0][0] = 0.0;  expected[0][1] = -1.0; expected[0][2] = 0.0; expected[0][3] = 0.000;
    expected[1][0] = 1.0;  expected[1][1] = 0.0;  expected[1][2] = 0.0; expected[1][3] = 1.600;
    expected[2][0] = 0.0;  expected[2][1] = 0.0;  expected[2][2] = 1.0; expected[2][3] = 2.335;
    expected[3][3] = 1.0;

    expectMatrixMatches(*symbolic, expected, "kr640 q1=pi/2 (hand computed)");
}

TEST(NumericFkValidationTest, EvaluatesNegativePrincipalAxisAgainstHandComputedPose)
{
    // Neither kr4 nor kr640 has a negative principal axis, so the negated
    // branch of JointTransformBuilder's fast path is never exercised by real
    // data. R(-Z, pi/2) = Rz(-pi/2).
    const auto robot = buildSynthetic(
        {makeJoint(ik::JointType::Revolute, {0.0, 0.0, -1.0}, {}, {}, "q1")});

    const auto symbolic =
        evaluateSymbolic(robot.fk, support::makeSymbolValues(robot.chain, {kPi / 2.0}));
    ASSERT_TRUE(symbolic.has_value());

    Matrix4 expected{};
    expected[0][0] =  0.0; expected[0][1] = 1.0; expected[0][2] = 0.0;
    expected[1][0] = -1.0; expected[1][1] = 0.0; expected[1][2] = 0.0;
    expected[2][2] =  1.0;
    expected[3][3] =  1.0;

    expectMatrixMatches(*symbolic, expected, "negative Z axis (hand computed)");
}

// --- synthetic, against the quaternion reference --------------------

TEST(NumericFkValidationTest, EvaluatesArbitraryAxisRevoluteAgainstQuaternionReference)
{
    // Both robots use principal axes only, so the general Rodrigues branch is
    // never reached by real data -- every real joint takes the fast path.
    const double scale = 1.0 / std::sqrt(14.0);
    const ik::Vector3 axis{1.0 * scale, 2.0 * scale, 3.0 * scale};

    const auto robot = buildSynthetic(
        {makeJoint(ik::JointType::Revolute, axis, {0.1, 0.2, 0.3}, {0.3, -0.2, 0.5}, "q1")});

    expectConfigurationMatchesReference(robot, {0.7}, "arbitrary axis revolute");
}

TEST(NumericFkValidationTest, EvaluatesRotatedPrismaticAgainstQuaternionReference)
{
    // No real robot in data/urdf has a prismatic joint, so without this the
    // branch stays numerically unvalidated. Origin turns about Z by pi/2 and
    // the slide is along X, so the displacement must land on +Y:
    //
    //   Rz(pi/2) * (0.3, 0, 0) = (0, 0.3, 0)
    //
    // The expectation is hand computed, not taken from the reference.
    const auto robot = buildSynthetic({makeJoint(
        ik::JointType::Prismatic, {1.0, 0.0, 0.0}, {}, {0.0, 0.0, kPi / 2.0}, "q1")});

    const auto symbolic =
        evaluateSymbolic(robot.fk, support::makeSymbolValues(robot.chain, {0.3}));
    ASSERT_TRUE(symbolic.has_value());

    Matrix4 expected{};
    expected[0][0] = 0.0; expected[0][1] = -1.0; expected[0][2] = 0.0; expected[0][3] = 0.0;
    expected[1][0] = 1.0; expected[1][1] =  0.0; expected[1][2] = 0.0; expected[1][3] = 0.3;
    expected[2][2] = 1.0; expected[2][3] = 0.0;
    expected[3][3] = 1.0;

    expectMatrixMatches(*symbolic, expected, "rotated prismatic (hand computed)");

    // Also cross-check against the quaternion reference, so the test covers
    // both oracles for this branch.
    const Matrix4 reference =
        toMatrix4(support::numericForwardKinematics(robot.chain, {0.3}));
    expectMatrixMatches(*symbolic, reference, "rotated prismatic (reference)");
}

// --- the reference itself -------------------------------------------

TEST(NumericFkValidationTest, RejectsDuplicateJointVariableNames)
{
    // The guarantee exists in makeSymbolValues; without this test nothing
    // pins it. A duplicate name would otherwise let the symbolic side
    // evaluate two joints from one binding -- invisible whenever the two
    // configuration values happen to be equal.
    const auto chain = makeChain({
        makeJoint(ik::JointType::Revolute, {0.0, 0.0, 1.0}, {}, {}, "q1"),
        makeJoint(ik::JointType::Revolute, {0.0, 1.0, 0.0}, {}, {}, "q1"),
    });

    EXPECT_THROW(support::makeSymbolValues(chain, {0.1, 0.2}), std::logic_error);
}

TEST(NumericFkValidationTest, NumericReferenceUsesCorrectRpyOrder)
{
    // rpy = (pi/2, 0, -pi/2), taken from joint_4 of kr4_r600.urdf: the two
    // conventions disagree here, whereas a single non-zero component would
    // give the same matrix either way and prove nothing.
    const Quaternion quaternion =
        support::fromRollPitchYaw(ik::Vector3{kPi / 2.0, 0.0, -kPi / 2.0});
    const Matrix3 rotation = support::toRotationMatrix(quaternion);

    // Rz(-pi/2) * Ry(0) * Rx(pi/2). The reversed order gives
    // {{0,1,0},{0,0,-1},{-1,0,0}}.
    const double expected[3][3] = {{0, 0, -1}, {-1, 0, 0}, {0, 1, 0}};

    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
        {
            SCOPED_TRACE(testing::Message() << "cell (" << row << ", " << column << ")");
            EXPECT_NEAR(rotation[row][column], expected[row][column], 1e-12);
        }
}

TEST(NumericFkValidationTest, NumericReferenceUsesCorrectCompositionOrder)
{
    // compose(a, b) must mean a * b: p = p_a + R_a * p_b.
    //
    // a = Rz(pi/2) with no translation, b = pure translation along X. The
    // correct order puts the result on +Y; the reversed one leaves it on +X.
    const RigidTransform a{support::fromAxisAngle(ik::Vector3{0.0, 0.0, 1.0}, kPi / 2.0),
                           {0.0, 0.0, 0.0}};
    const RigidTransform b{Quaternion{}, {1.0, 0.0, 0.0}};

    const RigidTransform composed = support::compose(a, b);

    EXPECT_NEAR(composed.translation.x, 0.0, 1e-12);
    EXPECT_NEAR(composed.translation.y, 1.0, 1e-12);
    EXPECT_NEAR(composed.translation.z, 0.0, 1e-12);
}

TEST(NumericFkValidationTest, NumericReferenceProducesProperRigidTransform)
{
    // A unit quaternion encodes a proper rotation mathematically, but in double
    // arithmetic the norm drifts and a wrong conversion can still produce a
    // non-orthogonal matrix. So the invariants are asserted, not assumed.
    const auto robot = loadRobot("kr4_r600.urdf");
    ASSERT_TRUE(robot.has_value());

    const JointConfiguration configuration{0.35, -0.45, 0.55, -0.65, 0.40, -0.30};
    const RigidTransform result =
        support::numericForwardKinematics(robot->chain, configuration);

    EXPECT_NEAR(support::norm(result.rotation), 1.0, 1e-12);

    const Matrix3 rotation = support::toRotationMatrix(result.rotation);

    // R^T * R == I
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
        {
            SCOPED_TRACE(testing::Message() << "R^T R (" << row << ", " << column << ")");
            double value = 0.0;
            for (std::size_t k = 0; k < 3; ++k)
                value += rotation[k][row] * rotation[k][column];
            EXPECT_NEAR(value, row == column ? 1.0 : 0.0, 1e-12);
        }

    // det(R) == +1, not -1: a reflection would satisfy R^T R = I too.
    const double determinant =
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1]) -
        rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0]) +
        rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0]);
    EXPECT_NEAR(determinant, 1.0, 1e-12);

    // The homogeneous last row, once embedded in 4x4.
    const Matrix4 embedded = toMatrix4(result);
    EXPECT_DOUBLE_EQ(embedded[3][0], 0.0);
    EXPECT_DOUBLE_EQ(embedded[3][1], 0.0);
    EXPECT_DOUBLE_EQ(embedded[3][2], 0.0);
    EXPECT_DOUBLE_EQ(embedded[3][3], 1.0);
}
