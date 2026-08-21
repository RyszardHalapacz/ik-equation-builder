#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ConstraintBuilder.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/model/Quaternion.hpp"
#include "ik_equations/model/QuaternionTarget.hpp"
#include "ik_equations/model/TargetValidation.hpp"
#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"
#include "support/NumericForwardKinematics.hpp"
#include "support/TransformComparison.hpp"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <utility>

namespace ik = kinemaforge::ik;
namespace support = kinemaforge::testsupport;

using ik::ConstraintBuilder;
using ik::IkTaskKind;
using ik::PoseTarget;
using ik::Quaternion;
using ik::RotationMatrix3;
using ik::TargetValidationErrorCode;
using ik::Vector3;
using ik::makePoseTargetFromQuaternion;
using ik::sameNode;
using ik::sameSource;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kPi = 3.14159265358979323846;

// The bound for these comparisons is 1e-12, the same one the FK cross-check is
// held to. Measured deviations sit four orders below it -- 1.11e-15 against
// Rodrigues and 1.78e-15 for a rotated vector -- so there is nothing here to
// flicker.
constexpr double kTolerance = 1e-12;

Quaternion axisAngleQuaternion(double x, double y, double z, double angle)
{
    const double length = std::sqrt(x * x + y * y + z * z);
    const double half = 0.5 * angle;
    const double sine = std::sin(half) / length;
    return Quaternion{std::cos(half), x * sine, y * sine, z * sine};
}

// Rodrigues: R = I + sin(t) K + (1 - cos(t)) K^2, with K = [k]_x and k a unit
// axis.
//
// This is the SECOND path, and its value lies in not going through quaternions
// at all. A swapped (w,x,y,z)/(x,y,z,w) convention, or a conjugate in place of
// the quaternion, is self-consistent inside the quaternion world -- only a
// comparison against the trigonometric route exposes it.
RotationMatrix3 rodrigues(double x, double y, double z, double angle)
{
    const double length = std::sqrt(x * x + y * y + z * z);
    const double kx = x / length;
    const double ky = y / length;
    const double kz = z / length;

    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double t = 1.0 - c;

    return RotationMatrix3{{{{c + kx * kx * t,      kx * ky * t - kz * s, kx * kz * t + ky * s},
                             {ky * kx * t + kz * s, c + ky * ky * t,      ky * kz * t - kx * s},
                             {kz * kx * t - ky * s, kz * ky * t + kx * s, c + kz * kz * t}}}};
}

void expectRotationsMatch(const RotationMatrix3& actual, const RotationMatrix3& expected,
                          double tolerance)
{
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            EXPECT_NEAR(actual.values[row][column], expected.values[row][column], tolerance)
                << "cell (" << row << ", " << column << ")";
}

struct LoadedRobot
{
    ik::KinematicChain chain;
    ik::SymbolicTransform fk;
};

// A copy of the helper in test_constraint_builder.cpp, deliberately: two
// translation units, no shared test header outside support/, and neither file
// gets to change the other by accident.
std::optional<LoadedRobot> loadRobot(const char* fileName)
{
    ik::UrdfModelLoader loader;
    const auto robot = loader.load(std::filesystem::path{KINEMAFORGE_URDF_DATA_DIR} / fileName);

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

} // namespace

// --- shape and values -----------------------------------------------

TEST(QuaternionTargetTest, AcceptsValidQuaternionTarget)
{
    const auto target = makePoseTargetFromQuaternion(
        Vector3{1.6, -0.4, 2.1}, axisAngleQuaternion(1.0, -2.0, 3.0, 0.7));

    ASSERT_TRUE(target.has_value());
    EXPECT_DOUBLE_EQ(target->position.x, 1.6);
    EXPECT_DOUBLE_EQ(target->position.y, -0.4);
    EXPECT_DOUBLE_EQ(target->position.z, 2.1);
    EXPECT_TRUE(ik::validate(*target).has_value());
}

TEST(QuaternionTargetTest, IdentityQuaternionProducesIdentityRotation)
{
    const auto target = makePoseTargetFromQuaternion(Vector3{}, Quaternion{});
    ASSERT_TRUE(target.has_value());

    // No tolerance: for q = (1,0,0,0) every product in the formula is zero and
    // 1 - 2*0 is exactly 1. An EXPECT_NEAR here would let through an
    // implementation that computes something merely almost-identity.
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            EXPECT_DOUBLE_EQ(target->orientation.values[row][column], row == column ? 1.0 : 0.0);
}

TEST(QuaternionTargetTest, ProducesEquivalentTargetsForQuaternionAndMatrix)
{
    // The "equivalent matrix" must NOT be the output of the same conversion, or
    // the test is a tautology. It is built independently, from Rodrigues, out of
    // the same (axis, angle).
    const double axis[3] = {1.0, -2.0, 3.0};
    const double angle = 0.7;
    const Vector3 position{0.2, 0.3, 0.4};

    const auto fromQuaternion = makePoseTargetFromQuaternion(
        position, axisAngleQuaternion(axis[0], axis[1], axis[2], angle));
    ASSERT_TRUE(fromQuaternion.has_value());

    const PoseTarget fromMatrix{position, rodrigues(axis[0], axis[1], axis[2], angle)};

    expectRotationsMatch(fromQuaternion->orientation, fromMatrix.orientation, kTolerance);
    EXPECT_DOUBLE_EQ(fromQuaternion->position.x, fromMatrix.position.x);
    EXPECT_DOUBLE_EQ(fromQuaternion->position.y, fromMatrix.position.y);
    EXPECT_DOUBLE_EQ(fromQuaternion->position.z, fromMatrix.position.z);
}

TEST(QuaternionTargetTest, AgreesWithRodriguesRotationFormula)
{
    struct Case
    {
        double x;
        double y;
        double z;
        double angle;
        const char* label;
    };

    for (const Case& testCase : {Case{0.0, 0.0, 1.0, 0.5 * kPi, "quarter turn about z"},
                                 Case{1.0, 0.0, 0.0, -0.25 * kPi, "negative, about x"},
                                 Case{1.0, -2.0, 3.0, 0.7, "skew axis"},
                                 Case{1.0, 1.0, 1.0, kPi - 1e-6, "just short of pi"},
                                 Case{0.0, 1.0, 0.0, 1e-9, "near identity"}})
    {
        SCOPED_TRACE(testCase.label);

        const auto target = makePoseTargetFromQuaternion(
            Vector3{}, axisAngleQuaternion(testCase.x, testCase.y, testCase.z, testCase.angle));
        ASSERT_TRUE(target.has_value());

        expectRotationsMatch(target->orientation,
                             rodrigues(testCase.x, testCase.y, testCase.z, testCase.angle),
                             kTolerance);
    }
}

TEST(QuaternionTargetTest, MatchesQuaternionRotationWithoutFormingAMatrix)
{
    // The third path: the oracle rotates a vector with a quaternion sandwich and
    // NEVER builds a matrix. What is compared is behaviour, not notation --
    // R*v against rotate().
    const Quaternion quaternion = axisAngleQuaternion(1.0, -2.0, 3.0, 0.7);

    const auto target = makePoseTargetFromQuaternion(Vector3{}, quaternion);
    ASSERT_TRUE(target.has_value());

    const support::Quaternion oracleQuaternion{quaternion.w, quaternion.x,
                                               quaternion.y, quaternion.z};

    for (const support::Vector3d& vector : {support::Vector3d{1.0, 0.0, 0.0},
                                            support::Vector3d{0.0, 1.0, 0.0},
                                            support::Vector3d{0.0, 0.0, 1.0},
                                            support::Vector3d{0.37, -1.4, 0.9}})
    {
        SCOPED_TRACE(testing::Message()
                     << "v = (" << vector.x << ", " << vector.y << ", " << vector.z << ")");

        const support::Vector3d expected = support::rotate(oracleQuaternion, vector);
        const auto& m = target->orientation.values;

        EXPECT_NEAR(m[0][0] * vector.x + m[0][1] * vector.y + m[0][2] * vector.z,
                    expected.x, kTolerance);
        EXPECT_NEAR(m[1][0] * vector.x + m[1][1] * vector.y + m[1][2] * vector.z,
                    expected.y, kTolerance);
        EXPECT_NEAR(m[2][0] * vector.x + m[2][1] * vector.y + m[2][2] * vector.z,
                    expected.z, kTolerance);
    }
}

TEST(QuaternionTargetTest, NegatedQuaternionProducesSameTarget)
{
    const Quaternion quaternion = axisAngleQuaternion(1.0, -2.0, 3.0, 0.7);
    const Quaternion negated{-quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z};

    const auto first = makePoseTargetFromQuaternion(Vector3{}, quaternion);
    const auto second = makePoseTargetFromQuaternion(Vector3{}, negated);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());

    // EXPECT_EQ, not EXPECT_NEAR: the formula is quadratic, so the sign cancels
    // in every product, and both the norm and the division are identical for -q.
    // The values are exactly equal -- not merely close -- and a weaker assertion
    // would let through an implementation that drops a w factor.
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            EXPECT_EQ(first->orientation.values[row][column],
                      second->orientation.values[row][column])
                << "cell (" << row << ", " << column << ")";
}

// --- rejections and the property ------------------------------------

TEST(QuaternionTargetTest, RejectsNonNormalizedQuaternionTarget)
{
    const Quaternion quaternion = axisAngleQuaternion(1.0, -2.0, 3.0, 0.7);
    const auto target = makePoseTargetFromQuaternion(
        Vector3{}, Quaternion{quaternion.w * 1.5, quaternion.x * 1.5,
                              quaternion.y * 1.5, quaternion.z * 1.5});

    ASSERT_FALSE(target.has_value());
    EXPECT_EQ(target.error().code, TargetValidationErrorCode::NonNormalizedQuaternion);
    EXPECT_FALSE(target.error().message.empty());
}

TEST(QuaternionTargetTest, RejectsNonFinitePositionInQuaternionTarget)
{
    // Position is validated first, and that is observable: the quaternion here
    // is broken too, yet the reported error is about the position.
    const auto target = makePoseTargetFromQuaternion(Vector3{kNaN, 0.0, 0.0},
                                                     Quaternion{0.0, 0.0, 0.0, 0.0});

    ASSERT_FALSE(target.has_value());
    EXPECT_EQ(target.error().code, TargetValidationErrorCode::NonFinitePosition);
}

TEST(QuaternionTargetTest, ProducedOrientationAlwaysPassesTargetValidation)
{
    // The property the whole boundary design rests on: a target leaving this
    // factory must pass the check ConstraintBuilder is about to run. Without
    // normalisation, a quaternion at the edge of the band gives
    // |R^T R - I| = 8e-8, eight times outside kOrientationTolerance -- and this
    // test fails.
    //
    // Fixed seed, so the run is reproducible; the input is rounded to nine
    // decimal places, i.e. to the documented input contract, whose worst
    // measured norm error is 9.03e-10.
    std::mt19937_64 generator{20260821};
    std::normal_distribution<double> gaussian{0.0, 1.0};

    for (int sample = 0; sample < 200; ++sample)
    {
        SCOPED_TRACE(testing::Message() << "sample " << sample);

        double raw[4] = {gaussian(generator), gaussian(generator),
                         gaussian(generator), gaussian(generator)};
        const double norm = std::sqrt(raw[0] * raw[0] + raw[1] * raw[1]
                                      + raw[2] * raw[2] + raw[3] * raw[3]);
        ASSERT_GT(norm, 1e-6);

        for (double& component : raw)
            component = std::round(component / norm * 1e9) / 1e9;

        const auto target = makePoseTargetFromQuaternion(
            Vector3{}, Quaternion{raw[0], raw[1], raw[2], raw[3]});

        ASSERT_TRUE(target.has_value()) << target.error().message;
        EXPECT_TRUE(ik::validate(target->orientation).has_value());
    }
}

// --- end to end: two independent paths ------------------------------

namespace {

// Both paths start from the same configuration and never meet until the
// constraint builder:
//
//                          configuration q
//                        /                \.
//   symbolic FK -> evaluate -> R          numeric oracle -> q_oracle
//            |                                     |
//   PoseTarget{position, R}       makePoseTargetFromQuaternion(position, q_oracle)
//            \                                     /
//              ConstraintBuilder -> two IkEquationSystem -> compare
//
// The left side runs on matrices through the whole symbolic layer, the right on
// quaternions through an independently written FK. Agreement across the twelve
// right-hand sides therefore means the new conversion agrees with a second
// implementation, not with itself.
//
// The oracle never normalises its quaternion along the way, and it does not need
// to: composing fourteen unit quaternions (seven joints, origin plus motion
// each) moves the norm by at most 1.55e-15 over 200 000 trials, six million
// times inside the acceptance band. If it ever drifted out, this test would
// report NonNormalizedQuaternion -- a real signal, not a flake.
void expectSystemsMatch(const char* fileName, const support::JointConfiguration& configuration)
{
    const auto robot = loadRobot(fileName);
    ASSERT_TRUE(robot.has_value());

    const auto values = support::makeSymbolValues(robot->chain, configuration);

    const auto evaluated = support::evaluateSymbolic(robot->fk, values);
    ASSERT_TRUE(evaluated.has_value());

    RotationMatrix3 rotation{};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            rotation.values[row][column] = (*evaluated)[row][column];

    const Vector3 position{(*evaluated)[0][3], (*evaluated)[1][3], (*evaluated)[2][3]};

    const support::RigidTransform reference =
        support::numericForwardKinematics(robot->chain, configuration);
    const Quaternion quaternion{reference.rotation.w, reference.rotation.x,
                                reference.rotation.y, reference.rotation.z};

    const auto fromQuaternion = makePoseTargetFromQuaternion(position, quaternion);
    ASSERT_TRUE(fromQuaternion.has_value()) << fromQuaternion.error().message;

    const ConstraintBuilder builder;
    const auto matrixSystem = builder.build(robot->fk, robot->chain,
                                            PoseTarget{position, rotation});
    const auto quaternionSystem = builder.build(robot->fk, robot->chain, *fromQuaternion);

    ASSERT_TRUE(matrixSystem.has_value());
    ASSERT_TRUE(quaternionSystem.has_value());

    EXPECT_EQ(quaternionSystem->taskKind(), IkTaskKind::Pose);
    ASSERT_EQ(quaternionSystem->equations().size(), 12u);
    ASSERT_EQ(matrixSystem->equations().size(), 12u);
    EXPECT_EQ(quaternionSystem->unknowns().size(), matrixSystem->unknowns().size());

    for (std::size_t index = 0; index < 12; ++index)
    {
        SCOPED_TRACE(testing::Message() << "equation " << index);

        const auto& fromMatrix = matrixSystem->equations()[index];
        const auto& fromQuat = quaternionSystem->equations()[index];

        EXPECT_TRUE(sameNode(fromQuat.lhs, fromMatrix.lhs));
        EXPECT_TRUE(sameSource(fromQuat.source, fromMatrix.source));
        EXPECT_NEAR(ik::constantValue(fromQuat.rhs), ik::constantValue(fromMatrix.rhs),
                    kTolerance);
    }
}

} // namespace

TEST(QuaternionTargetTest, BuildsKr640FullPoseSystemFromQuaternion)
{
    expectSystemsMatch("kr640.urdf",
                       support::JointConfiguration{0.3, -0.7, 1.1, 0.2, -0.5, 0.9});
}

TEST(QuaternionTargetTest, BuildsKr4FullPoseSystemFromQuaternion)
{
    // The second robot is not decoration: kr4_r600 carries non-zero rpy in its
    // joint origins, so its FK matrix has no zero cells for a mis-indexed cell
    // to hide behind.
    expectSystemsMatch("kr4_r600.urdf",
                       support::JointConfiguration{-0.2, 0.4, -0.9, 1.2, 0.6, -1.1});
}
