#include <gtest/gtest.h>

#include "ik_equations/model/TargetValidation.hpp"

#include <cmath>
#include <cstddef>
#include <limits>

using kinemaforge::ik::IkTarget;
using kinemaforge::ik::PoseTarget;
using kinemaforge::ik::PositionTarget;
using kinemaforge::ik::RotationMatrix3;
using kinemaforge::ik::TargetValidationErrorCode;
using kinemaforge::ik::Vector3;
using kinemaforge::ik::kOrientationTolerance;
using kinemaforge::ik::validate;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

RotationMatrix3 identity()
{
    RotationMatrix3 rotation{};
    rotation.values[0][0] = 1.0;
    rotation.values[1][1] = 1.0;
    rotation.values[2][2] = 1.0;
    return rotation;
}

RotationMatrix3 rotationAboutZ(double angle)
{
    RotationMatrix3 rotation{};
    rotation.values[0][0] = std::cos(angle);
    rotation.values[0][1] = -std::sin(angle);
    rotation.values[1][0] = std::sin(angle);
    rotation.values[1][1] = std::cos(angle);
    rotation.values[2][2] = 1.0;
    return rotation;
}

// Scaling the first axis by (1 + epsilon) makes (R^T R)(0,0) = (1 + epsilon)^2,
// so the orthogonality deviation is 2*epsilon to first order. That gives exact
// control over which side of kOrientationTolerance the matrix lands on, while
// keeping |det - 1| = epsilon, i.e. half the deviation -- so orthogonality is
// always the check that decides, as the documented order requires.
RotationMatrix3 withOrthogonalityDeviation(double deviation)
{
    RotationMatrix3 rotation = identity();
    rotation.values[0][0] = 1.0 + 0.5 * deviation;
    return rotation;
}

} // namespace

// --- position -------------------------------------------------------

TEST(TargetValidationTest, AcceptsFinitePositionTarget)
{
    EXPECT_TRUE(validate(PositionTarget{Vector3{1.6, 0.0, 2.335}}).has_value());
}

TEST(TargetValidationTest, RejectsNonFinitePositionTargetX)
{
    for (const double value : {kNaN, kInfinity, -kInfinity})
    {
        SCOPED_TRACE(testing::Message() << "x = " << value);
        const auto result = validate(PositionTarget{Vector3{value, 0.0, 0.0}});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFinitePosition);
        EXPECT_FALSE(result.error().message.empty());
    }
}

TEST(TargetValidationTest, RejectsNonFinitePositionTargetY)
{
    for (const double value : {kNaN, kInfinity, -kInfinity})
    {
        SCOPED_TRACE(testing::Message() << "y = " << value);
        const auto result = validate(PositionTarget{Vector3{0.0, value, 0.0}});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFinitePosition);
    }
}

TEST(TargetValidationTest, RejectsNonFinitePositionTargetZ)
{
    for (const double value : {kNaN, kInfinity, -kInfinity})
    {
        SCOPED_TRACE(testing::Message() << "z = " << value);
        const auto result = validate(PositionTarget{Vector3{0.0, 0.0, value}});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFinitePosition);
    }
}

// --- orientation ----------------------------------------------------

TEST(TargetValidationTest, AcceptsIdentityRotation)
{
    EXPECT_TRUE(validate(identity()).has_value());
}

TEST(TargetValidationTest, AcceptsValidRotationMatrix)
{
    EXPECT_TRUE(validate(rotationAboutZ(0.7)).has_value());
}

TEST(TargetValidationTest, RejectsNonFiniteRotationMatrix)
{
    // All nine cells, not one: a loop that skipped the first or last row would
    // otherwise pass while leaving a hole in the check.
    for (const double value : {kNaN, kInfinity, -kInfinity})
        for (std::size_t row = 0; row < 3; ++row)
            for (std::size_t column = 0; column < 3; ++column)
            {
                SCOPED_TRACE(testing::Message() << "value = " << value << " at (" << row
                                                << ", " << column << ")");
                RotationMatrix3 rotation = identity();
                rotation.values[row][column] = value;

                const auto result = validate(rotation);
                ASSERT_FALSE(result.has_value());
                EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFiniteOrientation);
            }
}

TEST(TargetValidationTest, RejectsNonOrthogonalRotationMatrix)
{
    // diag(1, 1, 0.5): scaling. It also has det != 1, but orthogonality is
    // checked first, so THIS is the code it must produce.
    RotationMatrix3 rotation = identity();
    rotation.values[2][2] = 0.5;

    const auto result = validate(rotation);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonOrthogonalOrientation);
}

TEST(TargetValidationTest, RejectsInvalidOrientationDeterminant)
{
    {
        // diag(1, 1, -1): a reflection. Exactly orthogonal, so it reaches the
        // determinant check.
        SCOPED_TRACE("reflection, det = -1");
        RotationMatrix3 rotation = identity();
        rotation.values[2][2] = -1.0;

        const auto result = validate(rotation);
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code,
                  TargetValidationErrorCode::InvalidOrientationDeterminant);
    }
    {
        // The case that makes "reflection" the wrong name: a uniform scaling
        // by (1 + 4e-9) deviates from orthogonality by ~8e-9 (inside the
        // tolerance) and from unit determinant by ~1.2e-8 (outside it), with a
        // POSITIVE determinant.
        SCOPED_TRACE("uniform scaling, det > 1 but not a reflection");
        RotationMatrix3 rotation{};
        const double scale = 1.0 + 4e-9;
        rotation.values[0][0] = scale;
        rotation.values[1][1] = scale;
        rotation.values[2][2] = scale;

        const auto result = validate(rotation);
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code,
                  TargetValidationErrorCode::InvalidOrientationDeterminant);
    }
}

TEST(TargetValidationTest, AcceptsRotationJustWithinTolerance)
{
    // Deviations far from the threshold would not pin it: any implementation
    // using anything between them would pass. These two sit on either side.
    EXPECT_TRUE(validate(withOrthogonalityDeviation(0.9 * kOrientationTolerance)).has_value());
}

TEST(TargetValidationTest, RejectsRotationJustBeyondTolerance)
{
    const auto result = validate(withOrthogonalityDeviation(1.1 * kOrientationTolerance));

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonOrthogonalOrientation);
}

// --- pose -----------------------------------------------------------

TEST(TargetValidationTest, AcceptsValidPoseTarget)
{
    EXPECT_TRUE(validate(PoseTarget{Vector3{1.6, 0.0, 2.335}, rotationAboutZ(0.7)}).has_value());
}

TEST(TargetValidationTest, RejectsPoseTargetWithNonFinitePosition)
{
    const auto result = validate(PoseTarget{Vector3{0.0, kNaN, 0.0}, identity()});

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFinitePosition);
}

TEST(TargetValidationTest, RejectsPoseTargetWithInvalidOrientation)
{
    RotationMatrix3 reflection = identity();
    reflection.values[2][2] = -1.0;

    const auto result = validate(PoseTarget{Vector3{1.0, 0.0, 0.0}, reflection});

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::InvalidOrientationDeterminant);
}

// --- IkTarget dispatcher --------------------------------------------

TEST(TargetValidationTest, ValidatesPositionTargetThroughIkTarget)
{
    EXPECT_TRUE(validate(IkTarget{PositionTarget{Vector3{1.0, 2.0, 3.0}}}).has_value());

    const auto result = validate(IkTarget{PositionTarget{Vector3{kNaN, 0.0, 0.0}}});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFinitePosition);
}

TEST(TargetValidationTest, ValidatesPoseTargetThroughIkTarget)
{
    EXPECT_TRUE(
        validate(IkTarget{PoseTarget{Vector3{1.0, 0.0, 0.0}, rotationAboutZ(0.3)}}).has_value());

    RotationMatrix3 scaling = identity();
    scaling.values[0][0] = 0.5;

    const auto result = validate(IkTarget{PoseTarget{Vector3{}, scaling}});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonOrthogonalOrientation);
}
