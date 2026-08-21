#include <gtest/gtest.h>

#include "ik_equations/model/TargetValidation.hpp"

#include "ik_equations/model/Quaternion.hpp"
#include "ik_equations/model/ToolAxis.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <string>

using kinemaforge::ik::IkTarget;
using kinemaforge::ik::PoseTarget;
using kinemaforge::ik::PositionTarget;
using kinemaforge::ik::Quaternion;
using kinemaforge::ik::DirectionAngleTarget;
using kinemaforge::ik::IkTarget;
using kinemaforge::ik::PositionAndDirectionAngleTarget;
using kinemaforge::ik::PositionAndToolDirectionTarget;
using kinemaforge::ik::RotationMatrix3;
using kinemaforge::ik::ToolAxis;
using kinemaforge::ik::ToolDirectionTarget;
using kinemaforge::ik::TargetValidationErrorCode;
using kinemaforge::ik::Vector3;
using kinemaforge::ik::kOrientationTolerance;
using kinemaforge::ik::kQuaternionNormTolerance;
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

Quaternion axisAngleQuaternion(double x, double y, double z, double angle)
{
    const double length = std::sqrt(x * x + y * y + z * z);
    const double half = 0.5 * angle;
    const double sine = std::sin(half) / length;
    return Quaternion{std::cos(half), x * sine, y * sine, z * sine};
}

// A skew axis: no component is zero or equal to another, so a dropped or
// swapped component has nowhere to hide.
Quaternion skewQuaternion()
{
    return axisAngleQuaternion(1.0, -2.0, 3.0, 0.7);
}

Quaternion scaled(const Quaternion& quaternion, double factor)
{
    return Quaternion{quaternion.w * factor, quaternion.x * factor,
                      quaternion.y * factor, quaternion.z * factor};
}

Quaternion withComponent(std::size_t index, double value)
{
    Quaternion quaternion{};   // identity
    double* const components[4] = {&quaternion.w, &quaternion.x,
                                   &quaternion.y, &quaternion.z};
    *components[index] = value;
    return quaternion;
}

Quaternion throughFloat(const Quaternion& quaternion)
{
    return Quaternion{static_cast<float>(quaternion.w), static_cast<float>(quaternion.x),
                      static_cast<float>(quaternion.y), static_cast<float>(quaternion.z)};
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


// --- quaternion -----------------------------------------------------

TEST(TargetValidationTest, AcceptsUnitQuaternion)
{
    EXPECT_TRUE(validate(Quaternion{}).has_value());   // identity
    EXPECT_TRUE(validate(skewQuaternion()).has_value());
}

TEST(TargetValidationTest, AcceptsNegatedUnitQuaternion)
{
    EXPECT_TRUE(validate(scaled(skewQuaternion(), -1.0)).has_value());
}

TEST(TargetValidationTest, RejectsNonFiniteQuaternion)
{
    // THIS is the test that pins the ORDER of the steps, not the NaN one below.
    // For inf the norm is inf too, and "inf > tolerance" is TRUE -- so a
    // validation that checked the norm first would return
    // NonNormalizedQuaternion here and this EXPECT_EQ would fail. The error code
    // would be claiming "not normalised" about data that are not numbers.
    for (const double value : {kInfinity, -kInfinity})
        for (std::size_t index = 0; index < 4; ++index)
        {
            SCOPED_TRACE(testing::Message() << "component " << index << " = " << value);
            const auto result = validate(withComponent(index, value));
            ASSERT_FALSE(result.has_value());
            EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFiniteQuaternion);
            EXPECT_FALSE(result.error().message.empty());
        }
}

TEST(TargetValidationTest, RejectsNaNQuaternionInsteadOfAcceptingIt)
{
    // This one pins that NaN is detected EXPLICITLY. Validation without a
    // separate finiteness step would compute a NaN norm, and "NaN > tolerance"
    // is FALSE -- so it would accept NaN as a unit quaternion and this test
    // would flip to a pass. The same shape of bug the 2026-08-17 review found in
    // within_limits().
    //
    // What it does NOT pin: the order of the steps. An implementation computing
    // the norm first but still running the finiteness check before returning
    // would also report NonFiniteQuaternion here, because the comparison against
    // NaN is false and simply falls through. The order is pinned by the inf case
    // above.
    for (std::size_t index = 0; index < 4; ++index)
    {
        SCOPED_TRACE(testing::Message() << "component " << index << " = NaN");
        const auto result = validate(withComponent(index, kNaN));
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFiniteQuaternion);
    }
}

TEST(TargetValidationTest, RejectsZeroQuaternion)
{
    const auto result = validate(Quaternion{0.0, 0.0, 0.0, 0.0});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonNormalizedQuaternion);
}

TEST(TargetValidationTest, RejectsNonNormalizedQuaternion)
{
    const auto result = validate(scaled(skewQuaternion(), 1.5));
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonNormalizedQuaternion);
    EXPECT_NE(result.error().message.find("1.5"), std::string::npos)
        << result.error().message;
}

TEST(TargetValidationTest, AcceptsQuaternionAtNormToleranceEdge)
{
    EXPECT_TRUE(validate(scaled(skewQuaternion(), 1.0 + 0.9 * kQuaternionNormTolerance))
                    .has_value());
}

TEST(TargetValidationTest, RejectsQuaternionJustOutsideNormTolerance)
{
    const auto result = validate(scaled(skewQuaternion(), 1.0 + 1.1 * kQuaternionNormTolerance));
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonNormalizedQuaternion);
}

TEST(TargetValidationTest, QuaternionErrorMessageCarriesFullPrecision)
{
    // std::to_string prints six decimals, so a norm of 1 + 1e-7 -- the very
    // reason for the rejection -- would render as "1.000000". Same problem as
    // the determinant, same fix.
    //
    // Checked by ROUND TRIP, not by substring. A substring assertion would test
    // someone else's formatting: for a scaled skew quaternion the message reads
    // "||q|| = 1.0000000999999998", and an expected substring like "1.0000001"
    // does not appear in it even though the contract is met perfectly. What
    // max_digits10 guarantees is that the value can be READ BACK without loss.
    //
    // An axis-aligned quaternion, so the expected value is sqrt(w*w) -- one
    // term, so the test does not restate the production norm formula.
    const Quaternion quaternion{1.0000001, 0.0, 0.0, 0.0};

    const auto result = validate(quaternion);
    ASSERT_FALSE(result.has_value());
    ASSERT_EQ(result.error().code, TargetValidationErrorCode::NonNormalizedQuaternion);

    const std::string marker = "||q|| = ";
    const auto position = result.error().message.find(marker);
    ASSERT_NE(position, std::string::npos) << result.error().message;

    const double reported = std::stod(result.error().message.substr(position + marker.size()));

    EXPECT_DOUBLE_EQ(reported, std::sqrt(quaternion.w * quaternion.w));

    // And this is the actual reason the test exists: the message has to tell the
    // rejected norm apart from one. A to_string rendering ("1.000000") would read
    // back as exactly 1.0, and this assertion would catch it.
    EXPECT_NE(reported, 1.0);
}

TEST(TargetValidationTest, AcceptsQuaternionThatCameThroughFloat)
{
    // Pins the contract, not a numeric behaviour: the norm threshold is NOT a
    // detector of where the data came from, and has no way to be one. Values
    // that are exactly representable in float come back with zero norm error, so
    // they always pass -- if anyone later "fixes" validation to reject
    // float-derived data, this test breaks first.
    //
    // A general quaternion through float is deliberately NOT asserted here: it
    // passes in 41.65% of cases (1 000 000 samples), so an expectation either way
    // would be a flaky test by construction.
    EXPECT_TRUE(validate(throughFloat(Quaternion{})).has_value());
    EXPECT_TRUE(validate(throughFloat(Quaternion{0.5, 0.5, 0.5, 0.5})).has_value());
}


// --- direction and tool axis ----------------------------------------

TEST(TargetValidationTest, AcceptsUnnormalizedDirection)
{
    // A direction is a ray: length carries no semantics, so anything finite and
    // non-zero names a direction. Contrast with a quaternion, where unit length
    // is the caller's declaration and a norm of 1.7 proves a bug.
    for (const Vector3& direction : {Vector3{0.0, 0.0, 1.0}, Vector3{0.0, 0.0, 5.0},
                                     Vector3{0.0, 0.0, 0.2}, Vector3{1e200, 1e200, 0.0},
                                     Vector3{1e-200, 1e-200, 0.0}})
    {
        SCOPED_TRACE(testing::Message() << "z = " << direction.z);
        EXPECT_TRUE(validate(ToolDirectionTarget{direction, ToolAxis::Z}).has_value());
    }
}

TEST(TargetValidationTest, RejectsNonFiniteDirection)
{
    for (const double value : {kNaN, kInfinity, -kInfinity})
        for (std::size_t index = 0; index < 3; ++index)
        {
            SCOPED_TRACE(testing::Message() << "component " << index << " = " << value);

            Vector3 direction{0.0, 0.0, 1.0};
            double* const components[3] = {&direction.x, &direction.y, &direction.z};
            *components[index] = value;

            const auto result = validate(ToolDirectionTarget{direction, ToolAxis::Z});
            ASSERT_FALSE(result.has_value());
            EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFiniteDirection);
        }

    // Priority, on a target broken twice over. The declared order is
    // NonFiniteDirection -> ZeroDirection -> InvalidToolAxis, and testing each
    // code in isolation does not prove it.
    {
        SCOPED_TRACE("non-finite direction and forged axis");
        const auto result = validate(
            ToolDirectionTarget{Vector3{kNaN, 0.0, 0.0}, static_cast<ToolAxis>(99)});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFiniteDirection);
    }
    {
        SCOPED_TRACE("zero direction and forged axis");
        const auto result = validate(
            ToolDirectionTarget{Vector3{0.0, 0.0, 0.0}, static_cast<ToolAxis>(99)});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::ZeroDirection);
    }
}

TEST(TargetValidationTest, RejectsZeroDirection)
{
    const auto result = validate(ToolDirectionTarget{Vector3{0.0, 0.0, 0.0}, ToolAxis::Z});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::ZeroDirection);
    EXPECT_FALSE(result.error().message.empty());
}

TEST(TargetValidationTest, RejectsInvalidToolAxisInTarget)
{
    // enum class stops a new enumerator from being ignored; it does not stop a
    // value forged with static_cast. The model defends CartesianComponent the
    // same way.
    const auto result = validate(
        ToolDirectionTarget{Vector3{0.0, 0.0, 1.0}, static_cast<ToolAxis>(99)});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::InvalidToolAxis);

    // Position first, exactly as in a pose target: a target broken in both
    // halves reports the position.
    const auto both = validate(PositionAndToolDirectionTarget{
        Vector3{kNaN, 0.0, 0.0}, Vector3{0.0, 0.0, 0.0}, ToolAxis::Z});
    ASSERT_FALSE(both.has_value());
    EXPECT_EQ(both.error().code, TargetValidationErrorCode::NonFinitePosition);
}


// --- angle to a reference direction ---------------------------------

TEST(TargetValidationTest, AcceptsAngleInsideOpenInterval)
{
    const double pi = std::numbers::pi_v<double>;

    for (const double angle : {0.3, 0.5 * pi, pi - 0.3})
    {
        SCOPED_TRACE(testing::Message() << "angle = " << angle);
        EXPECT_TRUE(validate(DirectionAngleTarget{Vector3{0.0, 0.0, 1.0}, angle}).has_value());
    }
}

TEST(TargetValidationTest, RejectsAngleOutsideRange)
{
    const double pi = std::numbers::pi_v<double>;

    for (const double angle : {-0.1, pi + 0.1, 3.0 * pi})
    {
        SCOPED_TRACE(testing::Message() << "angle = " << angle);

        const auto result = validate(DirectionAngleTarget{Vector3{0.0, 0.0, 1.0}, angle});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::AngleOutOfRange);
    }

    for (const double angle : {kNaN, kInfinity, -kInfinity})
    {
        SCOPED_TRACE(testing::Message() << "angle = " << angle);

        const auto result = validate(DirectionAngleTarget{Vector3{0.0, 0.0, 1.0}, angle});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFiniteAngle);
    }
}

TEST(TargetValidationTest, RejectsDegenerateAngle)
{
    const double pi = std::numbers::pi_v<double>;
    const Vector3 reference{0.0, 0.0, 1.0};

    {
        // The mathematical endpoints. Portable: no libm involved.
        SCOPED_TRACE("exact endpoints");
        for (const double angle : {0.0, pi})
        {
            const auto result = validate(DirectionAngleTarget{reference, angle});
            ASSERT_FALSE(result.has_value());
            EXPECT_EQ(result.error().code, TargetValidationErrorCode::DegenerateAngle);
        }
    }
    {
        // The numerical collapse. The angle at which cos() reaches exactly 1 is a
        // property of this libm, not a published tolerance -- so the test SEARCHES
        // for it instead of assuming a value. The iteration cap is a diagnostic,
        // not a bound: an unusual libm should fail here loudly rather than spin.
        SCOPED_TRACE("numerically collapsed onto 1");

        double angle = 1e-3;
        int steps = 0;
        while (steps < 200 && angle > 0.0 && std::abs(std::cos(angle)) != 1.0)
        {
            angle *= 0.5;
            ++steps;
        }

        ASSERT_LT(steps, 200) << "cos() never collapsed onto 1 -- unexpected libm";
        ASSERT_GT(angle, 0.0);
        ASSERT_LT(angle, pi);
        ASSERT_EQ(std::abs(std::cos(angle)), 1.0);

        const auto result = validate(DirectionAngleTarget{reference, angle});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::DegenerateAngle);
    }
    {
        SCOPED_TRACE("numerically collapsed onto -1");

        double delta = 1e-3;
        int steps = 0;
        while (steps < 200 && delta > 0.0 && std::abs(std::cos(pi - delta)) != 1.0)
        {
            delta *= 0.5;
            ++steps;
        }

        ASSERT_LT(steps, 200) << "cos() never collapsed onto -1 -- unexpected libm";
        ASSERT_GT(delta, 0.0);
        // Otherwise this would test the exact endpoint, which the block above
        // already covers -- and for a different reason.
        ASSERT_NE(pi - delta, pi);
        ASSERT_EQ(std::abs(std::cos(pi - delta)), 1.0);

        const auto result = validate(DirectionAngleTarget{reference, pi - delta});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::DegenerateAngle);
    }
    {
        // Priority: a broken reference outranks a broken angle.
        SCOPED_TRACE("zero reference and degenerate angle");
        const auto result = validate(DirectionAngleTarget{Vector3{0.0, 0.0, 0.0}, 0.0});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::ZeroDirection);
    }
    {
        SCOPED_TRACE("forged tool axis");
        const auto result = validate(
            DirectionAngleTarget{reference, 0.7, static_cast<ToolAxis>(99)});
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::InvalidToolAxis);
    }
    {
        // Compilation forces TargetVisitor to have an overload; it does not force
        // that overload to route anywhere sensible. This does.
        SCOPED_TRACE("through the IkTarget variant");
        const auto accepted = validate(IkTarget{DirectionAngleTarget{reference, 0.7}});
        EXPECT_TRUE(accepted.has_value());

        const auto rejected = validate(IkTarget{DirectionAngleTarget{reference, 0.0}});
        ASSERT_FALSE(rejected.has_value());
        EXPECT_EQ(rejected.error().code, TargetValidationErrorCode::DegenerateAngle);

        const auto position = validate(IkTarget{PositionAndDirectionAngleTarget{
            Vector3{kNaN, 0.0, 0.0}, reference, 0.7}});
        ASSERT_FALSE(position.has_value());
        EXPECT_EQ(position.error().code, TargetValidationErrorCode::NonFinitePosition);
    }
}
