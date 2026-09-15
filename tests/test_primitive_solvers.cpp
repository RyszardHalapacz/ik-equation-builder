#include <gtest/gtest.h>

#include "ik_equations/solver/PrimitiveSolvers.hpp"

#include <cmath>
#include <numbers>

namespace ik = kinemaforge::ik;

TEST(PrimitiveSolvers, LinearUniqueSolution)
{
    const auto result = ik::solveLinear(2.0, -6.0);
    ASSERT_EQ(result.kind, ik::ScalarSolutionKind::Finite);
    ASSERT_EQ(result.values.size(), 1U);
    EXPECT_DOUBLE_EQ(result.values.front(), 3.0);
}

TEST(PrimitiveSolvers, LinearDetectsNoneAndInfinite)
{
    EXPECT_EQ(ik::solveLinear(0.0, 1.0).kind, ik::ScalarSolutionKind::None);
    EXPECT_EQ(ik::solveLinear(0.0, 0.0).kind, ik::ScalarSolutionKind::Infinite);
}

TEST(PrimitiveSolvers, SineReturnsBothBranches)
{
    const auto result = ik::solveSine(0.5);
    ASSERT_EQ(result.kind, ik::ScalarSolutionKind::Finite);
    ASSERT_EQ(result.values.size(), 2U);
    EXPECT_NEAR(std::sin(result.values[0]), 0.5, 1e-12);
    EXPECT_NEAR(std::sin(result.values[1]), 0.5, 1e-12);
}

TEST(PrimitiveSolvers, SineRejectsUnreachableValue)
{
    EXPECT_EQ(ik::solveSine(1.01).kind, ik::ScalarSolutionKind::None);
}

TEST(PrimitiveSolvers, CosineDeduplicatesEndpoint)
{
    const auto result = ik::solveCosine(1.0);
    ASSERT_EQ(result.kind, ik::ScalarSolutionKind::Finite);
    ASSERT_EQ(result.values.size(), 1U);
    EXPECT_NEAR(result.values.front(), 0.0, 1e-12);
}

TEST(PrimitiveSolvers, SinCosReturnsAllBranches)
{
    const auto result = ik::solveSinCos(2.0, -1.0, 0.75);
    ASSERT_EQ(result.kind, ik::ScalarSolutionKind::Finite);
    ASSERT_EQ(result.values.size(), 2U);
    for (double value : result.values)
        EXPECT_NEAR(2.0 * std::sin(value) - std::cos(value), 0.75, 1e-12);
}

TEST(PrimitiveSolvers, SinCosDetectsDegenerateEquation)
{
    EXPECT_EQ(ik::solveSinCos(0.0, 0.0, 0.0).kind, ik::ScalarSolutionKind::Infinite);
    EXPECT_EQ(ik::solveSinCos(0.0, 0.0, 1.0).kind, ik::ScalarSolutionKind::None);
}

TEST(PrimitiveSolvers, NormalizeAngleUsesSinglePiSeam)
{
    EXPECT_NEAR(ik::normalizeAngle(std::numbers::pi_v<double>),
                -std::numbers::pi_v<double>, 1e-15);
    EXPECT_NEAR(ik::normalizeAngle(3.0 * std::numbers::pi_v<double>),
                -std::numbers::pi_v<double>, 1e-15);
}

TEST(PrimitiveSolvers, Planar2RFindsBothElbowBranches)
{
    const auto result = ik::solvePlanar2R(1.0, 1.0, 1.0, 1.0, 0.0);
    ASSERT_EQ(result.size(), 2U);

    for (const auto& branch : result)
    {
        const double x = std::cos(branch.firstJoint) +
                         std::cos(branch.firstJoint + branch.secondJoint);
        const double y = std::sin(branch.firstJoint) +
                         std::sin(branch.firstJoint + branch.secondJoint);
        EXPECT_NEAR(x, 1.0, 1e-12);
        EXPECT_NEAR(y, 1.0, 1e-12);
    }
    EXPECT_NE(result[0].elbow, result[1].elbow);
}

TEST(PrimitiveSolvers, Planar2RHonoursHomeAngleAndAxisSign)
{
    constexpr double home = std::numbers::pi_v<double> / 3.0;
    const double q1 = 0.4;
    const double q2 = -0.7;
    const double effective = home - q2; // second axis sign = -1
    const double x = 0.8 * std::cos(q1) + 0.5 * std::cos(q1 + effective);
    const double y = 0.8 * std::sin(q1) + 0.5 * std::sin(q1 + effective);

    const auto result = ik::solvePlanar2R(x, y, 0.8, 0.5, home, -1.0);
    ASSERT_FALSE(result.empty());

    bool found = false;
    for (const auto& branch : result)
        if (std::abs(ik::normalizeAngle(branch.firstJoint - q1)) < 1e-12 &&
            std::abs(ik::normalizeAngle(branch.secondJoint - q2)) < 1e-12)
            found = true;
    EXPECT_TRUE(found);
}

TEST(PrimitiveSolvers, Planar2RRejectsUnreachableTarget)
{
    EXPECT_TRUE(ik::solvePlanar2R(3.0, 0.0, 1.0, 1.0, 0.0).empty());
}

TEST(PrimitiveSolvers, Planar2RMarksCollinearConfigurationSingular)
{
    const auto result = ik::solvePlanar2R(2.0, 0.0, 1.0, 1.0, 0.0);
    ASSERT_EQ(result.size(), 1U);
    EXPECT_TRUE(result.front().singular);
}
