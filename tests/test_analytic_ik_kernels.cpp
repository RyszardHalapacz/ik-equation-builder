#include <gtest/gtest.h>

#include "fresh_generated/kr4_fresh.hpp"
#include "fresh_generated/kr640_fresh.hpp"
#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/solver/AnalyticIkSolver.hpp"
#include "ik_equations/solver/PrimitiveSolvers.hpp"
#include "ik_equations/solver/detail/AnalyticIkKernels.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <random>
#include <vector>

// Kernels shared by the general solver and the generated runtime
// (2026-11-16 follow-up #5). Each piece of IK math exists once; the two
// solvers differ only in how they obtain geometry and verify FK.

namespace ik = kinemaforge::ik;
namespace d = kinemaforge::ik::detail;
namespace gen = kinemaforge::ik::generated;
namespace ts = kinemaforge::testsupport;

namespace {
constexpr double kPi = std::numbers::pi_v<double>;
}

TEST(AnalyticIkKernels, SinCosDistinguishesNoneFiniteAndInfinite)
{
    EXPECT_EQ(d::solveSinCos(0.0, 0.0, 0.0, 1e-9).kind, d::SinCosKind::Infinite);
    EXPECT_EQ(d::solveSinCos(0.0, 0.0, 1.0, 1e-9).kind, d::SinCosKind::None);
    EXPECT_EQ(d::solveSinCos(1.0, 0.0, 2.0, 1e-9).kind, d::SinCosKind::None);
    const auto two = d::solveSinCos(1.0, 0.0, 0.5, 1e-9);
    ASSERT_EQ(two.kind, d::SinCosKind::Finite);
    ASSERT_EQ(two.values.size(), 2U);
    for (double x : two.values)
        EXPECT_NEAR(std::sin(x), 0.5, 1e-15);
    // Tangent: one distinct root.
    EXPECT_EQ(d::solveSinCos(1.0, 0.0, 1.0, 1e-9).values.size(), 1U);
}

TEST(AnalyticIkKernels, PublicSinCosIsTheKernel)
{
    std::mt19937_64 random{3};
    std::uniform_real_distribution<double> u(-2.0, 2.0);
    for (int i = 0; i < 1000; ++i)
    {
        const double a = u(random), b = u(random), c = u(random);
        const auto kernel = d::solveSinCos(a, b, c, 1e-9);
        const auto publicResult = ik::solveSinCos(a, b, c, 1e-9);
        const auto expected = kernel.kind == d::SinCosKind::Finite   ? ik::ScalarSolutionKind::Finite
                              : kernel.kind == d::SinCosKind::Infinite ? ik::ScalarSolutionKind::Infinite
                                                                       : ik::ScalarSolutionKind::None;
        ASSERT_EQ(publicResult.kind, expected);
        ASSERT_EQ(publicResult.values, kernel.values);
    }
}

TEST(AnalyticIkKernels, PlanarArmReachesTheTargetOnBothElbows)
{
    const auto arm = d::solvePlanarArm(1.0, 1.0, 1.0, 1.0, 0.0, 1.0, {}, 1e-9);
    EXPECT_FALSE(arm.foldedFamily);
    ASSERT_EQ(arm.branches.size(), 2U);
    for (const auto& branch : arm.branches)
    {
        EXPECT_NEAR(std::cos(branch.shoulder) + std::cos(branch.shoulder + branch.elbow), 1.0, 1e-12);
        EXPECT_NEAR(std::sin(branch.shoulder) + std::sin(branch.shoulder + branch.elbow), 1.0, 1e-12);
    }
    EXPECT_TRUE(d::solvePlanarArm(3.0, 0.0, 1.0, 1.0, 0.0, 1.0, {}, 1e-9).branches.empty());
}

TEST(AnalyticIkKernels, PlanarArmFlagsTheFoldedFamilyOfEqualLinks)
{
    const auto folded = d::solvePlanarArm(0.0, 0.0, 1.15, 1.15, 0.0, 1.0, {}, 1e-9);
    EXPECT_TRUE(folded.foldedFamily);
    // Unequal links cannot fold onto the shoulder axis.
    EXPECT_FALSE(d::solvePlanarArm(0.0, 0.0, 1.15, 1.25, 0.0, 1.0, {}, 1e-9).foldedFamily);
}

TEST(AnalyticIkKernels, PlanarArmSnapsAnIllConditionedElbowOntoItsLimitAndResolvesTheShoulder)
{
    // Straight arm, elbow limit just below the recovered 0: the elbow goes ON
    // the limit and the shoulder is recomputed for it, so the tip stays put.
    const double limit = -5e-8;
    const auto arm = d::solvePlanarArm(2.0, 0.0, 1.0, 1.0, 0.0, 1.0, {true, -1.0, limit}, 1e-9);
    ASSERT_FALSE(arm.branches.empty());
    for (const auto& branch : arm.branches)
    {
        EXPECT_EQ(branch.elbow, limit);
        EXPECT_NEAR(std::cos(branch.shoulder) + std::cos(branch.shoulder + branch.elbow), 2.0, 1e-14);
        EXPECT_NEAR(std::sin(branch.shoulder) + std::sin(branch.shoulder + branch.elbow), 0.0, 1e-14);
    }
}

TEST(AnalyticIkKernels, PublicPlanar2RIsTheKernelWithoutLimits)
{
    std::mt19937_64 random{4};
    std::uniform_real_distribution<double> u(-2.0, 2.0);
    for (int i = 0; i < 1000; ++i)
    {
        const double x = u(random), y = u(random), home = u(random);
        const double sign = i % 2 == 0 ? 1.0 : -1.0;
        const auto kernel = d::solvePlanarArm(x, y, 0.8, 0.5, home, sign, {}, 1e-9);
        const auto publicResult = ik::solvePlanar2R(x, y, 0.8, 0.5, home, sign, 1e-9);
        ASSERT_EQ(publicResult.size(), kernel.branches.size());
        for (std::size_t k = 0; k < publicResult.size(); ++k)
        {
            EXPECT_EQ(publicResult[k].firstJoint, kernel.branches[k].shoulder);
            EXPECT_EQ(publicResult[k].secondJoint, kernel.branches[k].elbow);
            EXPECT_EQ(publicResult[k].singular, kernel.branches[k].singular);
        }
    }
}

TEST(AnalyticIkKernels, EquivalentsListEveryAdmittedTurnSnappedOntoTheLimits)
{
    const d::LimitRange wide{true, -7.0, 7.0};
    EXPECT_EQ(d::equivalentCount(wide, 0.5), 3U);
    const auto values = d::equivalents(wide, 0.5);
    ASSERT_EQ(values.size(), 3U);
    EXPECT_NEAR(values[0], 0.5 - 2.0 * kPi, 1e-15);
    // Admitted just outside, returned exactly on the limit.
    const auto snapped = d::equivalents({true, 0.0, 1.0}, 1.0 + 5e-11);
    ASSERT_EQ(snapped.size(), 1U);
    EXPECT_EQ(snapped.front(), 1.0);
    EXPECT_EQ(d::equivalentCount({true, 0.0, 1.0}, 1.0 + 5e-11), 1U);
    EXPECT_TRUE(d::equivalents({true, 0.0, 1.0}, 2.0).empty());
    EXPECT_EQ(d::equivalents({}, 7.0).size(), 1U); // unlimited: one modulo value
}

namespace {
using M3 = std::array<std::array<double, 3>, 3>;
M3 mul(const M3& a, const M3& b)
{
    M3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                r[i][j] += a[i][k] * b[k][j];
    return r;
}
M3 rx(double t) { return M3{{{1, 0, 0}, {0, std::cos(t), -std::sin(t)}, {0, std::sin(t), std::cos(t)}}}; }
M3 ry(double t) { return M3{{{std::cos(t), 0, std::sin(t)}, {0, 1, 0}, {-std::sin(t), 0, std::cos(t)}}}; }
double distance(const M3& a, const M3& b)
{
    double worst = 0.0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            worst = std::max(worst, std::abs(a[i][j] - b[i][j]));
    return worst;
}
} // namespace

TEST(AnalyticIkKernels, WristGivesTwoRegularBranchesThatReproduceTheRotation)
{
    std::mt19937_64 random{5};
    std::uniform_real_distribution<double> angle(-3.0, 3.0);
    for (const double sign : {1.0, -1.0})
        for (int sample = 0; sample < 500; ++sample)
        {
            const double a = angle(random), b = angle(random), c = angle(random);
            if (std::abs(std::sin(b)) < 1e-3)
                continue;
            const M3 local = mul(mul(rx(a), ry(b)), rx(c * sign));
            const auto wrist = d::solveWrist(local, sign, {}, {}, {}, 1e-10);
            ASSERT_FALSE(wrist.singular);
            for (const auto& q : wrist.regular)
                EXPECT_LT(distance(mul(mul(rx(q[0]), ry(q[1])), rx(q[2] * sign)), local), 1e-13);
        }
}

TEST(AnalyticIkKernels, WristAtItsSingularityIsACoupledFamily)
{
    for (const double sign : {1.0, -1.0})
        for (const double pitch : {0.0, std::numbers::pi_v<double>})
        {
            const M3 local = mul(mul(rx(0.7), ry(pitch)), rx(-0.4 * sign));
            const auto wrist = d::solveWrist(local, sign, {}, {}, {}, 1e-10);
            ASSERT_TRUE(wrist.singular);
            EXPECT_EQ(wrist.aligned, pitch == 0.0);
            // Every q4 with q6 = offset + multiplier*q4 reproduces the rotation.
            for (const double q4 : {-2.0, 0.0, 1.3})
            {
                const double q6 = wrist.offset + wrist.multiplier * q4;
                EXPECT_LT(distance(mul(mul(rx(q4), ry(wrist.q5)), rx(q6 * sign)), local), 1e-13);
            }
        }
}

// Differential guard: the two solvers must return the same branches for the
// same robot and target. They share their math now; this catches any future
// drift directly rather than through an audit.
TEST(AnalyticIkKernels, GeneralAndGeneratedSolversReturnIdenticalBranches)
{
    for (const bool kr4 : {false, true})
    {
        const auto robot = ik::UrdfModelLoader{}.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) /
                                                     (kr4 ? "kr4_r600.urdf" : "kr640.urdf"));
        const auto chain = ik::KinematicChainBuilder{}.build(robot, "base_link", "tool0").value();
        const auto parameters = kr4 ? kr4_fresh::parameters : kr640_fresh::parameters;
        std::mt19937_64 random{kr4 ? 41U : 640U};
        for (int sample = 0; sample < 300; ++sample)
        {
            SCOPED_TRACE(sample);
            std::array<double, 6> q{};
            for (std::size_t i = 0; i < 6; ++i)
                q[i] = std::uniform_real_distribution<double>(parameters.limits[i].lower,
                                                              parameters.limits[i].upper)(random);
            if (sample % 5 == 0)
                q[4] = 0.0;
            const auto fk = ts::numericForwardKinematics(chain, std::vector<double>(q.begin(), q.end()));
            const ik::PoseTarget target{{fk.translation.x, fk.translation.y, fk.translation.z},
                                        {ts::toRotationMatrix(fk.rotation)}};
            const auto general = ik::AnalyticIkSolver{}.solve(chain, target);
            const auto compiled = gen::solve(parameters, {{target.position.x, target.position.y,
                                                           target.position.z},
                                                          target.orientation.values});
            ASSERT_TRUE(general) << general.error().message;
            ASSERT_EQ(general->branches.size(), compiled.branches.size());

            // Compare as sets: sort both by joint values.
            std::vector<std::array<double, 6>> a, b;
            for (const auto& branch : general->branches)
            {
                std::array<double, 6> v{};
                v.fill(-100.0);
                for (const auto& joint : branch.joints)
                    v[joint.actuatedJointPosition] = joint.value;
                a.push_back(v);
            }
            for (const auto& branch : compiled.branches)
            {
                auto v = branch.q;
                if (branch.singular)
                    v[3] = v[5] = -100.0;
                b.push_back(v);
            }
            std::sort(a.begin(), a.end());
            std::sort(b.begin(), b.end());
            for (std::size_t k = 0; k < a.size(); ++k)
                for (std::size_t i = 0; i < 6; ++i)
                    EXPECT_NEAR(a[k][i], b[k][i], 1e-9) << "branch " << k << " joint " << i;
            if (HasFailure())
                return;
        }
    }
}
