#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/solver/AnalyticIkSolver.hpp"
#include "ik_equations/solver/PreparedAnalyticIk.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <filesystem>
#include <random>
#include <thread>
#include <vector>

// PreparedAnalyticIk (MotionBridge report 2026-11-21, 5.5): everything that
// depends only on the robot -- validated chain, geometry, detected pattern,
// home transform -- is computed once; solve() then does only the work that
// depends on the target. Immutable after creation, so one object may be shared
// by any number of threads. Results are IDENTICAL to AnalyticIkSolver::solve.

namespace ik = kinemaforge::ik;
namespace ts = kinemaforge::testsupport;

namespace {

ik::KinematicChain kr640Chain()
{
    return ik::KinematicChainBuilder{}
        .build(ik::UrdfModelLoader{}.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / "kr640.urdf"),
               "base_link", "tool0")
        .value();
}

std::vector<ik::PoseTarget> targets(const ik::KinematicChain& chain, int count, unsigned seed,
                                    const std::optional<ik::FixedRigidTransform>& tcp = std::nullopt)
{
    std::mt19937_64 rng(seed);
    std::vector<ik::PoseTarget> result;
    for (int n = 0; n < count; ++n)
    {
        std::vector<double> q;
        for (const auto& joint : chain.joints)
            if (joint.type != ik::JointType::Fixed)
                q.push_back(joint.limits.lower + std::uniform_real_distribution<double>(0.05, 0.95)(rng) *
                                                     (joint.limits.upper - joint.limits.lower));
        const auto fk = tcp ? ts::numericTcpForwardKinematics(chain, q, *tcp) : ts::numericForwardKinematics(chain, q);
        result.push_back({{fk.translation.x, fk.translation.y, fk.translation.z}, {ts::toRotationMatrix(fk.rotation)}});
    }
    return result;
}

bool identical(const ik::AnalyticIkSolution& a, const ik::AnalyticIkSolution& b)
{
    if (a.status != b.status || a.branches.size() != b.branches.size() ||
        a.positionalFamilies.size() != b.positionalFamilies.size() ||
        a.shoulderAxisDistance != b.shoulderAxisDistance || a.orientationCorrection != b.orientationCorrection)
        return false;
    for (std::size_t i = 0; i < a.branches.size(); ++i)
    {
        const auto& x = a.branches[i];
        const auto& y = b.branches[i];
        if (x.shoulder != y.shoulder || x.elbow != y.elbow || x.wrist != y.wrist ||
            x.joints.size() != y.joints.size() || x.freeParameters.size() != y.freeParameters.size())
            return false;
        for (std::size_t j = 0; j < x.joints.size(); ++j)
            if (x.joints[j].actuatedJointPosition != y.joints[j].actuatedJointPosition ||
                x.joints[j].value != y.joints[j].value)
                return false;
    }
    return true;
}

} // namespace

TEST(PreparedAnalyticIk, GivesExactlyWhatTheOneShotSolverGives)
{
    const auto chain = kr640Chain();
    const auto prepared = ik::PreparedAnalyticIk::create(chain);
    ASSERT_TRUE(prepared) << prepared.error().message;
    for (const auto& target : targets(chain, 100, 0x9E3))
    {
        const auto expected = ik::AnalyticIkSolver{}.solve(chain, target);
        const auto actual = prepared->solve(target);
        ASSERT_TRUE(expected && actual);
        EXPECT_TRUE(identical(*expected, *actual));
    }
}

TEST(PreparedAnalyticIk, CompactResultMatchesToo)
{
    const auto chain = kr640Chain();
    const auto prepared = ik::PreparedAnalyticIk::create(chain);
    ASSERT_TRUE(prepared);
    for (const auto& target : targets(chain, 50, 0xC0))
    {
        const auto expected = ik::AnalyticIkSolver{}.solveCompact(chain, target);
        const auto actual = prepared->solveCompact(target);
        ASSERT_TRUE(expected && actual);
        EXPECT_EQ(actual->expandedBranchCount(), expected->expandedBranchCount());
        EXPECT_EQ(actual->branches.size(), expected->branches.size());
        EXPECT_EQ(actual->shoulderAxisDistance, expected->shoulderAxisDistance);
    }
}

TEST(PreparedAnalyticIk, HonoursTheToolTransform)
{
    const auto chain = kr640Chain();
    const ik::FixedRigidTransform tcp{{0.03, -0.04, 0.12}, {0.2, -0.1, 0.3}};
    const auto prepared = ik::PreparedAnalyticIk::create(chain, tcp);
    ASSERT_TRUE(prepared) << prepared.error().message;
    for (const auto& target : targets(chain, 30, 0x7C9, tcp))
    {
        const auto expected = ik::AnalyticIkSolver{}.solve(chain, target, tcp);
        const auto actual = prepared->solve(target);
        ASSERT_TRUE(expected && actual);
        EXPECT_TRUE(identical(*expected, *actual));
        EXPECT_FALSE(actual->branches.empty());
    }
}

TEST(PreparedAnalyticIk, ReportsAnUnsupportedChainWhenCreated)
{
    const ik::KinematicChain empty;
    const auto prepared = ik::PreparedAnalyticIk::create(empty);
    ASSERT_FALSE(prepared);
    EXPECT_EQ(prepared.error().code, ik::AnalyticIkSolverErrorCode::UnsupportedKinematicStructure);
}

TEST(PreparedAnalyticIk, RejectsAnInvalidTargetLikeTheOneShotSolver)
{
    const auto chain = kr640Chain();
    const auto prepared = ik::PreparedAnalyticIk::create(chain);
    ASSERT_TRUE(prepared);
    const ik::PoseTarget bad{{0.0, 0.0, 0.0}, {{{{2, 0, 0}, {0, 1, 0}, {0, 0, 1}}}}};
    const auto expected = ik::AnalyticIkSolver{}.solve(chain, bad);
    const auto actual = prepared->solve(bad);
    ASSERT_FALSE(expected);
    ASSERT_FALSE(actual);
    EXPECT_EQ(actual.error().code, expected.error().code);
    EXPECT_EQ(actual.error().code, ik::AnalyticIkSolverErrorCode::InvalidTarget);
}

TEST(PreparedAnalyticIk, BranchBudgetIsStillEnforced)
{
    auto chain = kr640Chain();
    for (auto& joint : chain.joints)
        if (joint.type != ik::JointType::Fixed)
        {
            joint.limits.hasPositionLimits = true;
            joint.limits.lower = -50.0;
            joint.limits.upper = 50.0;
        }
    const auto prepared = ik::PreparedAnalyticIk::create(chain);
    ASSERT_TRUE(prepared);
    const auto target = targets(kr640Chain(), 1, 3).front();
    ik::AnalyticIkSolverOptions options;
    options.maxBranches = 100;
    const auto result = prepared->solve(target, options);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, ik::AnalyticIkSolverErrorCode::ResourceLimitExceeded);
    EXPECT_GT(*result.error().requiredBranches, 100U);
}

TEST(PreparedAnalyticIk, SurvivesCopyAndMove)
{
    const auto chain = kr640Chain();
    auto original = ik::PreparedAnalyticIk::create(chain);
    ASSERT_TRUE(original);
    const auto target = targets(chain, 1, 11).front();
    const auto expected = ik::AnalyticIkSolver{}.solve(chain, target);
    ASSERT_TRUE(expected);
    const ik::PreparedAnalyticIk copy = *original;
    const ik::PreparedAnalyticIk moved = std::move(*original);
    EXPECT_TRUE(identical(*expected, *copy.solve(target)));
    EXPECT_TRUE(identical(*expected, *moved.solve(target)));
}

TEST(PreparedAnalyticIk, OneObjectServesManyThreads)
{
    const auto chain = kr640Chain();
    const auto prepared = ik::PreparedAnalyticIk::create(chain);
    ASSERT_TRUE(prepared);
    const auto cases = targets(chain, 80, 0x7D);
    std::vector<ik::AnalyticIkSolution> serial;
    for (const auto& target : cases)
        serial.push_back(*prepared->solve(target));

    std::vector<int> mismatches(4, 0);
    std::vector<std::thread> threads;
    for (int t = 0; t < 4; ++t)
        threads.emplace_back([&, t] {
            for (int round = 0; round < 5; ++round)
                for (std::size_t i = 0; i < cases.size(); ++i)
                {
                    const auto result = prepared->solve(cases[i]);
                    if (!result || !identical(*result, serial[i]))
                        ++mismatches[t];
                }
        });
    for (auto& thread : threads)
        thread.join();
    for (const int count : mismatches)
        EXPECT_EQ(count, 0);
}
