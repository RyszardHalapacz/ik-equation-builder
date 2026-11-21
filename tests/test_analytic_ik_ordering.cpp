#include <gtest/gtest.h>

#include "fresh_generated/kr4_fresh.hpp"
#include "fresh_generated/kr640_fresh.hpp"
#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/solver/AnalyticIkSolver.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <random>
#include <tuple>
#include <vector>

// Order of the branches in a result (MotionBridge report 2026-11-21, 5.4).
//
// The contract, stated once in AnalyticIkSolver.hpp: branches are sorted by
// (shoulder, elbow, wrist) -- the enum order -- and within one of those by
// the joint values in joint order q1..q6, ascending. Both solvers, solve() and
// expand(solveCompact()), return the same sequence for the same target.

namespace ik = kinemaforge::ik;
namespace gen = kinemaforge::ik::generated;
namespace ts = kinemaforge::testsupport;

namespace {

struct Robot
{
    ik::KinematicChain chain;
    gen::Parameters parameters;
    std::optional<ik::FixedRigidTransform> tcp;
};

Robot loadRobot(const char* urdf, const gen::Parameters& parameters)
{
    Robot robot;
    robot.chain = ik::KinematicChainBuilder{}
                      .build(ik::UrdfModelLoader{}.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / urdf),
                             "base_link", "tool0")
                      .value();
    robot.parameters = parameters;
    return robot;
}

ik::PoseTarget poseOf(const Robot& robot, const std::vector<double>& q)
{
    const auto fk = ts::numericForwardKinematics(robot.chain, q);
    return {{fk.translation.x, fk.translation.y, fk.translation.z}, {ts::toRotationMatrix(fk.rotation)}};
}

std::vector<std::vector<double>> randomConfigurations(const Robot& robot, unsigned seed, int count)
{
    std::mt19937_64 rng(seed);
    std::vector<ik::JointLimits> limits;
    for (const auto& joint : robot.chain.joints)
        if (joint.type != ik::JointType::Fixed)
            limits.push_back(joint.limits);
    std::vector<std::vector<double>> result;
    for (int n = 0; n < count; ++n)
    {
        std::vector<double> q;
        for (const auto& limit : limits)
            q.push_back(limit.lower +
                        std::uniform_real_distribution<double>(0.05, 0.95)(rng) * (limit.upper - limit.lower));
        result.push_back(std::move(q));
    }
    return result;
}

// What a branch is sorted by: its three labels, then its joint values in joint
// order, then (for a wrist family) the free interval.
using Key = std::tuple<int, int, int, std::vector<std::pair<std::size_t, double>>, double>;

Key keyOf(const ik::SolutionBranch& branch)
{
    std::vector<std::pair<std::size_t, double>> values;
    for (const auto& joint : branch.joints)
        values.emplace_back(joint.actuatedJointPosition, joint.value);
    return {static_cast<int>(branch.shoulder), static_cast<int>(branch.elbow),
            static_cast<int>(branch.wrist), values,
            branch.freeParameters.empty() ? 0.0 : branch.freeParameters.front().lower};
}

bool sameJoints(const ik::SolutionBranch& a, const ik::SolutionBranch& b)
{
    if (a.joints.size() != b.joints.size())
        return false;
    for (std::size_t i = 0; i < a.joints.size(); ++i)
        if (a.joints[i].actuatedJointPosition != b.joints[i].actuatedJointPosition ||
            a.joints[i].value != b.joints[i].value)
            return false;
    return true;
}

class OrderingTest : public testing::TestWithParam<int>
{
protected:
    Robot robot;
    void SetUp() override
    {
        robot = GetParam() == 0 ? loadRobot("kr640.urdf", kr640_fresh::parameters)
                                : loadRobot("kr4_r600.urdf", kr4_fresh::parameters);
    }
};

} // namespace

TEST_P(OrderingTest, SolveReturnsBranchesInTheDocumentedOrder)
{
    int checked = 0;
    for (const auto& q : randomConfigurations(robot, 0x0DE5, 200))
    {
        const auto solved = ik::AnalyticIkSolver{}.solve(robot.chain, poseOf(robot, q));
        ASSERT_TRUE(solved) << solved.error().message;
        const auto& branches = solved->branches;
        for (std::size_t i = 1; i < branches.size(); ++i)
            ASSERT_LE(keyOf(branches[i - 1]), keyOf(branches[i])) << "branch " << i;
        checked += static_cast<int>(branches.size());
    }
    EXPECT_GT(checked, 1000);
}

TEST_P(OrderingTest, ExpandedCompactResultHasTheSameOrderAsSolve)
{
    for (const auto& q : randomConfigurations(robot, 0xC0FFEE, 100))
    {
        const auto target = poseOf(robot, q);
        const auto solved = ik::AnalyticIkSolver{}.solve(robot.chain, target);
        const auto compact = ik::AnalyticIkSolver{}.solveCompact(robot.chain, target);
        ASSERT_TRUE(solved && compact);
        const auto expanded = ik::expand(*compact);
        ASSERT_EQ(expanded.size(), solved->branches.size());
        for (std::size_t i = 0; i < expanded.size(); ++i)
            ASSERT_TRUE(sameJoints(expanded[i], solved->branches[i])) << "branch " << i;
    }
}

TEST_P(OrderingTest, GeneratedSolverReturnsTheSameSequenceAsTheGeneralSolver)
{
    for (const auto& q : randomConfigurations(robot, 0xBEEF, 100))
    {
        const auto target = poseOf(robot, q);
        const auto solved = ik::AnalyticIkSolver{}.solve(robot.chain, target);
        const auto compiled = gen::solve(robot.parameters, {{target.position.x, target.position.y, target.position.z},
                                                            target.orientation.values});
        ASSERT_TRUE(solved);
        ASSERT_EQ(compiled.branches.size(), solved->branches.size());
        for (std::size_t i = 0; i < compiled.branches.size(); ++i)
            for (const auto& joint : solved->branches[i].joints)
                ASSERT_EQ(compiled.branches[i].q[joint.actuatedJointPosition], joint.value) << "branch " << i;
    }
}

TEST_P(OrderingTest, RepeatedCallsReturnIdenticalResults)
{
    const auto q = randomConfigurations(robot, 7, 1).front();
    const auto target = poseOf(robot, q);
    const auto first = ik::AnalyticIkSolver{}.solve(robot.chain, target);
    const auto second = ik::AnalyticIkSolver{}.solve(robot.chain, target);
    ASSERT_TRUE(first && second);
    ASSERT_EQ(first->branches.size(), second->branches.size());
    for (std::size_t i = 0; i < first->branches.size(); ++i)
        EXPECT_TRUE(sameJoints(first->branches[i], second->branches[i]));
}

TEST_P(OrderingTest, TheOrderHoldsWithWideLimitsSeveralTurnsAndWristSingularities)
{
    // Several full turns per joint, and q5 = 0 so the wrist is a family.
    for (auto& joint : robot.chain.joints)
        if (joint.type != ik::JointType::Fixed)
        {
            joint.limits.hasPositionLimits = true;
            joint.limits.lower = -7.0;
            joint.limits.upper = 7.0;
        }
    ik::AnalyticIkSolverOptions options;
    options.maxBranches = 1U << 22;
    int families = 0;
    for (auto q : randomConfigurations(robot, 0x51DE, 40))
    {
        q[4] = 0.0;
        for (auto& value : q)
            value = std::clamp(value, -6.9, 6.9);
        const auto solved = ik::AnalyticIkSolver{}.solve(robot.chain, poseOf(robot, q), std::nullopt, options);
        ASSERT_TRUE(solved) << solved.error().message;
        const auto& branches = solved->branches;
        for (std::size_t i = 1; i < branches.size(); ++i)
            ASSERT_LE(keyOf(branches[i - 1]), keyOf(branches[i])) << "branch " << i;
        families += std::any_of(branches.begin(), branches.end(),
                                [](const auto& b) { return !b.freeParameters.empty(); });
    }
    EXPECT_GT(families, 0);
}

INSTANTIATE_TEST_SUITE_P(Robots, OrderingTest, testing::Values(0, 1));
