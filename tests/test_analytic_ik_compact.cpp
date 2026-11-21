#include <gtest/gtest.h>

#include "fresh_generated/kr4_fresh.hpp"
#include "fresh_generated/kr640_fresh.hpp"
#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/solver/AnalyticIkSolver.hpp"
#include "ik_equations/solver/detail/AnalyticIkKernels.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <random>
#include <vector>

// Compact results (follow-up 2026-11-16 #4): geometric branches with, per
// joint, the SET of its in-limit full turns (base + 2*pi*k, k in a range)
// instead of the product of all combinations. Its size does not depend on how
// wide the limits are. expand() reproduces exactly what solve() returns.

namespace ik = kinemaforge::ik;
namespace d = kinemaforge::ik::detail;
namespace gen = kinemaforge::ik::generated;
namespace ts = kinemaforge::testsupport;

namespace {

constexpr double kPeriod = 2.0 * std::numbers::pi_v<double>;
using Configuration = std::array<double, 6>;

// A branch flattened for set comparison: q (singular q4/q6 replaced by the
// family's free interval and relation).
using Flat = std::array<double, 10>;

Flat flatten(const gen::Branch& b)
{
    Flat f{};
    for (std::size_t i = 0; i < 6; ++i)
        f[i] = b.singular && (i == 3 || i == 5) ? 0.0 : b.q[i];
    if (b.singular)
    {
        f[6] = b.freeLower;
        f[7] = b.freeUpper;
        f[8] = b.dependentMultiplier;
        f[9] = b.dependentOffset;
    }
    return f;
}

Flat flatten(const ik::SolutionBranch& b)
{
    Flat f{};
    for (const auto& joint : b.joints)
        f[joint.actuatedJointPosition] = joint.value;
    if (!b.freeParameters.empty())
    {
        f[6] = b.freeParameters.front().lower;
        f[7] = b.freeParameters.front().upper;
        f[8] = b.coupledFreeParameters.front().multiplier;
        f[9] = b.coupledFreeParameters.front().offset;
    }
    return f;
}

template <class Branches>
std::vector<Flat> flatAll(const Branches& branches)
{
    std::vector<Flat> out;
    for (const auto& b : branches)
        out.push_back(flatten(b));
    std::sort(out.begin(), out.end());
    return out;
}

void expectSameSets(std::vector<Flat> a, std::vector<Flat> b)
{
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t k = 0; k < a.size(); ++k)
        for (std::size_t i = 0; i < a[k].size(); ++i)
            ASSERT_EQ(a[k][i], b[k][i]) << "branch " << k << " field " << i;
}

struct Robot
{
    ik::KinematicChain chain;
    gen::Parameters parameters;

    explicit Robot(bool kr4)
    {
        const auto model = ik::UrdfModelLoader{}.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) /
                                                     (kr4 ? "kr4_r600.urdf" : "kr640.urdf"));
        chain = ik::KinematicChainBuilder{}.build(model, "base_link", "tool0").value();
        parameters = kr4 ? kr4_fresh::parameters : kr640_fresh::parameters;
    }

    ik::KinematicJoint& joint(std::size_t position)
    {
        for (auto& entry : chain.joints)
            if (entry.type != ik::JointType::Fixed && position-- == 0)
                return entry;
        throw std::logic_error("missing joint");
    }

    ik::PoseTarget pose(const Configuration& q) const
    {
        const auto fk = ts::numericForwardKinematics(chain, std::vector<double>(q.begin(), q.end()));
        return {{fk.translation.x, fk.translation.y, fk.translation.z}, {ts::toRotationMatrix(fk.rotation)}};
    }
};

void setLimit(Robot& robot, std::size_t position, double lower, double upper)
{
    auto& joint = robot.joint(position);
    joint.limits.hasPositionLimits = true;
    joint.limits.lower = lower;
    joint.limits.upper = upper;
    robot.parameters.limits[position] = {true, lower, upper};
}

gen::Pose generatedPose(const ik::PoseTarget& target)
{
    return {{target.position.x, target.position.y, target.position.z}, target.orientation.values};
}

} // namespace

// --- Kernel ---------------------------------------------------------------

TEST(AnalyticIkCompact, TurnSetIsTheSingleSourceOfEquivalents)
{
    std::mt19937_64 random{9};
    std::uniform_real_distribution<double> u(-20.0, 20.0);
    for (int sample = 0; sample < 5000; ++sample)
    {
        const double a = u(random), b = u(random);
        const d::LimitRange limit{sample % 5 != 0, std::min(a, b), std::max(a, b)};
        const double value = u(random);
        const auto set = d::turnSet(limit, value);
        const auto list = d::equivalents(limit, value);
        ASSERT_EQ(set.count(), list.size()) << sample;
        ASSERT_EQ(d::equivalentCount(limit, value), list.size());
        for (std::size_t i = 0; i < list.size(); ++i)
            ASSERT_EQ(set.value(set.first + static_cast<std::int64_t>(i)), list[i]) << sample;
    }
}

TEST(AnalyticIkCompact, TurnSetEndsSnapOntoTheLimits)
{
    const auto set = d::turnSet({true, 0.0, 1.0}, 1.0 + 5e-11);
    ASSERT_EQ(set.count(), 1U);
    EXPECT_EQ(set.value(set.first), 1.0);
    EXPECT_EQ(d::turnSet({true, 0.0, 1.0}, 2.0).count(), 0U);
    EXPECT_EQ(d::turnSet({}, 9.0).count(), 1U);
}

// --- General solver --------------------------------------------------------

TEST(AnalyticIkCompact, GeneralExpandEqualsSolveForRandomTargetsAndLimits)
{
    for (const bool kr4 : {false, true})
    {
        std::mt19937_64 random{kr4 ? 77U : 76U};
        for (int sample = 0; sample < 300; ++sample)
        {
            SCOPED_TRACE(sample);
            Robot robot{kr4};
            Configuration q{};
            for (std::size_t i = 0; i < 6; ++i)
                q[i] = std::uniform_real_distribution<double>(robot.parameters.limits[i].lower + 0.1,
                                                              robot.parameters.limits[i].upper - 0.1)(random);
            if (sample % 4 == 0)
                q[4] = 0.0;
            // Some joints with several full turns.
            for (std::size_t i : {3U, 5U})
                if (sample % 3 == 0)
                    setLimit(robot, i, q[i] - 7.0, q[i] + 8.0);
            const auto target = robot.pose(q);

            const auto full = ik::AnalyticIkSolver{}.solve(robot.chain, target);
            const auto compact = ik::AnalyticIkSolver{}.solveCompact(robot.chain, target);
            ASSERT_TRUE(full) << full.error().message;
            ASSERT_TRUE(compact) << compact.error().message;
            EXPECT_EQ(compact->status, full->status);
            EXPECT_EQ(compact->expandedBranchCount(), full->branches.size());
            expectSameSets(flatAll(ik::expand(*compact)), flatAll(full->branches));
            if (HasFailure())
                return;
        }
    }
}

TEST(AnalyticIkCompact, GeneralCompactHandlesLimitsTooWideToExpand)
{
    Robot robot{false};
    const Configuration q{0.4, -0.5, 0.7, 0.8, -0.9, 1.1};
    setLimit(robot, 3, -1e5, 1e5);
    setLimit(robot, 5, -1e5, 1e5);
    const auto target = robot.pose(q);

    const auto full = ik::AnalyticIkSolver{}.solve(robot.chain, target);
    ASSERT_FALSE(full);
    ASSERT_EQ(full.error().code, ik::AnalyticIkSolverErrorCode::ResourceLimitExceeded);

    const auto compact = ik::AnalyticIkSolver{}.solveCompact(robot.chain, target);
    ASSERT_TRUE(compact) << compact.error().message;
    EXPECT_EQ(compact->status, ik::AnalyticIkStatus::Solutions);
    EXPECT_EQ(compact->expandedBranchCount(), *full.error().requiredBranches);
    EXPECT_LE(compact->branches.size(), 8U); // geometric branches only

    // Any turn combination reproduces the target; check the extremes.
    for (const auto& branch : compact->branches)
        for (const bool high : {false, true})
        {
            Configuration values{};
            for (const auto& joint : branch.joints)
                values[joint.actuatedJointPosition] =
                    joint.value(high ? joint.lastTurn : joint.firstTurn);
            const auto reached = robot.pose(values);
            EXPECT_NEAR(reached.position.x, target.position.x, 1e-8);
            EXPECT_NEAR(reached.position.y, target.position.y, 1e-8);
            EXPECT_NEAR(reached.position.z, target.position.z, 1e-8);
        }
}

TEST(AnalyticIkCompact, GeneralCompactReportsNoSolutionAndUnsupportedLikeSolve)
{
    Robot robot{false};
    ik::PoseTarget far = robot.pose({0.4, -0.5, 0.7, 0.8, -0.9, 1.1});
    far.position.x += 100.0;
    const auto compact = ik::AnalyticIkSolver{}.solveCompact(robot.chain, far);
    ASSERT_TRUE(compact);
    EXPECT_EQ(compact->status, ik::AnalyticIkStatus::NoSolution);
    EXPECT_TRUE(compact->branches.empty());
}

// --- Generated runtime -----------------------------------------------------

TEST(AnalyticIkCompact, GeneratedExpandEqualsSolveForRandomTargetsAndLimits)
{
    for (const bool kr4 : {false, true})
    {
        std::mt19937_64 random{kr4 ? 87U : 86U};
        for (int sample = 0; sample < 300; ++sample)
        {
            SCOPED_TRACE(sample);
            Robot robot{kr4};
            Configuration q{};
            for (std::size_t i = 0; i < 6; ++i)
                q[i] = std::uniform_real_distribution<double>(robot.parameters.limits[i].lower + 0.1,
                                                              robot.parameters.limits[i].upper - 0.1)(random);
            if (sample % 4 == 0)
                q[4] = 0.0;
            for (std::size_t i : {3U, 5U})
                if (sample % 3 == 0)
                    setLimit(robot, i, q[i] - 7.0, q[i] + 8.0);
            const auto target = generatedPose(robot.pose(q));

            const auto full = gen::solve(robot.parameters, target);
            const auto compact = gen::solveCompact(robot.parameters, target);
            EXPECT_EQ(compact.status, full.status);
            EXPECT_EQ(compact.expandedBranchCount(), full.branches.size());
            expectSameSets(flatAll(gen::expand(compact)), flatAll(full.branches));
            if (HasFailure())
                return;
        }
    }
}

TEST(AnalyticIkCompact, GeneratedCompactHandlesLimitsTooWideToExpand)
{
    Robot robot{false};
    const Configuration q{0.4, -0.5, 0.7, 0.8, -0.9, 1.1};
    setLimit(robot, 3, -1e5, 1e5);
    setLimit(robot, 5, -1e5, 1e5);
    const auto target = generatedPose(robot.pose(q));
    const auto full = gen::solve(robot.parameters, target);
    ASSERT_EQ(full.status, gen::Status::ResourceLimitExceeded);
    const auto compact = gen::solveCompact(robot.parameters, target);
    EXPECT_EQ(compact.status, gen::Status::Solutions);
    EXPECT_EQ(compact.expandedBranchCount(), full.requiredBranches);
    EXPECT_LE(compact.branches.size(), 8U);
}
