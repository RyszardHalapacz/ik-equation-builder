#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/IkEquationBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/solver/AnalyticIkSolver.hpp"
#include "ik_equations/solver/PrimitiveSolvers.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

namespace ik = kinemaforge::ik;
namespace ts = kinemaforge::testsupport;
namespace {

ik::KinematicChain loadChain(const char* fileName)
{
    ik::UrdfModelLoader loader;
    const auto robot = loader.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName);
    const auto chain = ik::KinematicChainBuilder{}.build(robot, "base_link", "tool0");
    return chain ? *chain : ik::KinematicChain{};
}

ik::PoseTarget targetFor(const ik::KinematicChain& chain, const std::vector<double>& q)
{
    const auto fk = ts::numericForwardKinematics(chain, q);
    return {{fk.translation.x, fk.translation.y, fk.translation.z},
            {ts::toRotationMatrix(fk.rotation)}};
}

bool sameConfiguration(const ik::SolutionBranch& branch, const std::vector<double>& q,
                       double tolerance = 3e-7)
{
    if (branch.joints.size() != q.size())
        return false;
    for (std::size_t index = 0; index < q.size(); ++index)
        if (std::abs(ik::normalizeAngle(branch.joints[index].value - q[index])) > tolerance)
            return false;
    return true;
}

bool containsConfiguration(const ik::AnalyticIkSolution& solution, const std::vector<double>& q)
{
    return std::any_of(solution.branches.begin(), solution.branches.end(),
                       [&](const auto& branch) { return sameConfiguration(branch, q); });
}

std::vector<const ik::KinematicJoint*> actuated(const ik::KinematicChain& chain)
{
    std::vector<const ik::KinematicJoint*> result;
    for (const auto& joint : chain.joints)
        if (joint.type != ik::JointType::Fixed)
            result.push_back(&joint);
    return result;
}

} // namespace

TEST(AnalyticIkSolver, RecoversKr640Configuration)
{
    const auto chain = loadChain("kr640.urdf");
    const std::vector<double> q{0.4, -0.5, 0.7, 0.8, -0.9, 1.1};
    const auto result = ik::AnalyticIkSolver{}.solve(chain, targetFor(chain, q));
    ASSERT_TRUE(result);
    EXPECT_EQ(result->status, ik::AnalyticIkStatus::Solutions);
    EXPECT_TRUE(containsConfiguration(*result, q));
}

TEST(AnalyticIkSolver, RecoversKr4ConfigurationDespiteRotatedOrigins)
{
    const auto chain = loadChain("kr4_r600.urdf");
    const std::vector<double> q{0.5, -0.7, 0.8, 0.6, -0.8, 1.2};
    const auto result = ik::AnalyticIkSolver{}.solve(chain, targetFor(chain, q));
    ASSERT_TRUE(result);
    EXPECT_EQ(result->status, ik::AnalyticIkStatus::Solutions);
    EXPECT_TRUE(containsConfiguration(*result, q));
}

TEST(AnalyticIkSolver, ReturnsMultipleBranchesWhenGeometryAllowsThem)
{
    const auto chain = loadChain("kr640.urdf");
    const std::vector<double> q{0.4, -0.5, 0.7, 0.8, -0.9, 1.1};
    const auto result = ik::AnalyticIkSolver{}.solve(chain, targetFor(chain, q));
    ASSERT_TRUE(result);
    EXPECT_GE(result->branches.size(), 2U);
}

TEST(AnalyticIkSolver, WristSingularityIsAnInfiniteFamilyNotAnArbitraryValue)
{
    const auto chain = loadChain("kr640.urdf");
    const std::vector<double> q{0.4, -0.5, 0.7, 0.8, 0.0, 1.1};
    const auto result = ik::AnalyticIkSolver{}.solve(chain, targetFor(chain, q));
    ASSERT_TRUE(result);
    EXPECT_EQ(result->status, ik::AnalyticIkStatus::InfiniteFamily);

    const auto singular = std::find_if(result->branches.begin(), result->branches.end(),
        [](const auto& branch) { return branch.wrist == ik::WristBranch::Singular; });
    ASSERT_TRUE(singular != result->branches.end());
    ASSERT_FALSE(singular->freeParameters.empty());
    ASSERT_FALSE(singular->coupledFreeParameters.empty());
    EXPECT_EQ(singular->singularity, ik::IkSingularityKind::WristAligned);
}

TEST(AnalyticIkSolver, SingularRelationContainsOriginalWristConfiguration)
{
    const auto chain = loadChain("kr640.urdf");
    const std::vector<double> q{0.4, -0.5, 0.7, 0.8, 0.0, 1.1};
    const auto result = ik::AnalyticIkSolver{}.solve(chain, targetFor(chain, q));
    ASSERT_TRUE(result);

    bool found = false;
    for (const auto& branch : result->branches)
        for (const auto& relation : branch.coupledFreeParameters)
        {
            const double dependent = relation.offset + relation.multiplier *
                                     q[relation.freeActuatedJointPosition];
            if (std::abs(ik::normalizeAngle(dependent -
                                            q[relation.dependentActuatedJointPosition])) < 1e-7)
                found = true;
        }
    EXPECT_TRUE(found);
}

TEST(AnalyticIkSolver, UnreachableTargetReturnsNoSolution)
{
    const auto chain = loadChain("kr640.urdf");
    const ik::RotationMatrix3 rotation{{{{1.0, 0.0, 0.0},
                                          {0.0, 1.0, 0.0},
                                          {0.0, 0.0, 1.0}}}};
    const ik::PoseTarget target{{100.0, 100.0, 100.0}, rotation};
    const auto result = ik::AnalyticIkSolver{}.solve(chain, target);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->status, ik::AnalyticIkStatus::NoSolution);
    EXPECT_TRUE(result->branches.empty());
}

TEST(AnalyticIkSolver, RejectsInvalidOrientationBeforeSolving)
{
    const auto chain = loadChain("kr640.urdf");
    const ik::RotationMatrix3 rotation{{{{2.0, 0.0, 0.0},
                                          {0.0, 1.0, 0.0},
                                          {0.0, 0.0, 1.0}}}};
    ik::PoseTarget target{{}, rotation};
    const auto result = ik::AnalyticIkSolver{}.solve(chain, target);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, ik::AnalyticIkSolverErrorCode::InvalidTarget);
    EXPECT_TRUE(result.error().targetError.has_value());
}

TEST(AnalyticIkSolver, RespectsJointLimits)
{
    auto chain = loadChain("kr640.urdf");
    const std::vector<double> q{0.4, -0.5, 0.7, 0.8, -0.9, 1.1};
    const auto target = targetFor(chain, q);

    // Make q1 impossible while preserving valid geometry.
    for (auto& joint : chain.joints)
        if (joint.variable && joint.variable->index == 1)
        {
            joint.limits.lower = 1.5;
            joint.limits.upper = 1.6;
            joint.limits.hasPositionLimits = true;
        }

    const auto result = ik::AnalyticIkSolver{}.solve(chain, target);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->status, ik::AnalyticIkStatus::NoSolution);
    EXPECT_TRUE(result->branches.empty());
}

TEST(AnalyticIkSolver, RejectsPrismaticSixJointChain)
{
    auto chain = loadChain("kr640.urdf");
    for (auto& joint : chain.joints)
        if (joint.variable && joint.variable->index == 2)
            joint.type = ik::JointType::Prismatic;

    const auto result = ik::AnalyticIkSolver{}.solve(chain, targetFor(loadChain("kr640.urdf"),
        {0.2, -0.3, 0.4, 0.5, -0.6, 0.7}));
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, ik::AnalyticIkSolverErrorCode::UnsupportedKinematicStructure);
}

TEST(AnalyticIkSolver, RandomKr640RoundTripsRecoverGeneratingBranch)
{
    const auto chain = loadChain("kr640.urdf");
    const auto joints = actuated(chain);
    std::mt19937_64 random{0x640};

    for (int sample = 0; sample < 100; ++sample)
    {
        std::vector<double> q;
        for (const auto* joint : joints)
        {
            const double margin = 0.02 * (joint->limits.upper - joint->limits.lower);
            std::uniform_real_distribution<double> value(joint->limits.lower + margin,
                                                         joint->limits.upper - margin);
            q.push_back(value(random));
        }
        if (std::abs(q[4]) < 1e-4)
            q[4] += 0.01;

        const auto result = ik::AnalyticIkSolver{}.solve(chain, targetFor(chain, q));
        ASSERT_TRUE(result);
        EXPECT_TRUE(containsConfiguration(*result, q));
    }
}

TEST(AnalyticIkSolver, RandomKr4RoundTripsRecoverGeneratingBranch)
{
    const auto chain = loadChain("kr4_r600.urdf");
    const auto joints = actuated(chain);
    std::mt19937_64 random{0x4};

    for (int sample = 0; sample < 100; ++sample)
    {
        std::vector<double> q;
        for (const auto* joint : joints)
        {
            const double margin = 0.02 * (joint->limits.upper - joint->limits.lower);
            std::uniform_real_distribution<double> value(joint->limits.lower + margin,
                                                         joint->limits.upper - margin);
            q.push_back(value(random));
        }
        if (std::abs(q[4]) < 1e-4)
            q[4] += 0.01;

        const auto result = ik::AnalyticIkSolver{}.solve(chain, targetFor(chain, q));
        ASSERT_TRUE(result);
        EXPECT_TRUE(containsConfiguration(*result, q));
    }
}


TEST(AnalyticIkSolver, FacadeSolvesSelectedKr640Chain)
{
    ik::IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / "kr640.urdf"));
    ASSERT_TRUE(builder.selectChain("base_link", "tool0"));

    const auto chain = loadChain("kr640.urdf");
    const std::vector<double> q{0.4, -0.5, 0.7, 0.8, -0.9, 1.1};
    const auto result = builder.solveAnalyticIk(targetFor(chain, q));
    ASSERT_TRUE(result);
    EXPECT_TRUE(containsConfiguration(*result, q));
}

TEST(AnalyticIkSolver, FacadePreservesNestedAnalyticError)
{
    ik::IkEquationBuilder builder;
    ASSERT_TRUE(builder.loadRobotModel(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / "kr640.urdf"));
    ASSERT_TRUE(builder.selectChain("base_link", "tool0"));

    const ik::RotationMatrix3 rotation{{{{2.0, 0.0, 0.0},
                                          {0.0, 1.0, 0.0},
                                          {0.0, 0.0, 1.0}}}};
    const auto result = builder.solveAnalyticIk(ik::PoseTarget{{}, rotation});
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, ik::IkEquationBuilderErrorCode::AnalyticIkSolveFailed);
    ASSERT_TRUE(result.error().analyticIkError.has_value());
    EXPECT_EQ(result.error().analyticIkError->code, ik::AnalyticIkSolverErrorCode::InvalidTarget);
}
