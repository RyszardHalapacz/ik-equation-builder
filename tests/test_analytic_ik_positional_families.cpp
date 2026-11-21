#include <gtest/gtest.h>

#include "fresh_generated/equal_links_fresh.hpp"
#include "fresh_generated/kr4_fresh.hpp"
#include "fresh_generated/kr640_fresh.hpp"
#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/solver/AnalyticIkSolver.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <array>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <numeric>
#include <optional>
#include <vector>

// Positional singular families (follow-up 2026-11-16 #1).
//
// Wrist centre on the joint-1 axis: q1 is free. Equal arm links with the elbow
// fully folded: q2 is free. In both cases the wrist centre does not move with
// the free angle, the other arm joints are fixed, and q4..q6 follow from the
// wrist decomposition -- nonlinearly. Both solvers return such a family as a
// PositionalFamily plus evaluate(family, value) instead of reporting the case
// as unsupported.

namespace ik = kinemaforge::ik;
namespace gen = kinemaforge::ik::generated;
namespace ts = kinemaforge::testsupport;

namespace {

constexpr double kPeriod = 2.0 * std::numbers::pi_v<double>;
using Configuration = std::array<double, 6>;

struct Robot
{
    ik::KinematicChain chain;
    gen::Parameters parameters;

    Robot(const std::filesystem::path& urdf, const gen::Parameters& p) : parameters(p)
    {
        chain = ik::KinematicChainBuilder{}.build(ik::UrdfModelLoader{}.load(urdf), "base_link", "tool0").value();
    }

    void limit(std::size_t position, double lower, double upper)
    {
        for (auto& joint : chain.joints)
            if (joint.type != ik::JointType::Fixed && position-- == 0)
            {
                joint.limits.hasPositionLimits = true;
                joint.limits.lower = lower;
                joint.limits.upper = upper;
                break;
            }
    }

    void limitBoth(std::size_t position, double lower, double upper)
    {
        limit(position, lower, upper);
        parameters.limits[position] = {true, lower, upper};
    }

    ik::PoseTarget pose(const Configuration& q) const
    {
        const auto fk = ts::numericForwardKinematics(chain, std::vector<double>(q.begin(), q.end()));
        return {{fk.translation.x, fk.translation.y, fk.translation.z}, {ts::toRotationMatrix(fk.rotation)}};
    }
};

Robot kr640()
{
    return {std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / "kr640.urdf", kr640_fresh::parameters};
}

Robot equalLinks()
{
    return {std::filesystem::path(KINEMAFORGE_TEST_DATA_DIR) / "equal_links_6r.urdf",
            equal_links_fresh::parameters};
}

gen::Pose generatedPose(const ik::PoseTarget& target)
{
    return {{target.position.x, target.position.y, target.position.z}, target.orientation.values};
}

// An in-limit KR640 configuration with the wrist centre on the joint-1 axis.
Configuration shoulderSingularConfiguration(const Robot& robot)
{
    const auto& p = robot.parameters;
    const auto offset = [&](const Configuration& q) {
        const auto target = robot.pose(q);
        const auto& r = target.orientation.values;
        const auto& w = p.wristInTargetFrame;
        return target.position.x + r[0][0] * w.x + r[0][1] * w.y + r[0][2] * w.z - p.axes[0].point.x;
    };
    for (double q2 = -1.6; q2 <= 0.4; q2 += 0.1)
        for (double q3 = -2.0; q3 < 2.6; q3 += 0.05)
        {
            Configuration a{0.3, q2, q3, 0.4, 0.7, -0.2};
            Configuration b = a;
            b[2] = q3 + 0.05;
            if ((offset(a) < 0.0) == (offset(b) < 0.0))
                continue;
            for (int i = 0; i < 200; ++i)
            {
                Configuration m = a;
                m[2] = std::midpoint(a[2], b[2]);
                ((offset(m) < 0.0) == (offset(a) < 0.0) ? a : b) = m;
            }
            if (std::abs(offset(a)) < 1e-12)
                return a;
        }
    ADD_FAILURE() << "no shoulder-singular configuration found";
    return {};
}

double angularDistance(double a, double b)
{
    return std::abs(std::remainder(a - b, kPeriod));
}

bool sameConfiguration(const Configuration& a, const Configuration& b, double tolerance)
{
    for (std::size_t i = 0; i < 6; ++i)
        if (angularDistance(a[i], b[i]) > tolerance)
            return false;
    return true;
}

void expectReaches(const Robot& robot, const Configuration& q, const ik::PoseTarget& target)
{
    const auto reached = robot.pose(q);
    EXPECT_NEAR(reached.position.x, target.position.x, 2e-9);
    EXPECT_NEAR(reached.position.y, target.position.y, 2e-9);
    EXPECT_NEAR(reached.position.z, target.position.z, 2e-9);
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(reached.orientation.values[i][j], target.orientation.values[i][j], 2e-9);
}

// The families contain the generating configuration and every evaluated
// member is in limits and reaches the target.
void checkFamilies(const Robot& robot, const std::vector<ik::PositionalFamily>& families,
                   std::size_t freeJoint, const Configuration& q, const ik::PoseTarget& target)
{
    ASSERT_FALSE(families.empty());
    bool containsGenerating = false;
    for (const auto& family : families)
    {
        EXPECT_EQ(family.freeJoint, freeJoint);
        EXPECT_LE(family.freeLower, family.freeUpper);
        if (const auto member = ik::evaluate(family, q[freeJoint]))
            containsGenerating = containsGenerating || sameConfiguration(*member, q, 1e-7);
        int evaluated = 0;
        for (int step = 0; step <= 8; ++step)
        {
            const double value = std::lerp(family.freeLower, family.freeUpper, step / 8.0);
            const auto member = ik::evaluate(family, value);
            if (!member)
                continue;
            ++evaluated;
            EXPECT_EQ((*member)[freeJoint], value);
            for (std::size_t i = 0; i < 6; ++i)
                if (family.limits[i].enabled)
                {
                    EXPECT_GE((*member)[i], family.limits[i].lower);
                    EXPECT_LE((*member)[i], family.limits[i].upper);
                }
            expectReaches(robot, *member, target);
        }
        EXPECT_GT(evaluated, 0);
    }
    EXPECT_TRUE(containsGenerating);
}

} // namespace

TEST(AnalyticIkPositionalFamilies, ShoulderAxisFamilyFromTheGeneralSolver)
{
    Robot robot = kr640();
    const Configuration q = shoulderSingularConfiguration(robot);
    const auto target = robot.pose(q);
    const auto solved = ik::AnalyticIkSolver{}.solve(robot.chain, target);
    ASSERT_TRUE(solved) << solved.error().message;
    EXPECT_EQ(solved->status, ik::AnalyticIkStatus::InfiniteFamily);
    std::vector<ik::PositionalFamily> families;
    for (const auto& branch : solved->positionalFamilies)
    {
        EXPECT_EQ(branch.singularity, ik::IkSingularityKind::ShoulderAxis);
        families.push_back(branch.family);
    }
    checkFamilies(robot, families, 0, q, target);

    const auto compact = ik::AnalyticIkSolver{}.solveCompact(robot.chain, target);
    ASSERT_TRUE(compact) << compact.error().message;
    EXPECT_EQ(compact->status, ik::AnalyticIkStatus::InfiniteFamily);
    EXPECT_EQ(compact->positionalFamilies.size(), solved->positionalFamilies.size());
}

TEST(AnalyticIkPositionalFamilies, ShoulderAxisFamilyFromTheGeneratedSolver)
{
    Robot robot = kr640();
    const Configuration q = shoulderSingularConfiguration(robot);
    const auto target = robot.pose(q);
    const auto compiled = gen::solve(robot.parameters, generatedPose(target));
    EXPECT_EQ(compiled.status, gen::Status::InfiniteFamily);
    checkFamilies(robot, compiled.positionalFamilies, 0, q, target);
    EXPECT_EQ(gen::solveCompact(robot.parameters, generatedPose(target)).positionalFamilies.size(),
              compiled.positionalFamilies.size());
}

TEST(AnalyticIkPositionalFamilies, ShoulderFamilyRespectsTightLimitsOnTheWrist)
{
    // Limits around the generating configuration: members far from it leave
    // the wrist limits and evaluate() says so, but the family survives.
    Robot robot = kr640();
    const Configuration q = shoulderSingularConfiguration(robot);
    for (std::size_t i = 0; i < 6; ++i)
        robot.limitBoth(i, q[i] - 0.3, q[i] + 0.3);
    const auto target = robot.pose(q);
    const auto solved = ik::AnalyticIkSolver{}.solve(robot.chain, target);
    ASSERT_TRUE(solved) << solved.error().message;
    std::vector<ik::PositionalFamily> families;
    for (const auto& branch : solved->positionalFamilies)
        families.push_back(branch.family);
    checkFamilies(robot, families, 0, q, target);
    const auto compiled = gen::solve(robot.parameters, generatedPose(target));
    checkFamilies(robot, compiled.positionalFamilies, 0, q, target);
}

TEST(AnalyticIkPositionalFamilies, FoldedEqualLinksFamilyFromBothSolvers)
{
    Robot robot = equalLinks();
    const double folded = std::remainder(
        (std::numbers::pi_v<double> - robot.parameters.link2HomeAngle) / robot.parameters.axis3Sign, kPeriod);
    const Configuration q{0.4, -0.5, folded, 0.8, -0.9, 1.1};
    for (std::size_t i = 0; i < 6; ++i)
        robot.limitBoth(i, q[i] - 0.5, q[i] + 0.5);
    const auto target = robot.pose(q);

    const auto solved = ik::AnalyticIkSolver{}.solve(robot.chain, target);
    ASSERT_TRUE(solved) << solved.error().message;
    EXPECT_EQ(solved->status, ik::AnalyticIkStatus::InfiniteFamily);
    std::vector<ik::PositionalFamily> families;
    for (const auto& branch : solved->positionalFamilies)
    {
        EXPECT_EQ(branch.singularity, ik::IkSingularityKind::ElbowFolded);
        families.push_back(branch.family);
    }
    checkFamilies(robot, families, 1, q, target);

    const auto compiled = gen::solve(robot.parameters, generatedPose(target));
    EXPECT_EQ(compiled.status, gen::Status::InfiniteFamily);
    checkFamilies(robot, compiled.positionalFamilies, 1, q, target);
}

TEST(AnalyticIkPositionalFamilies, RegularTargetsHaveNoPositionalFamilies)
{
    Robot robot = kr640();
    const auto target = robot.pose({0.4, -0.5, 0.7, 0.8, -0.9, 1.1});
    const auto solved = ik::AnalyticIkSolver{}.solve(robot.chain, target);
    ASSERT_TRUE(solved);
    EXPECT_TRUE(solved->positionalFamilies.empty());
    EXPECT_TRUE(gen::solve(robot.parameters, generatedPose(target)).positionalFamilies.empty());
}

// The band between "ordinary" and "exactly on the axis" (MotionBridge report
// 2026-11-21, 5.3). A work-space map has cells at millimetres from the joint-1
// axis, not only on it. Measured on KR640: the switch to a family happens at
// kSingularLinear (1e-9 m); on either side the generating configuration is
// recovered, so the result is continuous in what it represents and there is no
// band in which a branch drops out.
namespace {

// Signed distance of the wrist centre from the joint-1 axis along x.
double wristOffsetX(const Robot& robot, const Configuration& q)
{
    const auto& p = robot.parameters;
    const auto target = robot.pose(q);
    const auto& r = target.orientation.values;
    const auto& w = p.wristInTargetFrame;
    return target.position.x + r[0][0] * w.x + r[0][1] * w.y + r[0][2] * w.z - p.axes[0].point.x;
}

// The singular configuration moved along q3 until the wrist centre is `distance` from the axis.
Configuration atDistanceFromAxis(const Robot& robot, const Configuration& singular, double distance)
{
    double lo = 0.0;
    double hi = 0.2;
    for (int i = 0; i < 200; ++i)
    {
        const double mid = 0.5 * (lo + hi);
        Configuration q = singular;
        q[2] += mid;
        (std::abs(wristOffsetX(robot, q)) < distance ? lo : hi) = mid;
    }
    Configuration q = singular;
    q[2] += hi;
    return q;
}

bool regularBranchesContain(const ik::AnalyticIkSolution& solved, const Configuration& q, double tolerance)
{
    for (const auto& branch : solved.branches)
    {
        double worst = 0.0;
        for (const auto& joint : branch.joints)
            worst = std::max(worst, angularDistance(joint.value, q[joint.actuatedJointPosition]));
        if (worst <= tolerance)
            return true;
    }
    return false;
}

} // namespace

TEST(AnalyticIkPositionalFamilies, NearTheShoulderAxisTheGeneratingConfigurationIsAlwaysRecovered)
{
    Robot robot = kr640();
    const Configuration singular = shoulderSingularConfiguration(robot);

    // Millimetres from the axis, as in a work-space map: ordinary branches, no
    // family, and the generating configuration among them (q1 is defined by the
    // target, so it is accurate to well below 1e-6).
    for (const double distance : {1e-6, 1e-4, 1e-3, 5e-3, 2e-2})
    {
        const Configuration q = atDistanceFromAxis(robot, singular, distance);
        const auto target = robot.pose(q);
        const auto solved = ik::AnalyticIkSolver{}.solve(robot.chain, target);
        ASSERT_TRUE(solved) << distance << ": " << solved.error().message;
        EXPECT_EQ(solved->status, ik::AnalyticIkStatus::Solutions) << distance;
        EXPECT_TRUE(solved->positionalFamilies.empty()) << distance;
        EXPECT_TRUE(regularBranchesContain(*solved, q, 1e-6)) << distance;
        const auto compiled = gen::solve(robot.parameters, generatedPose(target));
        EXPECT_EQ(compiled.status, gen::Status::Solutions) << distance;
        EXPECT_TRUE(compiled.positionalFamilies.empty()) << distance;
    }

    // At and below the singularity tolerance: a family, containing the same configuration.
    for (const double distance : {0.0, 1e-12, 1e-10})
    {
        const Configuration q = distance == 0.0 ? singular : atDistanceFromAxis(robot, singular, distance);
        const auto target = robot.pose(q);
        const auto solved = ik::AnalyticIkSolver{}.solve(robot.chain, target);
        ASSERT_TRUE(solved) << distance << ": " << solved.error().message;
        EXPECT_EQ(solved->status, ik::AnalyticIkStatus::InfiniteFamily) << distance;
        std::vector<ik::PositionalFamily> families;
        for (const auto& branch : solved->positionalFamilies)
            families.push_back(branch.family);
        bool contained = false;
        for (const auto& family : families)
            if (const auto member = ik::evaluate(family, q[0]))
                contained = contained || sameConfiguration(*member, q, 1e-6);
        EXPECT_TRUE(contained) << distance;
        EXPECT_EQ(gen::solve(robot.parameters, generatedPose(target)).status, gen::Status::InfiniteFamily)
            << distance;
    }
}
