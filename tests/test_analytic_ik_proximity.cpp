#include <gtest/gtest.h>

#include "fresh_generated/kr640_fresh.hpp"
#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/solver/AnalyticIkSolver.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <cmath>
#include <filesystem>
#include <random>
#include <vector>

// How close a result is to a singularity (MotionBridge report 2026-11-21, 5.3):
// the solver says it, so a caller does not have to guess a threshold from a
// 1e-9 m switch it cannot see.
//
//   shoulderAxisDistance   target property, metres: distance of the wrist
//                          centre from the joint-1 axis. 0 = q1 is free.
//   wristPitchSine(branch) per branch: |sin q5|. 0 = wrist family (q4/q6 couple).

namespace ik = kinemaforge::ik;
namespace gen = kinemaforge::ik::generated;
namespace ts = kinemaforge::testsupport;

namespace {

ik::KinematicChain kr640Chain()
{
    return ik::KinematicChainBuilder{}
        .build(ik::UrdfModelLoader{}.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / "kr640.urdf"),
               "base_link", "tool0")
        .value();
}

ik::PoseTarget poseOf(const ik::KinematicChain& chain, const std::vector<double>& q)
{
    const auto fk = ts::numericForwardKinematics(chain, q);
    return {{fk.translation.x, fk.translation.y, fk.translation.z}, {ts::toRotationMatrix(fk.rotation)}};
}

gen::Pose generatedPose(const ik::PoseTarget& target)
{
    return {{target.position.x, target.position.y, target.position.z}, target.orientation.values};
}

// Independent of the solvers: the wrist centre from the target pose, and its
// distance from the (vertical) joint-1 axis of the KR640.
double referenceShoulderAxisDistance(const ik::PoseTarget& target)
{
    const auto& p = kr640_fresh::parameters;
    const auto& r = target.orientation.values;
    const auto& w = p.wristInTargetFrame;
    const double x = target.position.x + r[0][0] * w.x + r[0][1] * w.y + r[0][2] * w.z - p.axes[0].point.x;
    const double y = target.position.y + r[1][0] * w.x + r[1][1] * w.y + r[1][2] * w.z - p.axes[0].point.y;
    return std::hypot(x, y);
}

} // namespace

TEST(AnalyticIkProximity, ShoulderAxisDistanceMatchesAnIndependentComputation)
{
    const auto chain = kr640Chain();
    std::mt19937_64 rng(0xD157);
    for (int n = 0; n < 50; ++n)
    {
        std::vector<double> q;
        for (const auto& joint : chain.joints)
            if (joint.type != ik::JointType::Fixed)
                q.push_back(joint.limits.lower + std::uniform_real_distribution<double>(0.05, 0.95)(rng) *
                                                     (joint.limits.upper - joint.limits.lower));
        const auto target = poseOf(chain, q);
        const double expected = referenceShoulderAxisDistance(target);

        const auto solved = ik::AnalyticIkSolver{}.solve(chain, target);
        const auto compact = ik::AnalyticIkSolver{}.solveCompact(chain, target);
        ASSERT_TRUE(solved && compact);
        EXPECT_NEAR(solved->shoulderAxisDistance, expected, 1e-9);
        EXPECT_NEAR(compact->shoulderAxisDistance, expected, 1e-9);
        EXPECT_NEAR(gen::solve(kr640_fresh::parameters, generatedPose(target)).shoulderAxisDistance, expected, 1e-9);
        EXPECT_NEAR(gen::solveCompact(kr640_fresh::parameters, generatedPose(target)).shoulderAxisDistance,
                    expected, 1e-9);
    }
}

TEST(AnalyticIkProximity, ShoulderAxisDistanceIsZeroOnTheAxis)
{
    // A target whose wrist centre is on the axis exactly: the home pose of the
    // KR640 with the arm folded back is not available, so build the pose from
    // a position that is on the axis by construction.
    const auto chain = kr640Chain();
    const auto& p = kr640_fresh::parameters;
    ik::PoseTarget target{{0.0, 0.0, 0.0}, {{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}}};
    // wrist = position + R * wristInTargetFrame; choose position so wrist.xy = axis.xy.
    target.position.x = p.axes[0].point.x - p.wristInTargetFrame.x;
    target.position.y = p.axes[0].point.y - p.wristInTargetFrame.y;
    target.position.z = 1.0;
    const auto solved = ik::AnalyticIkSolver{}.solve(chain, target);
    ASSERT_TRUE(solved) << solved.error().message;
    EXPECT_NEAR(solved->shoulderAxisDistance, 0.0, 1e-12);
}

TEST(AnalyticIkProximity, WristPitchSineIsTheAbsoluteSineOfJointFive)
{
    const auto chain = kr640Chain();
    const auto target = poseOf(chain, {0.4, -0.5, 0.7, 0.8, -0.9, 1.1});
    const auto solved = ik::AnalyticIkSolver{}.solve(chain, target);
    ASSERT_TRUE(solved && !solved->branches.empty());
    for (const auto& branch : solved->branches)
    {
        double q5 = 0.0;
        for (const auto& joint : branch.joints)
            if (joint.actuatedJointPosition == 4)
                q5 = joint.value;
        EXPECT_NEAR(ik::wristPitchSine(branch), std::abs(std::sin(q5)), 1e-15);
        EXPECT_GT(ik::wristPitchSine(branch), 0.0);
    }
    for (const auto& branch : gen::solve(kr640_fresh::parameters, generatedPose(target)).branches)
        EXPECT_NEAR(gen::wristPitchSine(branch), std::abs(std::sin(branch.q[4])), 1e-15);
}

TEST(AnalyticIkProximity, WristPitchSineIsZeroForAWristFamily)
{
    const auto chain = kr640Chain();
    const auto target = poseOf(chain, {0.4, -0.5, 0.7, 0.8, 0.0, 1.1});
    const auto solved = ik::AnalyticIkSolver{}.solve(chain, target);
    ASSERT_TRUE(solved);
    ASSERT_EQ(solved->status, ik::AnalyticIkStatus::InfiniteFamily);
    int families = 0;
    for (const auto& branch : solved->branches)
        if (!branch.freeParameters.empty())
        {
            ++families;
            EXPECT_EQ(ik::wristPitchSine(branch), 0.0);
        }
    EXPECT_GT(families, 0);
    for (const auto& branch : gen::solve(kr640_fresh::parameters, generatedPose(target)).branches)
    {
        if (branch.singular)
        {
            EXPECT_EQ(gen::wristPitchSine(branch), 0.0);
        }
    }
}
