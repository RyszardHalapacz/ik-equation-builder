#include <gtest/gtest.h>

#include "fresh_generated/equal_links_fresh.hpp"
#include "fresh_generated/kr4_fresh.hpp"
#include "fresh_generated/kr640_fresh.hpp"
#include "fresh_generated/kr640_tcp_fresh.hpp"
#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/solver/AnalyticIkSolver.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <random>
#include <stdexcept>
#include <vector>

namespace ik = kinemaforge::ik;
namespace gen = ik::generated;
namespace ts = kinemaforge::testsupport;

namespace {
constexpr double kPeriod = 2.0 * std::numbers::pi_v<double>;
using Configuration = std::array<double, 6>;

// Only translates the solution IR; all numerical checks below use the
// quaternion FK reference, independently of either solver's matrix arithmetic.
std::vector<gen::Branch> numericBranches(const ik::AnalyticIkSolution& result)
{
    std::vector<gen::Branch> branches;
    for (const auto& source : result.branches)
    {
        gen::Branch branch;
        for (const auto& joint : source.joints)
            branch.q[joint.actuatedJointPosition] = joint.value;
        if (!source.freeParameters.empty())
        {
            branch.singular = true;
            branch.freeLower = source.freeParameters.front().lower;
            branch.freeUpper = source.freeParameters.front().upper;
            branch.dependentOffset = source.coupledFreeParameters.front().offset;
            branch.dependentMultiplier = source.coupledFreeParameters.front().multiplier;
        }
        branches.push_back(branch);
    }
    return branches;
}

class AnalyticIkRegression : public testing::TestWithParam<int>
{
protected:
    ik::KinematicChain chain;
    gen::Parameters parameters;
    std::optional<ik::FixedRigidTransform> tcp;

    void SetUp() override
    {
        const bool kr4 = GetParam() == 1;
        const auto robot = ik::UrdfModelLoader{}.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) /
                                                     (kr4 ? "kr4_r600.urdf" : "kr640.urdf"));
        const auto built = ik::KinematicChainBuilder{}.build(robot, "base_link", "tool0");
        ASSERT_TRUE(built);
        chain = *built;
        parameters = kr4 ? kr4_fresh::parameters : kr640_fresh::parameters;
        if (GetParam() == 2)
        {
            tcp = ik::FixedRigidTransform{{0.03, -0.04, 0.12}, {0.2, -0.1, 0.3}};
            parameters = kr640_tcp_fresh::parameters;
        }
    }

    ik::KinematicJoint& joint(std::size_t position)
    {
        for (auto& entry : chain.joints)
            if (entry.type != ik::JointType::Fixed && position-- == 0)
                return entry;
        throw std::logic_error("missing test joint");
    }

    void limit(std::size_t position, double lower, double upper)
    {
        auto& entry = joint(position);
        entry.limits.hasPositionLimits = true;
        entry.limits.lower = lower;
        entry.limits.upper = upper;
        parameters.limits[position] = {true, lower, upper};
    }

    void restrictAround(const Configuration& q, double margin = 0.05)
    {
        for (std::size_t i = 0; i < q.size(); ++i)
            limit(i, q[i] - margin, q[i] + margin);
    }

    ik::PoseTarget pose(const Configuration& q) const
    {
        const std::vector<double> values(q.begin(), q.end());
        const auto fk = tcp ? ts::numericTcpForwardKinematics(chain, values, *tcp)
                            : ts::numericForwardKinematics(chain, values);
        return {{fk.translation.x, fk.translation.y, fk.translation.z},
                {ts::toRotationMatrix(fk.rotation)}};
    }

    bool contains(const std::vector<gen::Branch>& branches, const Configuration& q) const
    {
        return std::any_of(branches.begin(), branches.end(), [&](const auto& branch) {
            auto values = branch.q;
            if (branch.singular)
            {
                if (q[3] < branch.freeLower - 1e-9 || q[3] > branch.freeUpper + 1e-9)
                    return false;
                values[3] = q[3];
                values[5] = branch.dependentOffset + branch.dependentMultiplier * q[3];
            }
            for (std::size_t i = 0; i < q.size(); ++i)
            {
                double difference = values[i] - q[i];
                if (!parameters.limits[i].enabled)
                    difference = std::remainder(difference, kPeriod);
                if (std::abs(difference) > 3e-7)
                    return false;
            }
            return true;
        });
    }

    void checkBranches(const std::vector<gen::Branch>& branches, const ik::PoseTarget& target) const
    {
        ASSERT_FALSE(branches.empty());
        for (const auto& branch : branches)
        {
            if (branch.singular)
            {
                ASSERT_LE(branch.freeLower, branch.freeUpper);
            }
            for (double fraction : {0.0, 0.25, 0.5, 0.75, 1.0})
            {
                auto q = branch.q;
                if (branch.singular)
                {
                    q[3] = std::lerp(branch.freeLower, branch.freeUpper, fraction);
                    q[5] = branch.dependentOffset + branch.dependentMultiplier * q[3];
                }
                for (std::size_t i = 0; i < q.size(); ++i)
                {
                    ASSERT_TRUE(std::isfinite(q[i]));
                    if (parameters.limits[i].enabled)
                    {
                        EXPECT_GE(q[i], parameters.limits[i].lower - 2e-11);
                        EXPECT_LE(q[i], parameters.limits[i].upper + 2e-11);
                    }
                }
                const auto actual = pose(q);
                EXPECT_NEAR(actual.position.x, target.position.x, 2e-9);
                EXPECT_NEAR(actual.position.y, target.position.y, 2e-9);
                EXPECT_NEAR(actual.position.z, target.position.z, 2e-9);
                for (std::size_t i = 0; i < 3; ++i)
                    for (std::size_t j = 0; j < 3; ++j)
                        EXPECT_NEAR(actual.orientation.values[i][j], target.orientation.values[i][j], 2e-9);
                if (!branch.singular)
                    break;
            }
        }
    }

    void checkBoth(const Configuration& q, bool requireOriginal = true,
                   bool requireFamily = false) const
    {
        const auto target = pose(q);
        const auto solved = ik::AnalyticIkSolver{}.solve(chain, target, tcp);
        ASSERT_TRUE(solved) << solved.error().message;
        const auto compiled = gen::solve(parameters, {{target.position.x, target.position.y, target.position.z},
                                                      target.orientation.values});
        const std::array results{numericBranches(*solved), compiled.branches};
        for (std::size_t solver = 0; solver < results.size(); ++solver)
        {
            SCOPED_TRACE(solver == 0 ? "general solver" : "freshly generated solver");
            checkBranches(results[solver], target);
            if (requireOriginal)
            {
                EXPECT_TRUE(contains(results[solver], q));
            }
            if (requireFamily)
            {
                EXPECT_TRUE(std::any_of(results[solver].begin(), results[solver].end(),
                                       [](const auto& branch) { return branch.singular; }));
            }
        }
        if (requireFamily)
        {
            EXPECT_EQ(solved->status, ik::AnalyticIkStatus::InfiniteFamily);
            EXPECT_EQ(compiled.status, gen::Status::InfiniteFamily);
        }
    }
};

TEST_P(AnalyticIkRegression, RandomSingularTargetsKeepTheirFamiliesAndEveryBranchMatchesFk)
{
    std::mt19937_64 random{20261115};
    for (int sample = 0; sample < 1000; ++sample)
    {
        SCOPED_TRACE(sample);
        Configuration q{};
        for (std::size_t i = 0; i < q.size(); ++i)
            q[i] = std::uniform_real_distribution<double>(parameters.limits[i].lower + 0.01,
                                                          parameters.limits[i].upper - 0.01)(random);
        q[4] = 0.0;
        checkBoth(q, true, true);
        if (HasFailure()) return;
    }
}

TEST_P(AnalyticIkRegression, NearAlignedAndAntiAlignedWristsRemainReachable)
{
    for (double center : {0.0, std::numbers::pi_v<double>, -std::numbers::pi_v<double>})
        for (double delta : {0.0, -1e-12, 1e-12, -1e-9, 1e-9, -1e-8, 1e-8, -1e-6, 1e-6})
        {
            SCOPED_TRACE(testing::Message() << "center=" << center << " delta=" << delta);
            const Configuration q{0.4, -0.5, 0.7, 0.8, center + delta, 1.1};
            restrictAround(q);
            // Individual outer angles are ill-conditioned near singularity;
            // reachability and FK of every returned branch are the contract.
            checkBoth(q, false, delta == 0.0);
        }
}

TEST_P(AnalyticIkRegression, RandomAntiAlignedTargetsKeepTheirFamiliesAndEveryBranchMatchesFk)
{
    limit(4, -std::numbers::pi_v<double> - 0.1, std::numbers::pi_v<double> + 0.1);
    std::mt19937_64 random{20261116};
    for (int sample = 0; sample < 200; ++sample)
    {
        SCOPED_TRACE(sample);
        Configuration q{};
        for (std::size_t i = 0; i < q.size(); ++i)
            q[i] = std::uniform_real_distribution<double>(parameters.limits[i].lower + 0.01,
                                                          parameters.limits[i].upper - 0.01)(random);
        q[4] = sample % 2 == 0 ? std::numbers::pi_v<double> : -std::numbers::pi_v<double>;
        checkBoth(q, true, true);
        if (HasFailure()) return;
    }
}

TEST_P(AnalyticIkRegression, PreservesAllInLimitFullTurnsOfGeneratingConfiguration)
{
    for (double q4 : {0.8 - kPeriod, 0.8, 0.8 + kPeriod})
        for (double q6 : {1.1 - kPeriod, 1.1, 1.1 + kPeriod})
        {
            if (q4 < parameters.limits[3].lower || q4 > parameters.limits[3].upper ||
                q6 < parameters.limits[5].lower || q6 > parameters.limits[5].upper)
                continue;
            checkBoth({0.4, -0.5, 0.7, q4, -0.9, q6});
        }
}

TEST_P(AnalyticIkRegression, SingularCouplingUsesActualLimitsAcrossManyTurns)
{
    for (double q5 : {0.0, std::numbers::pi_v<double>})
        for (double freeTurns : {-4.0, 0.0, 5.0})
            for (double dependentTurns : {-5.0, 3.0, 7.0})
            {
                SCOPED_TRACE(testing::Message() << q5 << ", " << freeTurns << ", " << dependentTurns);
                const Configuration q{0.4, -0.5, 0.7, 0.8 + freeTurns * kPeriod, q5,
                                       1.1 + dependentTurns * kPeriod};
                restrictAround(q);
                checkBoth(q, true, true);
            }
}

TEST_P(AnalyticIkRegression, ContinuousDependentAxisKeepsOneModuloFamily)
{
    const Configuration q{0.4, -0.5, 0.7, 0.8, 0.0, 1.1 + 8.0 * kPeriod};
    restrictAround(q);
    joint(5).type = ik::JointType::Continuous;
    parameters.limits[5].enabled = false;
    checkBoth(q, true, true);
    const auto target = pose(q);
    const auto general = ik::AnalyticIkSolver{}.solve(chain, target, tcp);
    ASSERT_TRUE(general);
    EXPECT_EQ(general->branches.size(), 1U);
    EXPECT_EQ(gen::solve(parameters, {{target.position.x, target.position.y, target.position.z},
                                     target.orientation.values}).branches.size(), 1U);
}

TEST_P(AnalyticIkRegression, RandomRegularTargetsRecoverActualAnglesAndEveryBranchMatchesFk)
{
    std::mt19937_64 random{6404};
    for (int sample = 0; sample < 1000; ++sample)
    {
        SCOPED_TRACE(sample);
        Configuration q{};
        for (std::size_t i = 0; i < q.size(); ++i)
            q[i] = std::uniform_real_distribution<double>(parameters.limits[i].lower + 0.01,
                                                          parameters.limits[i].upper - 0.01)(random);
        if (std::abs(q[4]) < 1e-4) q[4] += 0.01;
        checkBoth(q);
        if (HasFailure()) return;
    }
}


// ---- F3.R2: branch budget, orientation projection, limit snapping ---------

namespace {
gen::Pose generatedPose(const ik::PoseTarget& target)
{
    return {{target.position.x, target.position.y, target.position.z}, target.orientation.values};
}
} // namespace

// ---- Audit 2026-11-16 ------------------------------------------------------

TEST_P(AnalyticIkRegression, ShoulderSingularTargetIsUnsupportedNotNoSolution)
{
    // Audit #3: with the wrist centre on the joint-1 axis q1 is free. The
    // general solver reports that as unsupported; the generated solver used to
    // say NoSolution for a target produced by a valid in-limit configuration.
    //
    // Construct such a configuration: q1 = 0, pick q2, then bisect q3 until the
    // wrist centre's offset from axis 1 (along x, as axis 1 is vertical for
    // both robots) vanishes.
    const auto wristOffset = [&](const Configuration& q) {
        const auto target = pose(q);
        const auto& r = target.orientation.values;
        const auto& w = parameters.wristInTargetFrame;
        const double wx = target.position.x + r[0][0] * w.x + r[0][1] * w.y + r[0][2] * w.z;
        return wx - parameters.axes[0].point.x;
    };
    const auto inLimits = [&](const Configuration& q) {
        for (std::size_t i = 0; i < q.size(); ++i)
            if (parameters.limits[i].enabled &&
                (q[i] < parameters.limits[i].lower || q[i] > parameters.limits[i].upper))
                return false;
        return true;
    };

    std::optional<Configuration> singular;
    for (double q2 = -1.6; q2 <= 0.4 && !singular; q2 += 0.1)
    {
        Configuration lo{0.0, q2, -2.0, 0.3, 0.6, 0.2};
        for (double q3 = -2.0; q3 < 2.6 && !singular; q3 += 0.05)
        {
            Configuration hi = lo;
            hi[2] = q3 + 0.05;
            Configuration a = lo;
            a[2] = q3;
            if ((wristOffset(a) < 0.0) == (wristOffset(hi) < 0.0))
                continue;
            for (int iteration = 0; iteration < 200; ++iteration)
            {
                Configuration mid = a;
                mid[2] = std::midpoint(a[2], hi[2]);
                ((wristOffset(mid) < 0.0) == (wristOffset(a) < 0.0) ? a : hi) = mid;
            }
            if (inLimits(a) && std::abs(wristOffset(a)) < 1e-12)
                singular = a;
        }
    }
    ASSERT_TRUE(singular.has_value()) << "no in-limit shoulder-singular configuration found";
    const auto target = pose(*singular);

    const auto solved = ik::AnalyticIkSolver{}.solve(chain, target, tcp);
    ASSERT_FALSE(solved);
    EXPECT_EQ(solved.error().code, ik::AnalyticIkSolverErrorCode::UnsupportedKinematicStructure);

    const auto compiled = gen::solve(parameters, generatedPose(target));
    EXPECT_EQ(compiled.status, gen::Status::UnsupportedSingularity);
    EXPECT_TRUE(compiled.branches.empty());
}

TEST_P(AnalyticIkRegression, SingularFamilyStaysExactlyWithinLimitsWhenIntervalsBarelyTouch)
{
    // Audit #7: when the free joint's own interval and the interval implied by
    // the dependent joint's limits miss each other by less than the solver
    // tolerance, the solver took their midpoint -- a value outside one of the
    // limits by ~2.5e-13. Returned values must satisfy the limits exactly.
    const Configuration q{0.4, -0.5, 0.7, 0.8, 0.0, 1.1};
    restrictAround(q);
    const auto target = pose(q);
    const auto reference = ik::AnalyticIkSolver{}.solve(chain, target, tcp);
    ASSERT_TRUE(reference) << reference.error().message;
    const auto family = std::find_if(reference->branches.begin(), reference->branches.end(),
                                     [](const auto& b) { return !b.coupledFreeParameters.empty(); });
    ASSERT_NE(family, reference->branches.end());
    const double offset = family->coupledFreeParameters.front().offset;
    const double multiplier = family->coupledFreeParameters.front().multiplier;

    const auto exactlyWithin = [](double value, double lower, double upper) {
        return value >= lower && value <= upper;
    };

    for (int step = 0; step < 40; ++step)
        for (double gap : {-3e-13, -1e-13, 0.0, 1e-13, 2.5e-13, 5e-13, 9e-13})
        {
            // Free joint: [freeUpper - 0.04, freeUpper]. Dependent limits chosen
            // so that the free interval they imply is [freeUpper + gap, ...].
            const double freeUpper = q[3] + 1e-3 * step - 0.02;
            const double freeLower = freeUpper - 0.04;
            const double d1 = offset + multiplier * (freeUpper + gap);
            const double d2 = offset + multiplier * (freeUpper + 0.04);
            limit(3, freeLower, freeUpper);
            limit(5, std::min(d1, d2), std::max(d1, d2));
            SCOPED_TRACE(testing::Message() << "step=" << step << " gap=" << gap);

            const auto solved = ik::AnalyticIkSolver{}.solve(chain, target, tcp);
            ASSERT_TRUE(solved) << solved.error().message;
            const auto compiled = gen::solve(parameters, generatedPose(target));
            const std::array results{numericBranches(*solved), compiled.branches};
            for (std::size_t solver = 0; solver < results.size(); ++solver)
            {
                SCOPED_TRACE(solver == 0 ? "general solver" : "generated solver");
                bool sawFamily = false;
                for (const auto& branch : results[solver])
                {
                    if (!branch.singular)
                        continue;
                    sawFamily = true;
                    const auto& fl = parameters.limits[3];
                    const auto& dl = parameters.limits[5];
                    EXPECT_LE(branch.freeLower, branch.freeUpper);
                    for (double free : {branch.freeLower, branch.freeUpper})
                    {
                        const double dependent =
                            branch.dependentOffset + branch.dependentMultiplier * free;
                        EXPECT_TRUE(exactlyWithin(free, fl.lower, fl.upper))
                            << std::setprecision(17) << free << " not in [" << fl.lower << ", "
                            << fl.upper << "]";
                        EXPECT_TRUE(exactlyWithin(dependent, dl.lower, dl.upper))
                            << std::setprecision(17) << dependent << " not in [" << dl.lower
                            << ", " << dl.upper << "]";
                    }
                }
                // Within the solver tolerance the touching family is kept.
                EXPECT_TRUE(sawFamily);
            }
            checkBranches(numericBranches(*solved), target);
            checkBranches(compiled.branches, target);
            if (HasFailure()) return;
        }
}

TEST_P(AnalyticIkRegression, NearlyStraightElbowAtItsLimitIsStillSolved)
{
    // Audit #4: near the straight-elbow singularity the elbow angle is
    // recovered with ~sqrt(eps) error. With the generating q3 exactly on a
    // limit, the recovered value landed ~8e-10 rad outside it and both
    // solvers rejected a reachable target. Position is insensitive to the
    // elbow angle there (second order), so the value can be put back on the
    // limit and still pass the FK check.
    const double straight = -parameters.link2HomeAngle / parameters.axis3Sign;
    for (double delta : {0.0, 1e-10, -1e-10, 1e-9, -1e-9, 1e-8, -1e-8, 1e-7, -1e-7, 1e-6, -1e-6})
        for (bool atUpper : {true, false})
        {
            SCOPED_TRACE(testing::Message() << "delta=" << delta << " atUpper=" << atUpper);
            const Configuration q{0.4, -0.5, straight + delta, 0.8, -0.9, 1.1};
            restrictAround(q);
            if (atUpper)
                limit(2, q[2] - 0.05, q[2]);
            else
                limit(2, q[2], q[2] + 0.05);
            checkBoth(q);
            if (HasFailure()) return;

            const auto target = pose(q);
            const auto solved = ik::AnalyticIkSolver{}.solve(chain, target, tcp);
            ASSERT_TRUE(solved);
            const auto compiled = gen::solve(parameters, generatedPose(target));
            for (const auto& branches : {numericBranches(*solved), compiled.branches})
                for (const auto& branch : branches)
                    for (std::size_t i = 0; i < 6; ++i)
                        if (!(branch.singular && (i == 3 || i == 5)))
                        {
                            EXPECT_GE(branch.q[i], parameters.limits[i].lower) << i;
                            EXPECT_LE(branch.q[i], parameters.limits[i].upper) << i;
                        }
        }
}

TEST_P(AnalyticIkRegression, WideLimitsReportResourceLimitInsteadOfExpanding)
{
    const Configuration q{0.4, -0.5, 0.7, 0.8, -0.9, 1.1};
    const auto target = pose(q);

    const auto expectExceeded = [&] {
        const auto solved = ik::AnalyticIkSolver{}.solve(chain, target, tcp);
        ASSERT_FALSE(solved);
        EXPECT_EQ(solved.error().code, ik::AnalyticIkSolverErrorCode::ResourceLimitExceeded);
        ASSERT_TRUE(solved.error().requiredBranches.has_value());
        EXPECT_GT(*solved.error().requiredBranches, ik::AnalyticIkSolverOptions{}.maxBranches);

        const auto compiled = gen::solve(parameters, generatedPose(target));
        EXPECT_EQ(compiled.status, gen::Status::ResourceLimitExceeded);
        EXPECT_TRUE(compiled.branches.empty());
        EXPECT_EQ(compiled.requiredBranches, *solved.error().requiredBranches);
    };

    // Measured before the fix: neither case returned within 120 s.
    limit(3, -1000.0, 1000.0);
    limit(5, -1000.0, 1000.0);
    expectExceeded();

    limit(3, q[3] - 0.05, q[3] + 0.05);
    limit(5, -1e5, 1e5);
    expectExceeded();

    // The count is taken from the limits alone, so even a range no vector
    // could hold is rejected immediately.
    limit(5, -1e15, 1e15);
    expectExceeded();
}

TEST_P(AnalyticIkRegression, LimitsBeyondInt64TurnsReportResourceLimitNotNoSolution)
{
    // Audit 2026-11-16 #8: a turn range that does not fit in int64_t was
    // treated as empty, so [-1e20, 1e20] produced NoSolution for a reachable
    // target. It must be reported as a resource/representation limit.
    const Configuration q{0.4, -0.5, 0.7, 0.8, -0.9, 1.1};
    const auto target = pose(q);
    limit(3, q[3] - 0.05, q[3] + 0.05);
    limit(5, -1e20, 1e20);

    for (const std::size_t budget : {std::size_t{4096}, std::numeric_limits<std::size_t>::max()})
    {
        SCOPED_TRACE(budget);
        const auto solved = ik::AnalyticIkSolver{}.solve(chain, target, tcp, {budget});
        ASSERT_FALSE(solved);
        EXPECT_EQ(solved.error().code, ik::AnalyticIkSolverErrorCode::ResourceLimitExceeded)
            << solved.error().message;

        const auto compiled = gen::solve(parameters, generatedPose(target), budget);
        EXPECT_EQ(compiled.status, gen::Status::ResourceLimitExceeded);
        EXPECT_TRUE(compiled.branches.empty());
    }
}

TEST(AnalyticIkNumerics, TurnRangeBeyondInt64IsSaturatedNotEmpty)
{
    namespace d = kinemaforge::ik::detail;
    EXPECT_EQ(d::turnCount(d::turnsWithin(-1e20, 1e20)),
              std::numeric_limits<std::size_t>::max());
    EXPECT_EQ(d::turnCount(d::turnsWithin(1e20, 1e20 + 1e6)),
              std::numeric_limits<std::size_t>::max());
    // Genuinely empty and ordinary ranges keep their exact counts.
    EXPECT_EQ(d::turnCount(d::turnsWithin(0.1, 0.2)), 0U);
    EXPECT_EQ(d::turnCount(d::turnsWithin(-0.1, 0.1)), 1U);
    EXPECT_EQ(d::turnCount(d::turnsWithin(-1e20, -1e20 + 1.0, 0.0)) ,
              std::numeric_limits<std::size_t>::max());
}

TEST(AnalyticIkNumerics, NearbyLimitSnapsOnlyJustOutsideEveryTurn)
{
    namespace d = kinemaforge::ik::detail;
    constexpr double m = 1e-7;
    EXPECT_FALSE(d::nearbyLimit(0.5, 0.0, 1.0, m));               // inside
    EXPECT_FALSE(d::nearbyLimit(1.0, 0.0, 1.0, m));               // on the limit
    EXPECT_FALSE(d::nearbyLimit(0.5 + kPeriod, 0.0, 1.0, m));     // another turn inside
    EXPECT_EQ(d::nearbyLimit(1.0 + 5e-8, 0.0, 1.0, m), 1.0);
    EXPECT_EQ(d::nearbyLimit(-5e-8, 0.0, 1.0, m), 0.0);
    EXPECT_EQ(d::nearbyLimit(1.0 + 5e-8 - kPeriod, 0.0, 1.0, m), 1.0);  // modulo 2*pi
    EXPECT_FALSE(d::nearbyLimit(1.0 + 2e-7, 0.0, 1.0, m));        // beyond the margin
    EXPECT_FALSE(d::nearbyLimit(std::nan(""), 0.0, 1.0, m));
}

TEST(AnalyticIkNumerics, CoupledFreeIntervalIsExactlyWithinBothLimits)
{
    namespace d = kinemaforge::ik::detail;
    std::mt19937_64 random{7};
    std::uniform_real_distribution<double> uniform(-3.0, 3.0);
    std::uniform_real_distribution<double> gapDistribution(-1e-12, 9e-13);
    for (int sample = 0; sample < 20000; ++sample)
    {
        const double multiplier = sample % 2 == 0 ? 1.0 : -1.0;
        const double offset = uniform(random) + kPeriod * static_cast<double>(sample % 7 - 3);
        const double freeLower = uniform(random);
        const double freeUpper = freeLower + 0.05;
        const double gap = gapDistribution(random);
        const double a = offset + multiplier * (freeUpper + gap);
        const double b = offset + multiplier * (freeUpper + 0.05);
        const double dependentLower = std::min(a, b);
        const double dependentUpper = std::max(a, b);
        const auto interval = d::coupledFreeInterval(freeLower, freeUpper, true, dependentLower,
                                                     dependentUpper, offset, multiplier, 1e-12);
        ASSERT_TRUE(interval) << sample;
        EXPECT_LE(interval->lower, interval->upper);
        EXPECT_LE(std::abs(interval->offset - offset), 3e-12);
        for (double free : {interval->lower, interval->upper})
        {
            const double dependent = interval->offset + multiplier * free;
            ASSERT_GE(free, freeLower) << sample;
            ASSERT_LE(free, freeUpper) << sample;
            ASSERT_GE(dependent, dependentLower) << sample;
            ASSERT_LE(dependent, dependentUpper) << sample;
        }
    }
    // Beyond the tolerance the family is dropped, not stretched.
    EXPECT_FALSE(d::coupledFreeInterval(0.0, 1.0, true, 1.0 + 1e-9, 2.0, 0.0, 1.0, 1e-12));
}

TEST_P(AnalyticIkRegression, BranchBudgetIsExactAndNeverTruncates)
{
    const Configuration q{0.4, -0.5, 0.7, 0.8, -0.9, 1.1};
    limit(3, -100.0, 100.0);
    limit(5, -100.0, 100.0);
    const auto target = pose(q);

    const auto refused = ik::AnalyticIkSolver{}.solve(chain, target, tcp, {0});
    ASSERT_FALSE(refused);
    ASSERT_EQ(refused.error().code, ik::AnalyticIkSolverErrorCode::ResourceLimitExceeded);
    const std::size_t required = *refused.error().requiredBranches;
    ASSERT_GT(required, 1000U);

    const auto oneShort = ik::AnalyticIkSolver{}.solve(chain, target, tcp, {required - 1});
    ASSERT_FALSE(oneShort);
    EXPECT_EQ(oneShort.error().code, ik::AnalyticIkSolverErrorCode::ResourceLimitExceeded);
    EXPECT_EQ(*oneShort.error().requiredBranches, required);

    const auto solved = ik::AnalyticIkSolver{}.solve(chain, target, tcp, {required});
    ASSERT_TRUE(solved) << solved.error().message;
    const auto general = numericBranches(*solved);
    EXPECT_LE(general.size(), required);
    EXPECT_GT(general.size(), required / 2);
    checkBranches(general, target);
    EXPECT_TRUE(contains(general, q));

    const auto compiledShort = gen::solve(parameters, generatedPose(target), required - 1);
    EXPECT_EQ(compiledShort.status, gen::Status::ResourceLimitExceeded);
    EXPECT_TRUE(compiledShort.branches.empty());

    const auto compiled = gen::solve(parameters, generatedPose(target), required);
    EXPECT_EQ(compiled.status, gen::Status::Solutions);
    EXPECT_EQ(compiled.requiredBranches, required);
    EXPECT_EQ(compiled.branches.size(), general.size());
    checkBranches(compiled.branches, target);
    EXPECT_TRUE(contains(compiled.branches, q));
}

TEST_P(AnalyticIkRegression, TargetsRoundedToEightDecimalsAreSolvedAgainstTheirNearestRotation)
{
    // Measured before the fix (KR640, 1000 targets): 8 solved, 755 NoSolution,
    // 237 rejected by validation. A matrix inside the validation band is not an
    // exact rotation, and no joint angles reproduce a non-rotation to 2e-9.
    // The target moved by up to 5e-9 per element, and near an elbow or wrist
    // singularity that is amplified in joint space, so the generating
    // configuration is only required to be recovered loosely here. Exactness is
    // carried by the FK check against the projected target.
    const auto recovered = [&](const std::vector<gen::Branch>& branches, const Configuration& q) {
        return std::any_of(branches.begin(), branches.end(), [&](const gen::Branch& branch) {
            for (std::size_t i = 0; i < q.size(); ++i)
                if (std::abs(branch.q[i] - q[i]) > 1e-4)
                    return false;
            return true;
        });
    };
    std::mt19937_64 random{20261117};
    int accepted = 0;
    for (int sample = 0; sample < 1000; ++sample)
    {
        SCOPED_TRACE(sample);
        Configuration q{};
        for (std::size_t i = 0; i < q.size(); ++i)
            q[i] = std::uniform_real_distribution<double>(parameters.limits[i].lower + 0.2,
                                                          parameters.limits[i].upper - 0.2)(random);
        if (std::abs(q[4]) < 0.05) q[4] = 0.3;

        auto target = pose(q);
        const auto round = [](double& value) { value = std::round(value * 1e8) / 1e8; };
        round(target.position.x);
        round(target.position.y);
        round(target.position.z);
        for (auto& row : target.orientation.values)
            for (double& value : row)
                round(value);

        if (!ik::validate(target))
            continue; // rounding pushed it outside the 1e-8 band; rejection is correct
        ++accepted;

        const auto projection = ik::nearestRotation(target.orientation);
        ASSERT_TRUE(projection);
        EXPECT_LE(projection->correction, ik::kMaxOrientationCorrection);
        const ik::PoseTarget projected{target.position, projection->rotation};

        const auto solved = ik::AnalyticIkSolver{}.solve(chain, target, tcp);
        ASSERT_TRUE(solved) << solved.error().message;
        EXPECT_EQ(solved->orientationCorrection, projection->correction);
        const auto general = numericBranches(*solved);
        checkBranches(general, projected);
        EXPECT_TRUE(recovered(general, q));

        const auto compiled = gen::solve(parameters, generatedPose(target));
        EXPECT_EQ(compiled.status, gen::Status::Solutions);
        EXPECT_EQ(compiled.orientationCorrection, projection->correction);
        checkBranches(compiled.branches, projected);
        EXPECT_TRUE(recovered(compiled.branches, q));
        if (HasFailure()) return;
    }
    EXPECT_GT(accepted, 500);
}

TEST_P(AnalyticIkRegression, CleanTargetsReportNegligibleOrientationCorrection)
{
    const auto target = pose({0.4, -0.5, 0.7, 0.8, -0.9, 1.1});
    const auto solved = ik::AnalyticIkSolver{}.solve(chain, target, tcp);
    ASSERT_TRUE(solved);
    EXPECT_LT(solved->orientationCorrection, 1e-14);
    EXPECT_LT(gen::solve(parameters, generatedPose(target)).orientationCorrection, 1e-14);
}

TEST_P(AnalyticIkRegression, ReturnedJointValuesNeverLeaveTheirLimits)
{
    // Measured before the fix: values up to 1.6e-13 beyond a limit.
    const auto strictlyInside = [&](const std::vector<gen::Branch>& branches) {
        for (const auto& branch : branches)
            for (std::size_t i = 0; i < branch.q.size(); ++i)
            {
                if (branch.singular && (i == 3 || i == 5))
                    continue;
                EXPECT_GE(branch.q[i], parameters.limits[i].lower);
                EXPECT_LE(branch.q[i], parameters.limits[i].upper);
            }
        for (const auto& branch : branches)
            if (branch.singular)
            {
                EXPECT_GE(branch.freeLower, parameters.limits[3].lower);
                EXPECT_LE(branch.freeUpper, parameters.limits[3].upper);
            }
    };
    const auto check = [&](const Configuration& q) {
        const auto target = pose(q);
        const auto solved = ik::AnalyticIkSolver{}.solve(chain, target, tcp);
        ASSERT_TRUE(solved) << solved.error().message;
        const auto general = numericBranches(*solved);
        const auto compiled = gen::solve(parameters, generatedPose(target)).branches;
        strictlyInside(general);
        strictlyInside(compiled);
        EXPECT_TRUE(contains(general, q));
        EXPECT_TRUE(contains(compiled, q));
    };

    std::mt19937_64 random{20261118};
    for (int sample = 0; sample < 600; ++sample)
    {
        SCOPED_TRACE(sample);
        Configuration q{};
        for (std::size_t i = 0; i < q.size(); ++i)
            q[i] = std::uniform_real_distribution<double>(parameters.limits[i].lower + 0.01,
                                                          parameters.limits[i].upper - 0.01)(random);
        if (std::abs(q[4]) < 0.05) q[4] = 0.3;
        const std::size_t joint = static_cast<std::size_t>(sample) % 6;
        q[joint] = (sample / 6) % 2 == 0 ? parameters.limits[joint].lower : parameters.limits[joint].upper;
        check(q);
        if (HasFailure()) return;
    }
    for (unsigned mask = 0; mask < 64; ++mask)
    {
        SCOPED_TRACE(mask);
        Configuration q{};
        for (std::size_t i = 0; i < q.size(); ++i)
            q[i] = (mask >> i) & 1U ? parameters.limits[i].upper : parameters.limits[i].lower;
        check(q);
        if (HasFailure()) return;
    }
}

INSTANTIATE_TEST_SUITE_P(Robots, AnalyticIkRegression, testing::Values(0, 1, 2),
    [](const testing::TestParamInfo<int>& info) {
        return info.param == 0 ? "Kr640" : info.param == 1 ? "Kr4" : "Kr640Tcp";
    });
} // namespace

// ---- Audit 2026-11-16 #2: equal arm links ------------------------------------

namespace {

struct EqualLinksRobot
{
    ik::KinematicChain chain;
    gen::Parameters parameters = equal_links_fresh::parameters;

    EqualLinksRobot()
    {
        const auto robot = ik::UrdfModelLoader{}.load(
            std::filesystem::path(KINEMAFORGE_TEST_DATA_DIR) / "equal_links_6r.urdf");
        chain = ik::KinematicChainBuilder{}.build(robot, "base_link", "tool0").value();
    }

    void limit(std::size_t position, double lower, double upper)
    {
        for (auto& entry : chain.joints)
            if (entry.type != ik::JointType::Fixed && position-- == 0)
            {
                entry.limits.hasPositionLimits = true;
                entry.limits.lower = lower;
                entry.limits.upper = upper;
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
        return {{fk.translation.x, fk.translation.y, fk.translation.z},
                {ts::toRotationMatrix(fk.rotation)}};
    }
};

} // namespace

TEST(AnalyticIkEqualLinks, LinksAreEqualInTheFixture)
{
    const EqualLinksRobot robot;
    EXPECT_DOUBLE_EQ(robot.parameters.link1, robot.parameters.link2);
}

TEST(AnalyticIkEqualLinks, RegularTargetStillRoundTrips)
{
    // Control: the equal-link geometry itself is supported.
    const EqualLinksRobot robot;
    const Configuration q{0.4, -0.5, 0.7, 0.8, -0.9, 1.1};
    const auto target = robot.pose(q);
    const auto solved = ik::AnalyticIkSolver{}.solve(robot.chain, target);
    ASSERT_TRUE(solved) << solved.error().message;
    EXPECT_EQ(solved->status, ik::AnalyticIkStatus::Solutions);
    const auto compiled = gen::solve(robot.parameters, generatedPose(target));
    EXPECT_EQ(compiled.status, gen::Status::Solutions);
}

TEST(AnalyticIkEqualLinks, FoldedElbowOnShoulderAxisIsUnsupportedNotOneArbitraryAngle)
{
    // With L1 == L2 and the elbow fully folded the wrist centre lies on the
    // joint-2 axis and q2 is free. The planar solver used to pick one q2
    // (atan2(0, 0)); once limits excluded that pick, both solvers returned
    // NoSolution for a target produced by an in-limit configuration.
    EqualLinksRobot robot;
    const double folded = std::remainder(
        (std::numbers::pi_v<double> - robot.parameters.link2HomeAngle) / robot.parameters.axis3Sign,
        kPeriod);
    const Configuration q{0.4, -0.5, folded, 0.8, -0.9, 1.1};
    for (std::size_t i = 0; i < q.size(); ++i)
        robot.limitBoth(i, q[i] - 0.05, q[i] + 0.05);
    const auto target = robot.pose(q);

    const auto solved = ik::AnalyticIkSolver{}.solve(robot.chain, target);
    ASSERT_FALSE(solved) << "status " << static_cast<int>(solved->status)
                         << ", branches " << solved->branches.size();
    EXPECT_EQ(solved.error().code, ik::AnalyticIkSolverErrorCode::UnsupportedKinematicStructure);

    const auto compiled = gen::solve(robot.parameters, generatedPose(target));
    EXPECT_EQ(compiled.status, gen::Status::UnsupportedSingularity);
    EXPECT_TRUE(compiled.branches.empty());
}
