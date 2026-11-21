#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/solver/PreparedAnalyticIk.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <numeric>
#include <random>
#include <vector>

// The geometric Jacobian and its singular values (MotionBridge report
// 2026-11-21, 5.6). PreparedAnalyticIk::jacobian(q) gives, for the tool frame
// the solver solves for (the flange, or the TCP if one was given):
//
//   jacobian[row][joint]; rows 0..2 = linear velocity of the tool origin in
//   the base frame, rows 3..5 = angular velocity, per unit joint velocity.
//   Linear rows are metres per radian, angular rows are dimensionless, so the
//   singular values mix units exactly as a "smallest singular value of the
//   Jacobian" computed elsewhere does.
//
// Checked here against independent references: central differences of the
// quaternion forward kinematics, and the determinant.

namespace ik = kinemaforge::ik;
namespace ts = kinemaforge::testsupport;

namespace {

using Matrix6 = std::array<std::array<double, 6>, 6>;
using Joints = std::array<double, 6>;

ik::KinematicChain kr640Chain()
{
    return ik::KinematicChainBuilder{}
        .build(ik::UrdfModelLoader{}.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / "kr640.urdf"),
               "base_link", "tool0")
        .value();
}

ts::RigidTransform forward(const ik::KinematicChain& chain, const Joints& q,
                           const std::optional<ik::FixedRigidTransform>& tcp)
{
    const std::vector<double> values(q.begin(), q.end());
    return tcp ? ts::numericTcpForwardKinematics(chain, values, *tcp) : ts::numericForwardKinematics(chain, values);
}

// Central differences: linear rows from the position, angular rows from
// vee(dR/dq * R^T).
Matrix6 numericJacobian(const ik::KinematicChain& chain, const Joints& q,
                        const std::optional<ik::FixedRigidTransform>& tcp)
{
    constexpr double h = 1e-6;
    Matrix6 jacobian{};
    const auto base = ts::toRotationMatrix(forward(chain, q, tcp).rotation);
    for (std::size_t joint = 0; joint < 6; ++joint)
    {
        Joints plus = q;
        Joints minus = q;
        plus[joint] += h;
        minus[joint] -= h;
        const auto fp = forward(chain, plus, tcp);
        const auto fm = forward(chain, minus, tcp);
        jacobian[0][joint] = (fp.translation.x - fm.translation.x) / (2 * h);
        jacobian[1][joint] = (fp.translation.y - fm.translation.y) / (2 * h);
        jacobian[2][joint] = (fp.translation.z - fm.translation.z) / (2 * h);
        const auto rp = ts::toRotationMatrix(fp.rotation);
        const auto rm = ts::toRotationMatrix(fm.rotation);
        double derivative[3][3];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                derivative[r][c] = (rp[r][c] - rm[r][c]) / (2 * h);
        // W = dR * R^T is skew-symmetric: [0 -wz wy; wz 0 -wx; -wy wx 0].
        double w[3][3] = {};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                for (int k = 0; k < 3; ++k)
                    w[r][c] += derivative[r][k] * base[c][k];
        jacobian[3][joint] = w[2][1];
        jacobian[4][joint] = w[0][2];
        jacobian[5][joint] = w[1][0];
    }
    return jacobian;
}

double determinant(Matrix6 m)
{
    double det = 1.0;
    for (std::size_t column = 0; column < 6; ++column)
    {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < 6; ++row)
            if (std::abs(m[row][column]) > std::abs(m[pivot][column]))
                pivot = row;
        if (m[pivot][column] == 0.0)
            return 0.0;
        if (pivot != column)
        {
            std::swap(m[pivot], m[column]);
            det = -det;
        }
        det *= m[column][column];
        for (std::size_t row = column + 1; row < 6; ++row)
        {
            const double factor = m[row][column] / m[column][column];
            for (std::size_t k = column; k < 6; ++k)
                m[row][k] -= factor * m[column][k];
        }
    }
    return det;
}

Joints randomJoints(const ik::KinematicChain& chain, std::mt19937_64& rng)
{
    Joints q{};
    std::size_t i = 0;
    for (const auto& joint : chain.joints)
        if (joint.type != ik::JointType::Fixed)
            q[i++] = joint.limits.lower + std::uniform_real_distribution<double>(0.05, 0.95)(rng) *
                                              (joint.limits.upper - joint.limits.lower);
    return q;
}

} // namespace

TEST(AnalyticIkJacobian, MatchesCentralDifferencesOfTheForwardKinematics)
{
    const auto chain = kr640Chain();
    const auto prepared = ik::PreparedAnalyticIk::create(chain);
    ASSERT_TRUE(prepared);
    std::mt19937_64 rng(0x7AC0);
    for (int n = 0; n < 40; ++n)
    {
        const Joints q = randomJoints(chain, rng);
        const auto analysis = prepared->jacobian(q);
        ASSERT_TRUE(analysis) << analysis.error().message;
        const Matrix6 reference = numericJacobian(chain, q, std::nullopt);
        for (std::size_t r = 0; r < 6; ++r)
            for (std::size_t c = 0; c < 6; ++c)
                EXPECT_NEAR(analysis->jacobian[r][c], reference[r][c], 2e-7) << "row " << r << " col " << c;
    }
}

TEST(AnalyticIkJacobian, IsTakenAtTheToolPointWhenOneIsGiven)
{
    const auto chain = kr640Chain();
    const ik::FixedRigidTransform tcp{{0.03, -0.04, 0.12}, {0.2, -0.1, 0.3}};
    const auto prepared = ik::PreparedAnalyticIk::create(chain, tcp);
    ASSERT_TRUE(prepared);
    std::mt19937_64 rng(0x7C9);
    const Joints q = randomJoints(chain, rng);
    const auto analysis = prepared->jacobian(q);
    ASSERT_TRUE(analysis);
    const Matrix6 reference = numericJacobian(chain, q, tcp);
    for (std::size_t r = 0; r < 6; ++r)
        for (std::size_t c = 0; c < 6; ++c)
            EXPECT_NEAR(analysis->jacobian[r][c], reference[r][c], 2e-7) << "row " << r << " col " << c;
}

TEST(AnalyticIkJacobian, SingularValuesAreOrderedAndMultiplyToTheDeterminant)
{
    const auto chain = kr640Chain();
    const auto prepared = ik::PreparedAnalyticIk::create(chain);
    ASSERT_TRUE(prepared);
    std::mt19937_64 rng(0x5EED);
    for (int n = 0; n < 40; ++n)
    {
        const auto analysis = prepared->jacobian(randomJoints(chain, rng));
        ASSERT_TRUE(analysis);
        const auto& sv = analysis->singularValues;
        EXPECT_TRUE(std::is_sorted(sv.begin(), sv.end(), std::greater<>{}));
        EXPECT_GT(sv.back(), 0.0);
        const double product = std::accumulate(sv.begin(), sv.end(), 1.0, std::multiplies<>{});
        EXPECT_NEAR(product, std::abs(determinant(analysis->jacobian)), 1e-9 * std::max(1.0, product));
        EXPECT_DOUBLE_EQ(analysis->minSingularValue, sv.back());
        EXPECT_NEAR(analysis->condition, sv.front() / sv.back(), 1e-9 * analysis->condition);
    }
}

TEST(AnalyticIkJacobian, SingularValueCollapsesAtTheWristSingularity)
{
    const auto chain = kr640Chain();
    const auto prepared = ik::PreparedAnalyticIk::create(chain);
    ASSERT_TRUE(prepared);
    const auto regular = prepared->jacobian({0.4, -0.5, 0.7, 0.8, 0.3, 1.1});
    const auto singular = prepared->jacobian({0.4, -0.5, 0.7, 0.8, 0.0, 1.1});
    ASSERT_TRUE(regular && singular);
    EXPECT_GT(regular->minSingularValue, 1e-3);
    EXPECT_LT(singular->minSingularValue, 1e-9);
    EXPECT_GT(singular->condition, 1e9);
}

TEST(AnalyticIkJacobian, RejectsNonFiniteJointValues)
{
    const auto chain = kr640Chain();
    const auto prepared = ik::PreparedAnalyticIk::create(chain);
    ASSERT_TRUE(prepared);
    const auto result = prepared->jacobian({0.0, 0.0, 0.0, 0.0, std::nan(""), 0.0});
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, ik::AnalyticIkSolverErrorCode::NonFiniteComputation);
}
