#include <gtest/gtest.h>

#include "generated/kr4_analytic_ik.hpp"
#include "generated/kr640_analytic_ik.hpp"
#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/codegen/CppAnalyticIkGenerator.hpp"
#include "support/NumericForwardKinematics.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

namespace ik = kinemaforge::ik;
namespace gen = kinemaforge::ik::generated;
namespace ts = kinemaforge::testsupport;
namespace {

ik::KinematicChain loadChain(const char* fileName)
{
    ik::UrdfModelLoader loader;
    const auto robot = loader.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / fileName);
    const auto chain = ik::KinematicChainBuilder{}.build(robot, "base_link", "tool0");
    return chain ? *chain : ik::KinematicChain{};
}

gen::Pose generatedTarget(const ik::KinematicChain& chain, const std::vector<double>& q)
{
    const auto fk = ts::numericForwardKinematics(chain, q);
    return {{fk.translation.x, fk.translation.y, fk.translation.z},
            ts::toRotationMatrix(fk.rotation)};
}

template <typename Result>
bool contains(const Result& result, const std::vector<double>& q)
{
    return std::any_of(result.branches.begin(), result.branches.end(), [&](const auto& branch) {
        if (branch.singular)
            return false;
        for (std::size_t index = 0; index < q.size(); ++index)
            if (std::abs(std::remainder(branch.q[index] - q[index],
                                        2.0 * 3.14159265358979323846)) > 3e-7)
                return false;
        return true;
    });
}

} // namespace

TEST(CppAnalyticIkGenerator, IsDeterministic)
{
    const auto chain = loadChain("kr640.urdf");
    const ik::CppAnalyticIkGenerator generator;
    const auto first = generator.generate(chain, std::nullopt, "example_solver");
    const auto second = generator.generate(chain, std::nullopt, "example_solver");
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_EQ(*first, *second);
}

TEST(CppAnalyticIkGenerator, EmitsOnlyLightweightGeneratedRuntimeDependency)
{
    const auto chain = loadChain("kr640.urdf");
    const auto generated = ik::CppAnalyticIkGenerator{}.generate(chain);
    ASSERT_TRUE(generated);
    EXPECT_NE(generated->find("GeneratedIkRuntime.hpp"), std::string::npos);
    EXPECT_EQ(generated->find("UrdfModelLoader"), std::string::npos);
    EXPECT_EQ(generated->find("Expression"), std::string::npos);
    EXPECT_EQ(generated->find("AnalyticIkSolver"), std::string::npos);
}

TEST(CppAnalyticIkGenerator, RejectsInvalidNamespaceName)
{
    const auto result = ik::CppAnalyticIkGenerator{}.generate(loadChain("kr640.urdf"),
                                                               std::nullopt, "bad-name");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, ik::CodeGenerationErrorCode::InvalidNamespaceName);
}

TEST(CppAnalyticIkGenerator, CheckedInKr640GeneratedSolverCompilesAndRecoversPose)
{
    const auto chain = loadChain("kr640.urdf");
    const std::vector<double> q{0.4, -0.5, 0.7, 0.8, -0.9, 1.1};
    const auto result = kr640_generated::solve(generatedTarget(chain, q));
    EXPECT_EQ(result.status, gen::Status::Solutions);
    EXPECT_TRUE(contains(result, q));
}

TEST(CppAnalyticIkGenerator, CheckedInKr4GeneratedSolverCompilesAndRecoversPose)
{
    const auto chain = loadChain("kr4_r600.urdf");
    const std::vector<double> q{0.5, -0.7, 0.8, 0.6, -0.8, 1.2};
    const auto result = kr4_generated::solve(generatedTarget(chain, q));
    EXPECT_EQ(result.status, gen::Status::Solutions);
    EXPECT_TRUE(contains(result, q));
}

TEST(CppAnalyticIkGenerator, GeneratedKr640ReportsWristSingularity)
{
    const auto chain = loadChain("kr640.urdf");
    const std::vector<double> q{0.4, -0.5, 0.7, 0.8, 0.0, 1.1};
    const auto result = kr640_generated::solve(generatedTarget(chain, q));
    EXPECT_EQ(result.status, gen::Status::InfiniteFamily);
    EXPECT_TRUE(std::any_of(result.branches.begin(), result.branches.end(),
                            [](const auto& branch) { return branch.singular; }));
}

TEST(CppAnalyticIkGenerator, TcpChangesGeneratedParameters)
{
    const auto chain = loadChain("kr640.urdf");
    const ik::FixedRigidTransform tcp{{0.03, -0.04, 0.12}, {0.2, -0.1, 0.3}};
    const auto withoutTcp = ik::CppAnalyticIkGenerator{}.generate(chain);
    const auto withTcp = ik::CppAnalyticIkGenerator{}.generate(chain, tcp);
    ASSERT_TRUE(withoutTcp);
    ASSERT_TRUE(withTcp);
    EXPECT_NE(*withoutTcp, *withTcp);
}
