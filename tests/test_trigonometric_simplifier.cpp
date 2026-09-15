#include <gtest/gtest.h>

#include "ik_equations/builders/EquationSimplifier.hpp"
#include "ik_equations/model/Equation.hpp"
#include "ik_equations/model/IkEquationSystem.hpp"
#include "ik_equations/symbolic/ExpressionEvaluator.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/RewriteEngine.hpp"
#include "ik_equations/symbolic/TrigonometricSimplifier.hpp"

#include <cmath>
#include <limits>
#include <random>
#include <span>
#include <vector>

namespace ik = kinemaforge::ik;
using ik::Expression;
using ik::ExpressionFactory;

namespace {
const ExpressionFactory kFactory;
Expression sym(const char* n) { return kFactory.symbol(n); }
Expression con(double v) { return kFactory.constant(v); }
Expression trig(const Expression& e) { return ik::simplifyTrigonometry(e, kFactory); }

Expression pythagorean(const Expression& argument)
{
    const Expression s = kFactory.sin(argument);
    const Expression c = kFactory.cos(argument);
    return kFactory.add(kFactory.multiply(s, s), kFactory.multiply(c, c));
}
} // namespace

TEST(TrigonometricSimplifierTest, NormalizesSineOfNegativeArgument)
{
    const Expression x = sym("x");
    EXPECT_TRUE(ik::structurallyEqual(trig(kFactory.sin(kFactory.negate(x))),
                                      ik::canonicalize(kFactory.negate(kFactory.sin(x)), kFactory)));
}

TEST(TrigonometricSimplifierTest, NormalizesCosineOfNegativeArgument)
{
    const Expression x = sym("x");
    EXPECT_TRUE(ik::structurallyEqual(trig(kFactory.cos(kFactory.negate(x))), kFactory.cos(x)));
}

TEST(TrigonometricSimplifierTest, CosineEvenRuleRunsThroughRewriteEngine)
{
    const Expression x = sym("x");
    const ik::RewriteRule rule = ik::TrigonometricRules::cosineEven();
    ik::RewriteSession session{std::span<const ik::RewriteRule>(&rule, 1), kFactory};
    const auto result = session.rewrite(kFactory.cos(kFactory.negate(x)));
    EXPECT_TRUE(result.changed);
    EXPECT_TRUE(ik::structurallyEqual(result.expression, kFactory.cos(x)));
}

TEST(TrigonometricSimplifierTest, ReducesPythagoreanPair)
{
    EXPECT_TRUE(ik::isOne(trig(pythagorean(sym("x")))));
}

TEST(TrigonometricSimplifierTest, ReducesPairInsideLargerSum)
{
    const Expression input = kFactory.add(sym("z"), pythagorean(sym("x")));
    ik::ExpressionEvaluator evaluator{{{"x", 0.7}, {"z", 5.0}}};
    const auto value = evaluator.evaluate(trig(input));
    ASSERT_TRUE(value.has_value());
    EXPECT_NEAR(*value, 6.0, 1e-12);
}

TEST(TrigonometricSimplifierTest, MatchesCommonArgumentStructurally)
{
    const Expression a = kFactory.add(sym("x"), sym("y"));
    const Expression b = kFactory.add(sym("y"), sym("x"));
    const Expression input = kFactory.add(
        kFactory.multiply(kFactory.sin(a), kFactory.sin(a)),
        kFactory.multiply(kFactory.cos(b), kFactory.cos(b)));
    EXPECT_TRUE(ik::isOne(trig(input)));
}

TEST(TrigonometricSimplifierTest, DoesNotReduceDifferentArguments)
{
    const Expression sx = kFactory.sin(sym("x"));
    const Expression cy = kFactory.cos(sym("y"));
    const Expression input = kFactory.add(kFactory.multiply(sx, sx), kFactory.multiply(cy, cy));
    EXPECT_FALSE(ik::isOne(trig(input)));
}

TEST(TrigonometricSimplifierTest, DoesNotReducePartialArgument)
{
    const Expression partial = kFactory.divide(con(1.0), sym("q"));
    EXPECT_FALSE(ik::isOne(trig(pythagorean(partial))));
}

TEST(TrigonometricSimplifierTest, PreservesDivisionByZeroForPartialArgument)
{
    const Expression partial = kFactory.divide(con(1.0), sym("q"));
    const Expression simplified = trig(pythagorean(partial));
    ik::ExpressionEvaluator evaluator{{{"q", 0.0}}};
    const auto value = evaluator.evaluate(simplified);
    ASSERT_FALSE(value.has_value());
    EXPECT_EQ(value.error().code, ik::EvaluationErrorCode::DivisionByZero);
}

TEST(TrigonometricSimplifierTest, IsIdempotent)
{
    const Expression input = kFactory.add(sym("z"), pythagorean(kFactory.negate(sym("x"))));
    ik::TrigonometricSimplificationSession session{kFactory};
    const Expression once = session.simplify(input);
    const Expression twice = session.simplify(once);
    EXPECT_TRUE(ik::sameNode(once, twice));
}

TEST(TrigonometricSimplifierTest, KeepsNodeIdentityInFreshSession)
{
    const Expression normal = trig(kFactory.add(sym("z"), pythagorean(sym("x"))));
    ik::TrigonometricSimplificationSession fresh{kFactory};
    EXPECT_TRUE(ik::sameNode(normal, fresh.simplify(normal)));
}

TEST(TrigonometricSimplifierTest, EquationSimplifierUsesOneTrigSessionAcrossSystem)
{
    const Expression shared = pythagorean(sym("x"));
    std::vector<ik::Equation> equations{
        {kFactory.add(shared, sym("a")), con(1.0), ik::PositionEquationSource{ik::CartesianComponent::X}},
        {kFactory.add(shared, sym("b")), con(2.0), ik::PositionEquationSource{ik::CartesianComponent::Y}},
        {kFactory.add(shared, sym("c")), con(3.0), ik::PositionEquationSource{ik::CartesianComponent::Z}}};
    auto system = ik::IkEquationSystem::create(ik::IkTaskKind::Position,
                                                {ik::JointVariable{"q1", 1}},
                                                std::move(equations));
    ASSERT_TRUE(system.has_value());
    ik::EquationSimplifier simplifier{kFactory};
    auto result = simplifier.simplify(*system);
    ASSERT_TRUE(result.has_value());
    for (const auto& equation : result->equations())
    {
        ik::ExpressionEvaluator evaluator{{{"a", 0.0}, {"b", 0.0}, {"c", 0.0}, {"x", 1.0}}};
        EXPECT_TRUE(evaluator.evaluate(equation.lhs).has_value());
    }
}

TEST(TrigonometricSimplifierTest, PreservesNumericValueRandomized)
{
    const Expression x = sym("x");
    const Expression z = sym("z");
    const Expression input = kFactory.add(
        z,
        kFactory.add(
            kFactory.multiply(kFactory.sin(kFactory.negate(x)), kFactory.sin(kFactory.negate(x))),
            kFactory.multiply(kFactory.cos(kFactory.negate(x)), kFactory.cos(kFactory.negate(x)))));
    const Expression simplified = trig(input);

    std::mt19937_64 rng{0xF33u};
    std::uniform_real_distribution<double> distribution{-3.0, 3.0};
    for (int sample = 0; sample < 500; ++sample)
    {
        const ik::SymbolValues values{{"x", distribution(rng)}, {"z", distribution(rng)}};
        ik::ExpressionEvaluator before{values};
        ik::ExpressionEvaluator after{values};
        const auto a = before.evaluate(input);
        const auto b = after.evaluate(simplified);
        ASSERT_TRUE(a.has_value());
        ASSERT_TRUE(b.has_value());
        EXPECT_NEAR(*a, *b, 1e-12);
    }
}

TEST(TrigonometricSimplifierTest, KeepsRepresentabilityBarrierOpaque)
{
    const double m = std::numeric_limits<double>::max();
    const Expression overflow = kFactory.add(con(m), con(m));
    const Expression barrier = kFactory.subtract(overflow, con(1.0));
    EXPECT_TRUE(ik::sameNode(ik::canonicalize(barrier, kFactory), trig(barrier)));
}
