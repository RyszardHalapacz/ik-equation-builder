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
#include <string>
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

// ---- F3.R2: signed / scaled pairs and the fixed point -----------------------

namespace {
Expression square(const Expression& e) { return kFactory.multiply(e, e); }
Expression canon(const Expression& e) { return ik::canonicalize(e, kFactory); }
} // namespace

TEST(TrigonometricSimplifierTest, ReducesNegatedPair)
{
    const Expression a = sym("a");
    const Expression reduced = trig(kFactory.subtract(a, pythagorean(sym("b"))));
    EXPECT_TRUE(ik::structurallyEqual(reduced, canon(kFactory.add(a, con(-1.0)))));
}

TEST(TrigonometricSimplifierTest, ReducesPairWithCommonConstantCoefficient)
{
    const Expression b = sym("b");
    const Expression scaled = kFactory.add(kFactory.multiply(con(2.0), square(kFactory.sin(b))),
                                           kFactory.multiply(con(2.0), square(kFactory.cos(b))));
    EXPECT_TRUE(ik::structurallyEqual(trig(scaled), con(2.0)));
}

TEST(TrigonometricSimplifierTest, ReducesPairWithCommonSymbolicFactor)
{
    const Expression a = sym("a");
    const Expression b = sym("b");
    const Expression scaled = kFactory.add(kFactory.multiply(square(kFactory.cos(b)), a),
                                           kFactory.multiply(square(kFactory.sin(b)), a));
    EXPECT_TRUE(ik::structurallyEqual(trig(scaled), a));
}

TEST(TrigonometricSimplifierTest, ReducesNegatedPairWithCommonFactor)
{
    const Expression a = sym("a");
    const Expression b = sym("b");
    const Expression x = sym("x");
    // x - 3*a*sin^2(b) - 3*a*cos^2(b)  ->  x - 3*a
    const Expression three_a = kFactory.multiply(con(3.0), a);
    const Expression input = kFactory.subtract(
        kFactory.subtract(x, kFactory.multiply(three_a, square(kFactory.sin(b)))),
        kFactory.multiply(three_a, square(kFactory.cos(b))));
    EXPECT_TRUE(ik::structurallyEqual(trig(input), canon(kFactory.subtract(x, three_a))));
}

TEST(TrigonometricSimplifierTest, DoesNotCombineDifferentCoefficients)
{
    const Expression b = sym("b");
    const Expression mixed = kFactory.add(kFactory.multiply(con(2.0), square(kFactory.sin(b))),
                                          kFactory.multiply(con(3.0), square(kFactory.cos(b))));
    EXPECT_TRUE(ik::structurallyEqual(trig(mixed), canon(mixed)));
}

TEST(TrigonometricSimplifierTest, DoesNotCombineOppositeSigns)
{
    const Expression b = sym("b");
    const Expression difference = kFactory.subtract(square(kFactory.cos(b)), square(kFactory.sin(b)));
    EXPECT_TRUE(ik::structurallyEqual(trig(difference), canon(difference)));
}

TEST(TrigonometricSimplifierTest, ReducesWhenTheCommonFactorIsItselfATrigSquare)
{
    // sin^2(u)*cos^2(u) reads both as a sine square and as a cosine square.
    const Expression u = sym("u");
    const Expression s2 = square(kFactory.sin(u));
    const Expression c2 = square(kFactory.cos(u));
    const Expression input = kFactory.add(kFactory.multiply(s2, c2), kFactory.multiply(c2, c2));
    EXPECT_TRUE(ik::structurallyEqual(trig(input), canon(c2)));
}

TEST(TrigonometricSimplifierTest, KeepsScaledPairWhoseArgumentHasADivisor)
{
    const Expression argument = kFactory.divide(sym("a"), sym("q"));
    const Expression scaled = kFactory.add(kFactory.multiply(con(2.0), square(kFactory.sin(argument))),
                                           kFactory.multiply(con(2.0), square(kFactory.cos(argument))));
    const Expression reduced = trig(scaled);
    EXPECT_FALSE(ik::isConstant(reduced));
    EXPECT_FALSE(ik::divisorsOf(reduced).empty());
}

TEST(TrigonometricSimplifierTest, KeepsDivisorOfTheCommonFactor)
{
    // (1/q)*sin^2(b) + (1/q)*cos^2(b) -> 1/q, never 1.
    const Expression b = sym("b");
    const Expression inverse = kFactory.divide(con(1.0), sym("q"));
    const Expression input = kFactory.add(kFactory.multiply(inverse, square(kFactory.sin(b))),
                                          kFactory.multiply(inverse, square(kFactory.cos(b))));
    EXPECT_TRUE(ik::structurallyEqual(trig(input), canon(inverse)));
}

TEST(TrigonometricSimplifierTest, ReachesTheFixedPointInOneCall)
{
    // Measured before the fix: the first call returned -a + sin^2(b) + cos^2(b),
    // only a second call reached 1 - a.
    const Expression a = sym("a");
    const Expression input = kFactory.negate(
        kFactory.multiply(kFactory.subtract(a, pythagorean(sym("b"))), pythagorean(sym("c"))));
    const Expression once = trig(input);
    EXPECT_TRUE(ik::structurallyEqual(once, canon(kFactory.subtract(con(1.0), a))));
    EXPECT_TRUE(ik::structurallyEqual(trig(once), once));
}

TEST(TrigonometricSimplifierTest, ReducesPairBeforeItsArgumentCollapsesToAConstant)
{
    // u = 0.1 * (sin^2 a + cos^2 a) simplifies to the constant 0.1. Simplifying
    // children first would fold sin(u)^2 and cos(u)^2 into two numbers that do
    // not add exactly, and the identity would be lost.
    const Expression u = kFactory.multiply(pythagorean(sym("a")), con(0.1));
    EXPECT_TRUE(ik::structurallyEqual(trig(pythagorean(u)), con(1.0)));
}

TEST(TrigonometricSimplifierTest, IsIdempotentOnRandomExpressions)
{
    // Measured before the fix with this generator shape: 22 of 300 000
    // expressions changed again on a second call.
    std::mt19937_64 random{20261120};
    const auto pick = [&](int upper) { return std::uniform_int_distribution<int>(0, upper)(random); };
    static const double constants[] = {0.0, 1.0, -1.0, 2.0, 0.5, 0.1, 3.0, -0.3, 4.0, 0.7};

    const auto leaf = [&]() -> Expression {
        const int choice = pick(8);
        if (choice < 6)
            return kFactory.symbol(std::string(1, static_cast<char>('a' + choice % 3)));
        return con(constants[pick(9)]);
    };
    const auto make = [&](auto&& self, int depth) -> Expression {
        if (depth == 0 || pick(9) == 0)
            return leaf();
        switch (pick(9))
        {
        case 0: case 1: return kFactory.add(self(self, depth - 1), self(self, depth - 1));
        case 2: return kFactory.subtract(self(self, depth - 1), self(self, depth - 1));
        case 3: case 4: return kFactory.multiply(self(self, depth - 1), self(self, depth - 1));
        case 5:
        {
            const Expression numerator = self(self, depth - 1);
            const Expression denominator = self(self, depth - 1);
            return ik::isZero(denominator) ? numerator : kFactory.divide(numerator, denominator);
        }
        case 6: return kFactory.negate(self(self, depth - 1));
        case 7: return kFactory.sin(self(self, depth - 1));
        case 8: return kFactory.cos(self(self, depth - 1));
        default: return pythagorean(self(self, depth - 1));
        }
    };

    for (int sample = 0; sample < 40000; ++sample)
    {
        const Expression expression = make(make, 1 + sample % 6);
        const Expression once = trig(expression);
        ASSERT_TRUE(ik::structurallyEqual(trig(once), once)) << "sample " << sample;
    }
}
