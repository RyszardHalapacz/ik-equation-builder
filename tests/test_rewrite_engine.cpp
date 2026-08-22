#include <gtest/gtest.h>

#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/symbolic/ExpressionEvaluator.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/RewriteEngine.hpp"

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace ik = kinemaforge::ik;

using ik::Expression;
using ik::ExpressionFactory;
using ik::RewriteResult;
using ik::RewriteRule;
using ik::RewriteSession;
using ik::sameNode;
using ik::structurallyEqual;

namespace {

const ExpressionFactory kFactory;

// Diagnostic counters. Legal under the determinism contract precisely because
// nothing below reads them to decide anything -- they are written, never
// branched on.
std::size_t g_subtractionConsultations = 0;

void resetCounters()
{
    g_subtractionConsultations = 0;
}

// x - x -> 0. Domain-safe when x carries no division, rejected when it does.
std::optional<Expression> selfSubtraction(const Expression& expression,
                                          const ExpressionFactory& factory)
{
    const auto* node = std::get_if<ik::SubtractNode>(&expression.node().value);
    if (node == nullptr)
        return std::nullopt;

    ++g_subtractionConsultations;

    if (!structurallyEqual(node->lhs, node->rhs))
        return std::nullopt;

    return factory.constant(0.0);
}

// -(-x) -> x
std::optional<Expression> doubleNegation(const Expression& expression, const ExpressionFactory&)
{
    const auto* outer = std::get_if<ik::NegateNode>(&expression.node().value);
    if (outer == nullptr)
        return std::nullopt;

    const auto* inner = std::get_if<ik::NegateNode>(&outer->operand.node().value);
    if (inner == nullptr)
        return std::nullopt;

    return inner->operand;
}

// x * 0 -> 0. THE unsafe one: correct wherever x is defined, and defined in
// more places than x is.
std::optional<Expression> multiplyByZero(const Expression& expression,
                                         const ExpressionFactory& factory)
{
    const auto* node = std::get_if<ik::MultiplyNode>(&expression.node().value);
    if (node == nullptr)
        return std::nullopt;

    if (!ik::isZero(node->lhs) && !ik::isZero(node->rhs))
        return std::nullopt;

    return factory.constant(0.0);
}

// x / x -> 1. Also unsafe: it drops the guard x != 0.
std::optional<Expression> selfDivision(const Expression& expression,
                                       const ExpressionFactory& factory)
{
    const auto* node = std::get_if<ik::DivideNode>(&expression.node().value);
    if (node == nullptr || !structurallyEqual(node->lhs, node->rhs))
        return std::nullopt;

    return factory.constant(1.0);
}

// The counterexample from the architecture: a rule that BUILDS material the
// input never contained. (q+q) + (r-r) = 2q = 2 * -(-q); measure 7 -> 5.
Expression counterexampleInput()
{
    const Expression q = kFactory.symbol("q");
    const Expression r = kFactory.symbol("r");
    return kFactory.add(kFactory.add(q, q), kFactory.subtract(r, r));
}

std::optional<Expression> introducesDoubleNegation(const Expression& expression,
                                                   const ExpressionFactory& factory)
{
    if (!structurallyEqual(expression, counterexampleInput()))
        return std::nullopt;

    const Expression q = factory.symbol("q");
    return factory.multiply(factory.constant(2.0), factory.negate(factory.negate(q)));
}

// Same input, different admissible result: (q+q) + (r-r) -> q + q. Measure 7 -> 3.
std::optional<Expression> collapsesToPlainSum(const Expression& expression,
                                              const ExpressionFactory& factory)
{
    if (!structurallyEqual(expression, counterexampleInput()))
        return std::nullopt;

    const Expression q = factory.symbol("q");
    return factory.add(q, q);
}

// (a/b) * 0 -> 0/b. Equal to the input ON THE SAME DOMAIN -- both are 0 wherever
// b != 0 and both are undefined at b == 0 -- and measure 5 -> 3. It exists to be
// the SECOND rule tried at a node where the first one was rejected.
std::optional<Expression> zeroFactorIntoNumerator(const Expression& expression,
                                                  const ExpressionFactory& factory)
{
    const auto* node = std::get_if<ik::MultiplyNode>(&expression.node().value);
    if (node == nullptr || !ik::isZero(node->rhs))
        return std::nullopt;

    const auto* ratio = std::get_if<ik::DivideNode>(&node->lhs.node().value);
    if (ratio == nullptr)
        return std::nullopt;

    // ratio->rhs cannot be a literal zero: the factory would never have built
    // that Divide in the first place.
    return factory.divide(factory.constant(0.0), ratio->rhs);
}

RewriteRule rule(std::string name, std::optional<Expression> (*apply)(const Expression&,
                                                                     const ExpressionFactory&))
{
    return RewriteRule{std::move(name), apply};
}

RewriteSession sessionOf(std::vector<RewriteRule> rules,
                         std::size_t stepLimit = ik::kDefaultRewriteStepLimit)
{
    return RewriteSession{rules, kFactory, stepLimit};
}

} // namespace

TEST(RewriteEngineTest, CollectsDivisorsFromNestedExpression)
{
    const Expression a = kFactory.symbol("a");
    const Expression x = kFactory.symbol("x");
    const Expression y = kFactory.symbol("y");

    // a/x + (a/x)/y -- x appears twice and must be reported once.
    const Expression nested = kFactory.add(kFactory.divide(a, x),
                                           kFactory.divide(kFactory.divide(a, x), y));

    const std::vector<Expression> divisors = ik::divisorsOf(nested);

    ASSERT_EQ(divisors.size(), 2u);
    EXPECT_TRUE(structurallyEqual(divisors[0], x) || structurallyEqual(divisors[1], x));
    EXPECT_TRUE(structurallyEqual(divisors[0], y) || structurallyEqual(divisors[1], y));
}

TEST(RewriteEngineTest, RejectsRewriteThatWidensDomain)
{
    const Expression q = kFactory.symbol("q");
    const Expression unsafe = kFactory.multiply(kFactory.divide(kFactory.constant(1.0), q),
                                                kFactory.constant(0.0));

    RewriteSession session = sessionOf({rule("x*0", multiplyByZero)});
    const RewriteResult result = session.rewrite(unsafe);

    EXPECT_FALSE(result.changed);
    EXPECT_TRUE(sameNode(result.expression, unsafe));
}

TEST(RewriteEngineTest, AllowsSameRewriteWhenNoDivisionPresent)
{
    const Expression q = kFactory.symbol("q");
    const Expression safe = kFactory.multiply(q, kFactory.constant(0.0));

    RewriteSession session = sessionOf({rule("x*0", multiplyByZero)});
    const RewriteResult result = session.rewrite(safe);

    EXPECT_TRUE(result.changed);
    EXPECT_TRUE(ik::isZero(result.expression));
}

TEST(RewriteEngineTest, RejectsCancellationOfSelfDivision)
{
    const Expression q = kFactory.symbol("q");
    const Expression ratio = kFactory.divide(q, q);

    RewriteSession session = sessionOf({rule("x/x", selfDivision)});
    const RewriteResult result = session.rewrite(ratio);

    EXPECT_FALSE(result.changed);
}

// The test that would fail under a GLOBAL comparison of divisors: guards go
// from { -(-q) } to { q }, which are different sets, and the rewrite is still
// correct. The condition is local, so this passes.
TEST(RewriteEngineTest, RewritesInsideDenominator)
{
    const Expression a = kFactory.symbol("a");
    const Expression q = kFactory.symbol("q");
    const Expression awkward = kFactory.divide(a, kFactory.negate(kFactory.negate(q)));

    RewriteSession session = sessionOf({rule("-(-x)", doubleNegation)});
    const RewriteResult result = session.rewrite(awkward);

    EXPECT_TRUE(result.changed);
    EXPECT_TRUE(structurallyEqual(result.expression, kFactory.divide(a, q)));
}

TEST(RewriteEngineTest, KeepsOriginalDenominatorWhenRewriteYieldsZero)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);
    const Expression ratio = kFactory.divide(kFactory.constant(1.0), shared);

    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});
    const RewriteResult result = session.rewrite(ratio);

    EXPECT_TRUE(sameNode(result.expression, ratio));
}

TEST(RewriteEngineTest, StillRewritesSameSubtreeElsewhere)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);

    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});
    const RewriteResult result = session.rewrite(kFactory.add(shared, kFactory.constant(5.0)));

    // add(0, 5) folds in the factory, so the whole thing collapses to 5.
    EXPECT_TRUE(result.changed);
    EXPECT_TRUE(ik::isConstant(result.expression));
    EXPECT_DOUBLE_EQ(ik::constantValue(result.expression), 5.0);
}

TEST(RewriteEngineTest, ReportsChangedWhenARuleFires)
{
    const Expression q = kFactory.symbol("q");
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});

    const RewriteResult result = session.rewrite(kFactory.subtract(q, q));

    EXPECT_TRUE(result.changed);
    EXPECT_EQ(result.steps, 1u);
    EXPECT_FALSE(result.reachedLimit);
}

TEST(RewriteEngineTest, ReportsUnchangedOnFixedPoint)
{
    const Expression q = kFactory.symbol("q");
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});

    const RewriteResult result = session.rewrite(kFactory.add(q, kFactory.symbol("r")));

    EXPECT_FALSE(result.changed);
    EXPECT_EQ(result.steps, 0u);
}

// The architecture's blocker, as a test: after the first rule fires, the fresh
// -(-q) it built must be normalised before anything is called a fixed point.
TEST(RewriteEngineTest, RewritesFreshDescendantsIntroducedByRule)
{
    RewriteSession session = sessionOf({rule("A->2*-(-q)", introducesDoubleNegation),
                                        rule("-(-x)", doubleNegation)});

    const RewriteResult result = session.rewrite(counterexampleInput());

    const Expression expected = kFactory.multiply(kFactory.constant(2.0), kFactory.symbol("q"));
    EXPECT_TRUE(structurallyEqual(result.expression, expected));
    EXPECT_EQ(result.steps, 2u);
}

TEST(RewriteEngineTest, StopsAtStepLimitAndSaysSo)
{
    const Expression q = kFactory.symbol("q");
    const Expression r = kFactory.symbol("r");
    const Expression twoPairs = kFactory.add(kFactory.subtract(q, q), kFactory.subtract(r, r));

    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)}, 1);
    const RewriteResult result = session.rewrite(twoPairs);

    EXPECT_TRUE(result.reachedLimit);
    EXPECT_EQ(result.steps, 1u);

    // The partial result is still a valid expression with the same value.
    ik::ExpressionEvaluator evaluator{{{"q", 0.3}, {"r", -1.25}}};
    const auto value = evaluator.evaluate(result.expression);
    ASSERT_TRUE(value.has_value());
    EXPECT_DOUBLE_EQ(*value, 0.0);
}

// root1 from the architecture's worked example: a rule fires, the guard drops
// it at the parent, the output is identical to the input.
TEST(RewriteEngineTest, ReportsUnchangedWhenGuardRejectsInDenominator)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);
    const Expression root1 = kFactory.divide(kFactory.constant(1.0), shared);

    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});
    const RewriteResult result = session.rewrite(root1);

    EXPECT_FALSE(result.changed);
    EXPECT_EQ(result.steps, 1u);
}

// root2 from the same example, in the SAME session: the output differs while no
// rule fires here, because the memo already holds the answer.
TEST(RewriteEngineTest, ReportsChangedWithoutStepsOnMemoHit)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);

    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});

    const RewriteResult first = session.rewrite(kFactory.divide(kFactory.constant(1.0), shared));
    ASSERT_EQ(first.steps, 1u);

    const RewriteResult second = session.rewrite(kFactory.add(shared, kFactory.constant(5.0)));

    EXPECT_TRUE(second.changed);
    EXPECT_EQ(second.steps, 0u);
}

TEST(RewriteEngineTest, KeepsNodeIdentityWhenNothingApplies)
{
    const Expression q = kFactory.symbol("q");
    const Expression tree = kFactory.add(kFactory.sin(q), kFactory.cos(q));

    RewriteSession session = sessionOf({rule("-(-x)", doubleNegation)});
    const RewriteResult result = session.rewrite(tree);

    EXPECT_TRUE(sameNode(result.expression, tree));
    EXPECT_FALSE(result.changed);
}

// Innermost order, measured by step count: rewriting q-q first lets the factory
// fold both negations away in one step. Outermost order would take two.
TEST(RewriteEngineTest, AppliesRulesInnermostFirst)
{
    const Expression q = kFactory.symbol("q");
    const Expression nested = kFactory.negate(kFactory.negate(kFactory.subtract(q, q)));

    RewriteSession session = sessionOf({rule("-(-x)", doubleNegation),
                                        rule("x-x", selfSubtraction)});
    const RewriteResult result = session.rewrite(nested);

    EXPECT_TRUE(ik::isZero(result.expression));
    EXPECT_EQ(result.steps, 1u);
}

// The rejected rule is tried FIRST at this node; the second rule must still get
// its turn AT THE SAME NODE. This is the claim the architecture makes, and the
// traversal test below does not prove it.
TEST(RewriteEngineTest, ContinuesWithNextRuleAfterDomainRejection)
{
    const Expression q = kFactory.symbol("q");
    const Expression unsafe = kFactory.multiply(kFactory.divide(kFactory.constant(1.0), q),
                                                kFactory.constant(0.0));

    RewriteSession session = sessionOf({rule("x*0", multiplyByZero),
                                        rule("(a/b)*0 -> 0/b", zeroFactorIntoNumerator)});
    const RewriteResult result = session.rewrite(unsafe);

    EXPECT_TRUE(result.changed);
    EXPECT_EQ(result.steps, 1u);
    EXPECT_TRUE(structurallyEqual(result.expression,
                                  kFactory.divide(kFactory.constant(0.0), q)));
}

// A rule rejected at one node does not abort the pass: the engine keeps
// rewriting elsewhere in the same tree.
TEST(RewriteEngineTest, RejectedRuleDoesNotStopTheTraversal)
{
    const Expression q = kFactory.symbol("q");
    const Expression r = kFactory.symbol("r");
    const Expression unsafe = kFactory.multiply(kFactory.divide(kFactory.constant(1.0), q),
                                                kFactory.constant(0.0));
    const Expression tree = kFactory.add(unsafe, kFactory.subtract(r, r));

    RewriteSession session = sessionOf({rule("x*0", multiplyByZero),
                                        rule("x-x", selfSubtraction)});
    const RewriteResult result = session.rewrite(tree);

    EXPECT_EQ(result.steps, 1u);
    EXPECT_TRUE(structurallyEqual(result.expression, unsafe));
}

TEST(RewriteEngineTest, UsesFirstAdmissibleRuleInDeclaredOrder)
{
    const Expression q = kFactory.symbol("q");

    RewriteSession firstWins = sessionOf({rule("A->2*-(-q)", introducesDoubleNegation),
                                          rule("A->q+q", collapsesToPlainSum)});
    const RewriteResult a = firstWins.rewrite(counterexampleInput());

    RewriteSession reversed = sessionOf({rule("A->q+q", collapsesToPlainSum),
                                         rule("A->2*-(-q)", introducesDoubleNegation)});
    const RewriteResult b = reversed.rewrite(counterexampleInput());

    EXPECT_TRUE(structurallyEqual(
        a.expression,
        kFactory.multiply(kFactory.constant(2.0), kFactory.negate(kFactory.negate(q)))));
    EXPECT_TRUE(structurallyEqual(b.expression, kFactory.add(q, q)));
    EXPECT_FALSE(structurallyEqual(a.expression, b.expression));
}

TEST(RewriteEngineTest, PreservesSharingAcrossRewrite)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.negate(kFactory.negate(q));
    const Expression tree = kFactory.add(shared, shared);

    RewriteSession session = sessionOf({rule("-(-x)", doubleNegation)});
    const RewriteResult result = session.rewrite(tree);

    const auto* node = std::get_if<ik::AddNode>(&result.expression.node().value);
    ASSERT_NE(node, nullptr);
    EXPECT_TRUE(sameNode(node->lhs, node->rhs));
}

TEST(RewriteEngineTest, RewritesSharedSubtreeOnce)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);

    // Eight occurrences of one node.
    Expression tree = shared;
    for (int level = 0; level < 3; ++level)
        tree = kFactory.add(tree, tree);

    resetCounters();
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});
    const RewriteResult result = session.rewrite(tree);

    EXPECT_TRUE(ik::isZero(result.expression));
    EXPECT_EQ(g_subtractionConsultations, 1u);
    EXPECT_EQ(result.steps, 1u);
}

TEST(RewriteEngineTest, SharesMemoAcrossRootsInOneSession)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);

    resetCounters();
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});

    const RewriteResult first = session.rewrite(kFactory.add(shared, kFactory.symbol("a")));
    ASSERT_FALSE(first.reachedLimit);
    ASSERT_TRUE(first.changed);
    ASSERT_EQ(first.steps, 1u);
    ASSERT_EQ(g_subtractionConsultations, 1u);

    const RewriteResult second = session.rewrite(kFactory.multiply(shared, kFactory.symbol("b")));

    EXPECT_EQ(g_subtractionConsultations, 1u);
    EXPECT_EQ(second.steps, 0u);
    EXPECT_TRUE(second.changed);
}

// The memo key owns its node, so an entry made while rewriting a temporary
// cannot be answered for a different node that later reuses the address.
//
// Stated honestly: this cannot deterministically FAIL for a raw-pointer key --
// it depends on the allocator reusing an address. It exercises the pattern,
// churns allocations to make reuse likely, and the actual protection is the
// key type, not this test.
TEST(RewriteEngineTest, SurvivesRewriteOfTemporaryExpression)
{
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});

    for (int iteration = 0; iteration < 200; ++iteration)
    {
        const Expression q = kFactory.symbol("q" + std::to_string(iteration));
        const RewriteResult result = session.rewrite(
            kFactory.add(kFactory.subtract(q, q), kFactory.constant(iteration)));

        ASSERT_TRUE(ik::isConstant(result.expression)) << iteration;
        ASSERT_DOUBLE_EQ(ik::constantValue(result.expression), iteration) << iteration;
    }
}

// A call stopped by the limit must leave NOTHING in the session: the next root
// has to consult the rules again rather than pick up an unfinished result.
TEST(RewriteEngineTest, DoesNotReuseIncompleteMemoAfterStepLimit)
{
    const Expression q = kFactory.symbol("q");
    const Expression r = kFactory.symbol("r");
    const Expression shared = kFactory.subtract(q, q);

    resetCounters();
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)}, 1);

    const RewriteResult interrupted =
        session.rewrite(kFactory.add(shared, kFactory.subtract(r, r)));
    ASSERT_TRUE(interrupted.reachedLimit);

    const std::size_t afterInterrupted = g_subtractionConsultations;

    RewriteResult second = session.rewrite(shared);

    EXPECT_GT(g_subtractionConsultations, afterInterrupted);
    EXPECT_TRUE(ik::isZero(second.expression));
    EXPECT_EQ(second.steps, 1u);
}

TEST(RewriteEngineTest, CommitsMemoOnlyAfterReachingFixedPoint)
{
    const Expression q = kFactory.symbol("q");
    const Expression shared = kFactory.subtract(q, q);

    resetCounters();
    RewriteSession session = sessionOf({rule("x-x", selfSubtraction)});

    const RewriteResult complete = session.rewrite(kFactory.add(shared, kFactory.symbol("a")));
    ASSERT_FALSE(complete.reachedLimit);

    const std::size_t afterComplete = g_subtractionConsultations;
    const RewriteResult second = session.rewrite(shared);

    EXPECT_EQ(g_subtractionConsultations, afterComplete);
    EXPECT_EQ(second.steps, 0u);
}

// The session copies its rules. Overwriting the caller's array afterwards must
// not change what the session does -- with a span it would.
TEST(RewriteEngineTest, OutlivesTheRuleArrayItWasBuiltFrom)
{
    const Expression q = kFactory.symbol("q");

    std::vector<RewriteRule> rules{rule("x-x", selfSubtraction)};
    RewriteSession session{rules, kFactory};

    rules[0] = rule("-(-x)", doubleNegation);
    rules.clear();

    const RewriteResult result = session.rewrite(kFactory.subtract(q, q));

    EXPECT_TRUE(ik::isZero(result.expression));
    EXPECT_EQ(result.steps, 1u);
}

// Definedness, not just value -- and with the pole IN the point set by name.
TEST(RewriteEngineTest, AgreesOnDefinednessAndValue)
{
    const Expression q1 = kFactory.symbol("q1");
    const Expression denominator = kFactory.subtract(q1, kFactory.constant(0.5));
    const Expression original = kFactory.multiply(kFactory.divide(kFactory.symbol("a"), denominator),
                                                  kFactory.constant(0.0));

    RewriteSession session = sessionOf({rule("x*0", multiplyByZero)});
    const RewriteResult result = session.rewrite(original);

    // Deterministic boundary points first, then a spread. 0.5 EXACTLY is the
    // only point at which the rejected rewrite would differ, so it is not left
    // to sampling.
    const std::array<double, 9> points{0.5,   0.5 - 1e-9, 0.5 + 1e-9, 0.5 - 1e-6, 0.5 + 1e-6,
                                       -2.0,  0.0,        1.25,       3.5};

    for (const double point : points)
    {
        ik::ExpressionEvaluator before{{{"q1", point}, {"a", 2.0}}};
        ik::ExpressionEvaluator after{{{"q1", point}, {"a", 2.0}}};

        const auto originalValue = before.evaluate(original);
        const auto rewrittenValue = after.evaluate(result.expression);

        ASSERT_EQ(originalValue.has_value(), rewrittenValue.has_value()) << point;

        // Braces are REQUIRED, not style: EXPECT_NEAR expands to an if/else, so
        // an unbraced body trips -Werror=dangling-else.
        if (originalValue.has_value())
        {
            EXPECT_NEAR(*originalValue, *rewrittenValue, 1e-12) << point;
        }
    }

    // ...and the point set has teeth: at the pole the REJECTED replacement
    // would have disagreed. Without this the test above could pass for an
    // engine that accepted the unsafe rewrite.
    ik::ExpressionEvaluator atPole{{{"q1", 0.5}, {"a", 2.0}}};
    ik::ExpressionEvaluator atPoleFolded{{{"q1", 0.5}, {"a", 2.0}}};
    EXPECT_FALSE(atPole.evaluate(original).has_value());
    EXPECT_TRUE(atPoleFolded.evaluate(kFactory.constant(0.0)).has_value());
}

// The other half: an ACCEPTED rewrite must preserve definedness too. Guards go
// from { -(-q) } to { q } -- different sets, same domain -- which is exactly the
// case a global comparison of divisors would have rejected.
TEST(RewriteEngineTest, AgreesOnDefinednessAcrossAnAcceptedRewrite)
{
    const Expression a = kFactory.symbol("a");
    const Expression q = kFactory.symbol("q");
    const Expression awkward = kFactory.divide(a, kFactory.negate(kFactory.negate(q)));

    RewriteSession session = sessionOf({rule("-(-x)", doubleNegation)});
    const RewriteResult result = session.rewrite(awkward);

    ASSERT_TRUE(result.changed);
    ASSERT_TRUE(structurallyEqual(result.expression, kFactory.divide(a, q)));

    // Zero EXACTLY, then either side of it: the pole is where a widened domain
    // would show, and it is in the list by name rather than by sampling.
    const std::array<double, 5> points{0.0, -1e-9, 1e-9, -1.0, 1.0};

    for (const double point : points)
    {
        ik::ExpressionEvaluator before{{{"q", point}, {"a", 3.0}}};
        ik::ExpressionEvaluator after{{{"q", point}, {"a", 3.0}}};

        const auto originalValue = before.evaluate(awkward);
        const auto rewrittenValue = after.evaluate(result.expression);

        ASSERT_EQ(originalValue.has_value(), rewrittenValue.has_value()) << point;

        // Braces are REQUIRED, not style: EXPECT_NEAR expands to an if/else, so
        // an unbraced body trips -Werror=dangling-else.
        if (originalValue.has_value())
        {
            EXPECT_NEAR(*originalValue, *rewrittenValue, 1e-12) << point;
        }
    }
}

// End-to-end on the real 21 882-node DAG: each cell wrapped in a double
// negation the engine must peel off, returning the ORIGINAL nodes.
//
// The rule fires exactly at the twelve wrapped roots and nowhere else, which
// is a measured fact and not an assumption: both FK DAGs contain six Negate
// nodes each and NOT ONE Negate(Negate(...)). Four cells per robot are
// constants -- the [0 0 0 1] row -- and those fold at build time.
TEST(RewriteEngineTest, RewritesRealForwardKinematicsAndKeepsItsNodes)
{
    ik::UrdfModelLoader loader;
    const auto robot = loader.load(std::filesystem::path(KINEMAFORGE_URDF_DATA_DIR) / "kr640.urdf");

    ik::KinematicChainBuilder chainBuilder;
    const auto chain = chainBuilder.build(robot, "base_link", "tool0");
    ASSERT_TRUE(chain.has_value());

    const ik::JointTransformBuilder transformBuilder;
    const ik::ForwardKinematicsBuilder fkBuilder;
    const ik::SymbolicTransform fk = fkBuilder.build(*chain, transformBuilder);

    RewriteSession session = sessionOf({rule("-(-x)", doubleNegation)});

    for (std::size_t row = 0; row < ik::SymbolicTransform::rows; ++row)
        for (std::size_t column = 0; column < ik::SymbolicTransform::columns; ++column)
        {
            const Expression& cell = fk(row, column);
            const RewriteResult result =
                session.rewrite(kFactory.negate(kFactory.negate(cell)));

            EXPECT_TRUE(structurallyEqual(result.expression, cell)) << row << ',' << column;

            // Constant cells fold at build time -- negate(constant) collapses --
            // so the wrap never existed and the identity claim does not apply.
            // Everywhere else the engine must hand back the very node it was
            // given, not a rebuilt copy of it.
            if (!ik::isConstant(cell))
            {
                EXPECT_TRUE(sameNode(result.expression, cell)) << row << ',' << column;
            }
        }
}
