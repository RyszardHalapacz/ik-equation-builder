#include "ik_equations/symbolic/TrigonometricSimplifier.hpp"

#include "ik_equations/symbolic/RewriteEngine.hpp"

#include <algorithm>
#include <cstddef>
#include <optional>
#include <utility>
#include <variant>

namespace kinemaforge::ik {

namespace {

std::optional<Expression> cosineEvenApply(const Expression& expression,
                                          const ExpressionFactory& factory)
{
    const auto* cosine = std::get_if<CosNode>(&expression.node().value);
    if (cosine == nullptr)
        return std::nullopt;

    const auto* negated = std::get_if<NegateNode>(&cosine->operand.node().value);
    if (negated == nullptr)
        return std::nullopt;

    return factory.cos(negated->operand);
}

enum class TrigKind { Sine, Cosine };

// One way of reading a sum term as  sign * cofactors * t(u) * t(u).
struct PairCandidate
{
    TrigKind kind;
    Expression argument;
    std::vector<Expression> cofactors; // every other factor, in canonical order
};

struct TermShape
{
    bool negative{};
    std::vector<PairCandidate> candidates;
};

void flattenProduct(const Expression& expression, std::vector<Expression>& factors)
{
    if (const auto* product = std::get_if<MultiplyNode>(&expression.node().value))
    {
        flattenProduct(product->lhs, factors);
        flattenProduct(product->rhs, factors);
        return;
    }
    factors.push_back(expression);
}

// A term can have several readings -- sin(u)^2 * cos(u)^2 is a sine square
// with cofactors cos(u), cos(u) and a cosine square with cofactors sin(u),
// sin(u) -- so every distinct squared sin/cos factor yields one candidate.
TermShape shapeOf(const Expression& term)
{
    TermShape shape;
    const Expression* body = &term;
    if (const auto* negated = std::get_if<NegateNode>(&term.node().value))
    {
        shape.negative = true;
        body = &negated->operand;
    }

    std::vector<Expression> factors;
    flattenProduct(*body, factors);

    for (std::size_t i = 0; i < factors.size(); ++i)
    {
        const ExpressionType type = factors[i].type();
        if (type != ExpressionType::Sin && type != ExpressionType::Cos)
            continue;

        const bool seenBefore = std::any_of(factors.begin(), factors.begin() + static_cast<std::ptrdiff_t>(i),
                                            [&](const Expression& earlier) {
                                                return structurallyEqual(earlier, factors[i]);
                                            });
        if (seenBefore)
            continue;

        std::size_t partner = factors.size();
        for (std::size_t j = i + 1; j < factors.size(); ++j)
            if (structurallyEqual(factors[j], factors[i]))
            {
                partner = j;
                break;
            }
        if (partner == factors.size())
            continue;

        PairCandidate candidate{
            type == ExpressionType::Sin ? TrigKind::Sine : TrigKind::Cosine,
            type == ExpressionType::Sin
                ? std::get<SinNode>(factors[i].node().value).operand
                : std::get<CosNode>(factors[i].node().value).operand,
            {}};
        for (std::size_t k = 0; k < factors.size(); ++k)
            if (k != i && k != partner)
                candidate.cofactors.push_back(factors[k]);
        std::sort(candidate.cofactors.begin(), candidate.cofactors.end(),
                  [](const Expression& lhs, const Expression& rhs) {
                      return compareExpressions(lhs, rhs) < 0;
                  });
        shape.candidates.push_back(std::move(candidate));
    }
    return shape;
}

bool sameCofactors(const std::vector<Expression>& lhs, const std::vector<Expression>& rhs)
{
    if (lhs.size() != rhs.size())
        return false;
    for (std::size_t i = 0; i < lhs.size(); ++i)
        if (!structurallyEqual(lhs[i], rhs[i]))
            return false;
    return true;
}

// sign * product(cofactors), built so that ExpressionFactory never multiplies
// two constants (that would introduce a rounded constant). Canonical products
// carry at most one constant unless two could not be merged exactly; that
// rare shape is left alone.
std::optional<Expression> collapsedTerm(const std::vector<Expression>& cofactors, bool negative,
                                        const ExpressionFactory& factory)
{
    std::optional<Expression> constant;
    std::optional<Expression> product;
    for (const Expression& factor : cofactors)
    {
        if (isConstant(factor))
        {
            if (constant)
                return std::nullopt;
            constant = factor;
            continue;
        }
        product = product ? factory.multiply(*product, factor) : factor;
    }

    Expression result = factory.constant(1.0);
    if (product && constant)
        result = factory.multiply(*constant, *product);
    else if (product)
        result = *product;
    else if (constant)
        result = *constant;

    return negative ? factory.negate(result) : result;
}

bool allConstants(const std::vector<Expression>& terms)
{
    return std::all_of(terms.begin(), terms.end(), [](const Expression& term) {
        return isConstant(term);
    });
}

std::optional<double> reduceConstantsExactly(std::vector<double> constants)
{
    bool joined = true;
    while (joined && constants.size() > 1)
    {
        joined = false;
        std::sort(constants.begin(), constants.end());
        std::vector<double> reduced;
        reduced.push_back(constants.front());
        for (std::size_t i = 1; i < constants.size(); ++i)
        {
            if (detail::addsExactly(reduced.back(), constants[i]))
            {
                reduced.back() += constants[i];
                joined = true;
            }
            else
            {
                reduced.push_back(constants[i]);
            }
        }
        constants = std::move(reduced);
    }

    if (constants.size() != 1)
        return std::nullopt;
    return constants.front();
}

} // namespace

RewriteRule TrigonometricRules::cosineEven()
{
    return RewriteRule{"cos(-x)->cos(x)", cosineEvenApply};
}

TrigonometricSimplificationSession::TrigonometricSimplificationSession(ExpressionFactory factory)
    : factory_(std::move(factory))
    , canonicalizer_(factory_)
{
}

Expression TrigonometricSimplificationSession::simplify(const Expression& expression)
{
    // One pass can expose work for the next: a negation pushed into a sum by
    // the final canonicalisation, a factor that collapsed to 1. Iterating to a
    // fixed point is what makes simplify() idempotent by construction. Every
    // productive round removes nodes, so the bound is a safety net, not a
    // tuning knob.
    constexpr int maximumRounds = 32;
    Expression current = simplifyNode(canonicalizer_.canonicalize(expression));
    for (int round = 0; round < maximumRounds; ++round)
    {
        const Expression next = simplifyNode(canonicalizer_.canonicalize(current));
        if (sameNode(next, current) || structurallyEqual(next, current))
            break;
        current = next;
    }
    return current;
}

// Replaces every pair of terms that differ only in sin/cos of the same
// argument. Returns whether anything was replaced; `terms` is updated in place.
bool TrigonometricSimplificationSession::reducePythagoreanPairs(std::vector<Expression>& terms)
{
    bool reducedAny = false;
    bool reduced = true;
    while (reduced)
    {
        reduced = false;

        std::vector<TermShape> shapes;
        shapes.reserve(terms.size());
        for (const Expression& term : terms)
            shapes.push_back(shapeOf(term));

        for (std::size_t i = 0; i < terms.size() && !reduced; ++i)
            for (std::size_t j = i + 1; j < terms.size() && !reduced; ++j)
            {
                if (shapes[i].negative != shapes[j].negative)
                    continue;
                for (const PairCandidate& left : shapes[i].candidates)
                {
                    const auto right = std::find_if(
                        shapes[j].candidates.begin(), shapes[j].candidates.end(),
                        [&](const PairCandidate& candidate) {
                            return candidate.kind != left.kind &&
                                   structurallyEqual(candidate.argument, left.argument) &&
                                   sameCofactors(candidate.cofactors, left.cofactors);
                        });
                    if (right == shapes[j].candidates.end())
                        continue;

                    // sin^2(u)+cos^2(u)=1 is algebraically true wherever u is
                    // defined, but dropping u would widen the symbolic domain
                    // if u itself contains a divisor. The cofactors stay in the
                    // result, so their divisors are preserved.
                    if (!divisorsOf(left.argument).empty())
                        continue;

                    const auto collapsed = collapsedTerm(left.cofactors, shapes[i].negative, factory_);
                    if (!collapsed)
                        continue;

                    const Expression replacement = canonicalizer_.canonicalize(*collapsed);
                    terms.erase(terms.begin() + static_cast<std::ptrdiff_t>(j));
                    terms.erase(terms.begin() + static_cast<std::ptrdiff_t>(i));
                    if (replacement.type() == ExpressionType::Add)
                        collectAddTerms(replacement, terms);
                    else
                        terms.push_back(replacement);
                    reduced = true;
                    reducedAny = true;
                    break;
                }
            }
    }
    return reducedAny;
}

void TrigonometricSimplificationSession::collectAddTerms(
    const Expression& expression, std::vector<Expression>& terms)
{
    if (const auto* add = std::get_if<AddNode>(&expression.node().value))
    {
        collectAddTerms(add->lhs, terms);
        collectAddTerms(add->rhs, terms);
        return;
    }
    terms.push_back(expression);
}

Expression TrigonometricSimplificationSession::rebuildSum(
    const std::vector<Expression>& terms, const Expression& fallback)
{
    if (terms.empty())
        return factory_.constant(0.0);
    if (terms.size() == 1)
        return terms.front();

    // If all remaining terms are constants, constructing even one Add through
    // ExpressionFactory may introduce a new rounded constant. Reduce only when
    // the complete result is exactly representable; otherwise keep the old
    // subtree as a representability barrier.
    if (allConstants(terms))
    {
        std::vector<double> values;
        values.reserve(terms.size());
        for (const Expression& term : terms)
            values.push_back(constantValue(term));
        if (const auto exact = reduceConstantsExactly(std::move(values)))
            return factory_.constant(*exact);
        return fallback;
    }

    // Start from a non-constant term so no factory.add call ever receives two
    // constants. F3.2 will flatten, order and balance the result afterwards.
    const auto firstNonConstant = std::find_if(terms.begin(), terms.end(), [](const Expression& term) {
        return !isConstant(term);
    });

    Expression result = *firstNonConstant;
    for (auto it = terms.begin(); it != terms.end(); ++it)
    {
        if (it == firstNonConstant)
            continue;
        result = factory_.add(std::move(result), *it);
    }
    return canonicalizer_.canonicalize(result);
}

Expression TrigonometricSimplificationSession::simplifySum(const Expression& expression)
{
    // Pairs are looked for BEFORE the children are simplified as well as after.
    // Before: if u simplifies to a constant, sin(u)*sin(u) folds into a number
    // and the pattern is gone. After: sin(-u)^2 + cos(u)^2 only becomes a pair
    // once the odd/even rules have normalised the arguments.
    constexpr int maximumRounds = 32;
    Expression current = expression;
    for (int round = 0; round < maximumRounds; ++round)
    {
        if (current.type() != ExpressionType::Add)
            return simplifyNode(current);

        std::vector<Expression> rawTerms;
        collectAddTerms(current, rawTerms);

        bool changed = reducePythagoreanPairs(rawTerms);

        std::vector<Expression> terms;
        terms.reserve(rawTerms.size());
        for (const Expression& raw : rawTerms)
        {
            const Expression simplified = simplifyNode(raw);
            if (!sameNode(simplified, raw))
                changed = true;
            if (simplified.type() == ExpressionType::Add)
                collectAddTerms(simplified, terms);
            else
                terms.push_back(simplified);
        }

        if (reducePythagoreanPairs(terms))
            changed = true;

        if (!changed)
            return current;

        // rebuildSum() hands `current` back unchanged at a representability
        // barrier (only constants left that do not add exactly).
        const Expression next = rebuildSum(terms, current);
        if (sameNode(next, current) || structurallyEqual(next, current))
            return current;
        current = next;
    }
    return current;
}

Expression TrigonometricSimplificationSession::simplifyNode(const Expression& expression)
{
    if (const auto cached = memo_.find(expression); cached != memo_.end())
        return cached->second;

    Expression result = expression;

    switch (expression.type())
    {
    case ExpressionType::Constant:
    case ExpressionType::Symbol:
        break;

    case ExpressionType::Add:
        result = simplifySum(expression);
        break;

    case ExpressionType::Subtract:
        // A Subtract surviving F3.2 is inside a representability barrier.
        // Preserve it verbatim; F3.3 must not accidentally dissolve that barrier.
        break;

    case ExpressionType::Multiply:
    {
        const auto& node = std::get<MultiplyNode>(expression.node().value);
        const Expression lhs = simplifyNode(node.lhs);
        const Expression rhs = simplifyNode(node.rhs);
        result = canonicalizer_.canonicalize(factory_.multiply(lhs, rhs));
        break;
    }

    case ExpressionType::Divide:
    {
        const auto& node = std::get<DivideNode>(expression.node().value);
        const Expression lhs = simplifyNode(node.lhs);
        Expression rhs = simplifyNode(node.rhs);
        if (isZero(rhs))
            rhs = node.rhs; // representability guard: never build literal /0
        result = canonicalizer_.canonicalize(factory_.divide(lhs, rhs));
        break;
    }

    case ExpressionType::Negate:
    {
        const auto& node = std::get<NegateNode>(expression.node().value);
        // Negate(Add) can survive only at a F3.2 representability barrier.
        if (node.operand.type() == ExpressionType::Add)
            break;
        result = canonicalizer_.canonicalize(factory_.negate(simplifyNode(node.operand)));
        break;
    }

    case ExpressionType::Sin:
    {
        const auto& node = std::get<SinNode>(expression.node().value);
        Expression operand = simplifyNode(node.operand);
        if (const auto* negative = std::get_if<NegateNode>(&operand.node().value))
            result = canonicalizer_.canonicalize(
                factory_.negate(factory_.sin(negative->operand)));
        else
            result = factory_.sin(std::move(operand));
        break;
    }

    case ExpressionType::Cos:
    {
        const auto& node = std::get<CosNode>(expression.node().value);
        Expression operand = simplifyNode(node.operand);
        if (const auto* negative = std::get_if<NegateNode>(&operand.node().value))
            result = factory_.cos(negative->operand);
        else
            result = factory_.cos(std::move(operand));
        break;
    }
    }

    result = canonicalizer_.canonicalize(result);
    if (!sameNode(result, expression) && structurallyEqual(result, expression))
        result = expression;

    // -(x * (a - b)) canonicalises into a sum whose terms nobody has examined
    // as a sum yet. simplifyNode on an Add goes through simplifySum, which
    // never re-enters this branch, so the recursion is one level deep.
    if (expression.type() != ExpressionType::Add && result.type() == ExpressionType::Add &&
        !sameNode(result, expression))
        result = simplifyNode(result);

    memo_.emplace(expression, result);
    return result;
}

Expression simplifyTrigonometry(const Expression& expression, const ExpressionFactory& factory)
{
    TrigonometricSimplificationSession session{factory};
    return session.simplify(expression);
}

} // namespace kinemaforge::ik
