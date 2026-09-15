#include "ik_equations/symbolic/TrigonometricSimplifier.hpp"

#include "ik_equations/symbolic/RewriteEngine.hpp"

#include <algorithm>
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

struct TrigSquare
{
    TrigKind kind;
    Expression argument;
};

std::optional<TrigSquare> trigSquare(const Expression& expression)
{
    const auto* product = std::get_if<MultiplyNode>(&expression.node().value);
    if (product == nullptr || !structurallyEqual(product->lhs, product->rhs))
        return std::nullopt;

    if (const auto* sine = std::get_if<SinNode>(&product->lhs.node().value))
        return TrigSquare{TrigKind::Sine, sine->operand};
    if (const auto* cosine = std::get_if<CosNode>(&product->lhs.node().value))
        return TrigSquare{TrigKind::Cosine, cosine->operand};
    return std::nullopt;
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
    const Expression canonical = canonicalizer_.canonicalize(expression);
    return simplifyNode(canonical);
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
    std::vector<Expression> rawTerms;
    collectAddTerms(expression, rawTerms);

    std::vector<Expression> terms;
    for (const Expression& raw : rawTerms)
    {
        const Expression simplified = simplifyNode(raw);
        if (simplified.type() == ExpressionType::Add)
            collectAddTerms(simplified, terms);
        else
            terms.push_back(simplified);
    }

    std::vector<bool> removed(terms.size(), false);
    std::size_t replacements = 0;

    for (std::size_t i = 0; i < terms.size(); ++i)
    {
        if (removed[i])
            continue;
        const auto left = trigSquare(terms[i]);
        if (!left)
            continue;

        for (std::size_t j = i + 1; j < terms.size(); ++j)
        {
            if (removed[j])
                continue;
            const auto right = trigSquare(terms[j]);
            if (!right || left->kind == right->kind)
                continue;
            if (!structurallyEqual(left->argument, right->argument))
                continue;

            // sin^2(u)+cos^2(u)=1 is algebraically true wherever u is defined,
            // but replacing it by literal 1 would widen the symbolic domain if
            // u itself contains a divisor.
            if (!divisorsOf(left->argument).empty())
                continue;

            removed[i] = true;
            removed[j] = true;
            ++replacements;
            break;
        }
    }

    const bool childrenChanged = [&] {
        if (terms.size() != rawTerms.size())
            return true;
        for (std::size_t i = 0; i < terms.size(); ++i)
            if (!sameNode(terms[i], rawTerms[i]))
                return true;
        return false;
    }();

    if (replacements == 0 && !childrenChanged)
        return expression;

    std::vector<Expression> rebuiltTerms;
    rebuiltTerms.reserve(terms.size() - 2 * replacements + replacements);
    for (std::size_t i = 0; i < terms.size(); ++i)
        if (!removed[i])
            rebuiltTerms.push_back(terms[i]);
    for (std::size_t i = 0; i < replacements; ++i)
        rebuiltTerms.push_back(factory_.constant(1.0));

    return rebuildSum(rebuiltTerms, expression);
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

    memo_.emplace(expression, result);
    return result;
}

Expression simplifyTrigonometry(const Expression& expression, const ExpressionFactory& factory)
{
    TrigonometricSimplificationSession session{factory};
    return session.simplify(expression);
}

} // namespace kinemaforge::ik
