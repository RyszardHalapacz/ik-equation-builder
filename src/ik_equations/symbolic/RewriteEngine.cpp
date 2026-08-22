#include "ik_equations/symbolic/RewriteEngine.hpp"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <type_traits>
#include <utility>
#include <variant>

namespace kinemaforge::ik {

namespace {

using GuardCache = detail::ExpressionMap<std::vector<Expression>>;

// The default branch of an exhaustive if-constexpr chain needs a condition that
// is dependent on the template parameter; a plain `static_assert(false)` fires
// while merely parsing the template.
template <typename>
inline constexpr bool alwaysFalse = false;

// Structural, not by identity: `a/x + b/x` built from two separately
// constructed `x` nodes must yield ONE guard, or the size-plus-inclusion test
// in sameGuards() stops being a set comparison.
void appendUnique(std::vector<Expression>& guards, const Expression& candidate)
{
    const auto equivalent = [&candidate](const Expression& present) {
        return structurallyEqual(present, candidate);
    };
    if (std::none_of(guards.begin(), guards.end(), equivalent))
        guards.push_back(candidate);
}

void appendAll(std::vector<Expression>& guards, const std::vector<Expression>& more)
{
    for (const Expression& guard : more)
        appendUnique(guards, guard);
}

// THE domain analyser. There is exactly one, and both the free functions and
// RewriteSession go through it.
//
// Computed compositionally rather than by scanning from scratch:
//
//     guards(Constant | Symbol) = {}
//     guards(op(a, b))          = guards(a) u guards(b)
//     guards(op(a))             = guards(a)
//     guards(Divide(a, b))      = guards(a) u guards(b) u { b }
//
// so each unique node is computed once. That bounds the NUMBER of computations,
// not the total cost: each union deduplicates structurally, so the cost of one
// entry grows with the number of DISTINCT guards below it. For everything this
// project builds today that number is zero -- no builder calls
// ExpressionFactory::divide, so no FK DAG contains a single Divide node.
//
// `committed` is the session-wide cache when a session calls this and nullptr
// when a free function does; `pending` always receives the new entries.
//
// Returns a reference into either `pending` or `committed`. std::unordered_map
// keeps references to its elements valid across rehashing -- only iterators are
// invalidated -- so the recursive calls below cannot dangle what this returns,
// and `committed` is not written to at all while a call is in flight.
const std::vector<Expression>& collectGuards(const Expression& expression,
                                             GuardCache& pending,
                                             const GuardCache* committed)
{
    if (const auto found = pending.find(expression); found != pending.end())
        return found->second;

    if (committed != nullptr)
        if (const auto found = committed->find(expression); found != committed->end())
            return found->second;

    std::vector<Expression> guards;

    // EVERY node kind spelled out, on purpose. A generic
    // `else if constexpr (requires { node.operand; })` would silently absorb a
    // future SqrtNode and report guards(sqrt(x)) = guards(x), losing x >= 0 --
    // and it would compile. Here a tenth node kind lands in the default branch
    // and STOPS THE BUILD with a message naming the decision that has to be
    // made. This is a safety boundary; explicit duplication is the feature.
    std::visit(
        [&](const auto& node) {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, ConstantNode> ||
                          std::is_same_v<Node, SymbolNode>)
            {
                // No children, no guards.
            }
            else if constexpr (std::is_same_v<Node, AddNode> ||
                               std::is_same_v<Node, SubtractNode> ||
                               std::is_same_v<Node, MultiplyNode>)
            {
                appendAll(guards, collectGuards(node.lhs, pending, committed));
                appendAll(guards, collectGuards(node.rhs, pending, committed));
            }
            else if constexpr (std::is_same_v<Node, DivideNode>)
            {
                appendAll(guards, collectGuards(node.lhs, pending, committed));
                appendAll(guards, collectGuards(node.rhs, pending, committed));

                // The only place in the language that restricts the domain.
                appendUnique(guards, node.rhs);
            }
            else if constexpr (std::is_same_v<Node, NegateNode> ||
                               std::is_same_v<Node, SinNode> ||
                               std::is_same_v<Node, CosNode>)
            {
                appendAll(guards, collectGuards(node.operand, pending, committed));
            }
            else
            {
                static_assert(alwaysFalse<Node>,
                              "unhandled Expression node in domain analysis: decide whether "
                              "it restricts the domain before adding it to the language");
            }
        },
        expression.node().value);

    return pending.emplace(expression, std::move(guards)).first->second;
}

// Set equality. Both sides are structurally deduplicated by construction, so
// equal sizes plus one-way inclusion is enough.
bool sameGuards(const std::vector<Expression>& before, const std::vector<Expression>& after)
{
    if (before.size() != after.size())
        return false;

    return std::all_of(before.begin(), before.end(), [&after](const Expression& guard) {
        return std::any_of(after.begin(), after.end(), [&guard](const Expression& other) {
            return structurallyEqual(guard, other);
        });
    });
}

} // namespace

std::vector<Expression> divisorsOf(const Expression& expression)
{
    GuardCache cache;
    return collectGuards(expression, cache, nullptr);
}

bool hasSameDomainGuards(const Expression& before, const Expression& after)
{
    GuardCache cache;

    // Both references stay valid while the second call inserts -- see the note
    // on reference stability in collectGuards.
    const std::vector<Expression>& beforeGuards = collectGuards(before, cache, nullptr);
    const std::vector<Expression>& afterGuards = collectGuards(after, cache, nullptr);

    return sameGuards(beforeGuards, afterGuards);
}

namespace {

#ifndef NDEBUG

// Node count WITH MULTIPLICITY, memoised by node identity. Memoising the count
// does not turn it into a count of unique nodes: the parent adds a shared
// child's total once per slot it occupies, which is exactly what "with
// multiplicity" means.
std::size_t measureOf(const Expression& expression, detail::ExpressionMap<std::size_t>& cache)
{
    if (const auto found = cache.find(expression); found != cache.end())
        return found->second;

    std::size_t total = 1;

    std::visit(
        [&](const auto& node) {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, ConstantNode> || std::is_same_v<Node, SymbolNode>)
            {
            }
            else if constexpr (requires { node.lhs; node.rhs; })
            {
                total += measureOf(node.lhs, cache);
                total += measureOf(node.rhs, cache);
            }
            else
            {
                total += measureOf(node.operand, cache);
            }
        },
        expression.node().value);

    cache.emplace(expression, total);
    return total;
}

// Exists only to be called from an assert(), like isActuated in
// JointTransformBuilder -- and with the same caveat stated plainly: a build
// with NDEBUG defined enforces NEITHER of these contracts.
//
// A plain assert() cannot name the offending rule, and "some rule broke the
// measure contract" is a diagnosis nobody can act on when the set has a dozen
// entries. Hence the message on stderr before returning false.
[[maybe_unused]] bool ruleContractHolds(const RewriteRule& rule,
                                        const Expression& input,
                                        const Expression& replacement,
                                        detail::ExpressionMap<std::size_t>& measures)
{
    if (sameNode(replacement, input))
    {
        std::fprintf(stderr,
                     "rewrite rule '%s' returned its input; nullopt means "
                     "\"does not apply\"\n",
                     rule.name.c_str());
        return false;
    }

    const std::size_t before = measureOf(input, measures);
    const std::size_t after = measureOf(replacement, measures);

    if (after >= before)
    {
        std::fprintf(stderr,
                     "rewrite rule '%s' did not decrease the measure: %zu -> %zu\n",
                     rule.name.c_str(), before, after);
        return false;
    }

    return true;
}

#endif // NDEBUG

} // namespace

RewriteSession::RewriteSession(std::span<const RewriteRule> rules, ExpressionFactory factory,
                               std::size_t stepLimit)
    : rules_(rules.begin(), rules.end())
    , factory_(std::move(factory))
    , stepLimit_(stepLimit)
{
}

RewriteResult RewriteSession::rewrite(const Expression& expression)
{
    Call call;
    Expression result = rewriteNode(expression, call);

    // Commit, or discard everything this call built. Both caches, one moment.
    // merge() moves the nodes across; a key already present in the session keeps
    // the session's value, which by the determinism contract says the same thing.
    if (!call.reachedLimit)
    {
        memo_.merge(call.memo);
        guards_.merge(call.guards);
    }

    // `changed` is computed BEFORE the move: elements of a braced-init-list are
    // evaluated in order, so writing sameNode(expression, result) as the second
    // element would read a moved-from Expression.
    const bool changed = !sameNode(expression, result);
    return RewriteResult{std::move(result), changed, call.steps, call.reachedLimit};
}

const Expression* RewriteSession::lookup(const Expression& expression, const Call& call) const
{
    // Call-local first: both tiers hold fixed points, so the order is a
    // performance detail, and the local map is the smaller one.
    if (const auto found = call.memo.find(expression); found != call.memo.end())
        return &found->second;

    if (const auto found = memo_.find(expression); found != memo_.end())
        return &found->second;

    return nullptr;
}

const std::vector<Expression>& RewriteSession::domainGuards(const Expression& expression,
                                                           Call& call)
{
    return collectGuards(expression, call.guards, &guards_);
}

Expression RewriteSession::rewriteNode(const Expression& expression, Call& call)
{
    // THE single point of step-limit propagation. Once the budget is gone every
    // deeper call returns its input untouched, so the recursion unwinds by
    // rebuilding the tree from whatever was already normalised: the result is a
    // valid, partially rewritten expression and no caller needs a special case.
    // Nothing is memoised on this path -- the commit in rewrite() is skipped for
    // the whole call, and the emplace below is never reached.
    if (call.reachedLimit)
        return expression;

    if (const Expression* cached = lookup(expression, call))
        return *cached;

    Expression current = expression;

    while (true)
    {
        // Back to the CHILDREN after every firing, not just to the rule attempt:
        // a rule receives the factory and may build a subtree that was never in
        // the input, and calling that a fixed point without normalising it would
        // be a lie. For children already at their fixed point this costs one
        // memo hit.
        Expression rebuilt = rewriteChildren(current, call);
        if (call.reachedLimit)
            return rebuilt;

        const std::optional<Expression> replacement = firstAdmissibleRule(rebuilt, call);
        if (!replacement)
        {
            current = std::move(rebuilt);
            break;
        }

        if (call.steps >= stepLimit_)
        {
            // Admissible, but there is no budget to take it. The last form that
            // was actually built is returned; the replacement is dropped.
            call.reachedLimit = true;
            return rebuilt;
        }

        ++call.steps;
        current = *replacement;
    }

    // Written AFTER the loop, keyed by this call's INPUT and holding its fixed
    // point. The intermediate values of `current` are states in progress, not
    // results, and never reach the cache -- which is what makes "every
    // committed entry is a fixed point" true by construction rather than by
    // one condition somewhere inside the loop.
    call.memo.emplace(expression, current);
    return current;
}

Expression RewriteSession::rewriteChildren(const Expression& expression, Call& call)
{
    return std::visit(
        [&](const auto& node) -> Expression {
            using Node = std::decay_t<decltype(node)>;

            if constexpr (std::is_same_v<Node, ConstantNode> || std::is_same_v<Node, SymbolNode>)
            {
                return expression;
            }
            else if constexpr (requires { node.lhs; node.rhs; })
            {
                Expression lhs = rewriteNode(node.lhs, call);
                Expression rhs = rewriteNode(node.rhs, call);

                if constexpr (std::is_same_v<Node, DivideNode>)
                {
                    // Representability guard. A denominator that simplified to a
                    // literal zero cannot be built -- ExpressionFactory::divide
                    // asserts against it -- and that contract is NOT being
                    // loosened for this engine: the simplification is dropped and
                    // the original denominator kept. The node stays exactly as
                    // undefined as it was, which is the honest outcome.
                    if (isZero(rhs))
                        rhs = node.rhs;
                }

                // Identity preservation: if nothing actually entered the parent,
                // return the ORIGINAL node. Rebuilding a structurally identical
                // node would make `changed` always true and would break the
                // sharing this engine exists to preserve.
                if (sameNode(lhs, node.lhs) && sameNode(rhs, node.rhs))
                    return expression;

                if constexpr (std::is_same_v<Node, AddNode>)
                    return factory_.add(std::move(lhs), std::move(rhs));
                else if constexpr (std::is_same_v<Node, SubtractNode>)
                    return factory_.subtract(std::move(lhs), std::move(rhs));
                else if constexpr (std::is_same_v<Node, MultiplyNode>)
                    return factory_.multiply(std::move(lhs), std::move(rhs));
                else
                {
                    static_assert(std::is_same_v<Node, DivideNode>);
                    return factory_.divide(std::move(lhs), std::move(rhs));
                }
            }
            else
            {
                Expression operand = rewriteNode(node.operand, call);

                if (sameNode(operand, node.operand))
                    return expression;

                if constexpr (std::is_same_v<Node, NegateNode>)
                    return factory_.negate(std::move(operand));
                else if constexpr (std::is_same_v<Node, SinNode>)
                    return factory_.sin(std::move(operand));
                else
                {
                    static_assert(std::is_same_v<Node, CosNode>);
                    return factory_.cos(std::move(operand));
                }
            }
        },
        expression.node().value);
}

std::optional<Expression> RewriteSession::firstAdmissibleRule(const Expression& expression,
                                                             Call& call)
{
    // Declaration order is semantics: the first admissible rule wins. A rule
    // rejected by the domain condition does not abort the pass and does not
    // block the rules after it.
    for (const RewriteRule& rule : rules_)
    {
        std::optional<Expression> replacement = rule.apply(expression, factory_);
        if (!replacement)
            continue;

        assert(ruleContractHolds(rule, expression, *replacement, call.measures) &&
               "rewrite rule broke its contract");

        // The domain condition, and it is LOCAL: between this rule's input and
        // its output, never between whole trees.
        const std::vector<Expression>& before = domainGuards(expression, call);
        const std::vector<Expression>& after = domainGuards(*replacement, call);

        if (!sameGuards(before, after))
            continue;

        return replacement;
    }

    return std::nullopt;
}

} // namespace kinemaforge::ik
