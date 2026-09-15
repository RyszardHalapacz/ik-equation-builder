#pragma once

#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"
#include "ik_equations/symbolic/ExpressionIdentityMap.hpp"
#include "ik_equations/symbolic/RewriteRule.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace kinemaforge::ik {

// The syntactic domain guards of an expression: the divisor of every Divide
// node, deduplicated STRUCTURALLY. Complete for the nine node kinds that exist
// today -- a new partial operator (sqrt, log, asin) would need a richer model,
// not one more entry here.
//
// Structural deduplication, not by identity, because hasSameDomainGuards()
// compares sizes and then inclusion; that is a valid set comparison only when
// neither side holds two equivalent entries.
[[nodiscard]] std::vector<Expression> divisorsOf(const Expression& expression);

// Compares those guards AS SETS -- order-independent, because divisorsOf walks
// the tree and two guards may come out in either order. A plain vector ==
// vector would reject a rewrite for reordering its own output.
//
// Deliberately NOT called preservesDomain: it decides a syntactic question, and
// only LOCALLY -- at the point a rule fires, between the rule's input and its
// output. Applying it to whole trees before and after a rewrite is wrong and
// would reject correct rewrites (a/-(-q) and a/q have different divisors and
// the same domain); the whole-tree property follows by induction instead.
//
// These two are the API and the tests' tool, not the path the engine walks:
// called directly they traverse the DAG from scratch. RewriteSession uses the
// cached form, over the SAME implementation -- two independent domain analysers
// is exactly the shape in which a safety condition quietly stops meaning what
// its test says.
[[nodiscard]] bool hasSameDomainGuards(const Expression& before, const Expression& after);

// Rewrites expressions to a fixed point under one fixed, ordered set of rules.
//
// A SESSION, not a free function, for the same reason ExpressionEvaluator is
// one: the sixteen cells of a transform and the two sides of twelve equations
// are many roots of ONE shared DAG, and the caches work between them.
//
// RULE ORDER IS SEMANTICS. The session preserves the order given to the
// constructor and the first admissible rule wins. The engine promises neither
// confluence nor a unique normal form -- two correct rules may lead to two
// different fixed points, and which one is reached depends on their order.
// Choosing the set and its order is the caller's job.
//
// FIXED POINT means a fixed point of the complete rewrite policy: the rules in
// their declared order, the local domain condition, and the contextual
// representability guard. It does NOT mean that no descendant would be
// rewriteable in isolation. a/(q-q) is a fixed point of this engine even with
// `x - x -> 0` in the set, because the guard refuses to put a literal zero in a
// denominator -- the subexpression matches, the policy still declines.
//
// The caches grow monotonically and one session is one rule set; a different
// set needs a new session, which is cheap.
//
// Not thread-safe: the caches are mutable. The DAG itself is immutable, so
// several sessions may read the same tree concurrently.
class RewriteSession
{
public:
    // Takes a span and COPIES it: the session owns its rules.
    RewriteSession(std::span<const RewriteRule> rules, ExpressionFactory factory,
                   std::size_t stepLimit = kDefaultRewriteStepLimit);

    // A session is not a value, for the same reason ExpressionEvaluator is not:
    // copying would duplicate the caches with no clear meaning.
    RewriteSession(const RewriteSession&) = delete;
    RewriteSession& operator=(const RewriteSession&) = delete;

    // No explicit noexcept: unordered_map's move constructor is only
    // conditionally noexcept, so the compiler-derived specifier states the
    // truth instead of turning a throwing move into std::terminate.
    RewriteSession(RewriteSession&&) = default;
    RewriteSession& operator=(RewriteSession&&) = default;

    [[nodiscard]] RewriteResult rewrite(const Expression& expression);

private:
    using Memo = detail::ExpressionMap<Expression>;
    using Guards = detail::ExpressionMap<std::vector<Expression>>;

    // Everything one rewrite() call owns. New cache entries land here and are
    // committed to the session only if the call reaches a fixed point, so a
    // call stopped by the step limit leaves nothing behind -- neither an
    // unfinished result the next root could pick up, nor the temporary DAGs a
    // guard entry would keep alive through its key.
    struct Call
    {
        Memo memo;
        Guards guards;

        // Node measures for the rule-contract assert. Per CALL, not per session:
        // measureOf() is asked about every candidate replacement, INCLUDING the
        // ones the domain check then rejects, and a session-lifetime cache would
        // keep those throwaway DAGs alive through their keys for no benefit. The
        // memo and the guards earn their session lifetime; a diagnostic counter
        // does not.
        //
        // Present in both configurations rather than behind #ifndef NDEBUG: a
        // default-constructed unordered_map allocates nothing, and a member that
        // changes the layout between Release and diagnostic builds costs more in
        // surprise than it saves in bytes.
        detail::ExpressionMap<std::size_t> measures;

        std::size_t steps{};
        bool reachedLimit{};
    };

    Expression rewriteNode(const Expression& expression, Call& call);
    Expression rewriteChildren(const Expression& expression, Call& call);
    std::optional<Expression> firstAdmissibleRule(const Expression& expression, Call& call);

    const Expression* lookup(const Expression& expression, const Call& call) const;
    const std::vector<Expression>& domainGuards(const Expression& expression, Call& call);

    std::vector<RewriteRule> rules_;
    ExpressionFactory factory_;
    std::size_t stepLimit_;

    Memo memo_;      // proven fixed points only
    Guards guards_;  // domain guards, committed with the memo
};

} // namespace kinemaforge::ik
