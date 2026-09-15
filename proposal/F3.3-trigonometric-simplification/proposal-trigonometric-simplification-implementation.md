# Proposal: F3.3 — trigonometric simplification implementation

**Stage:** F3.3 · **implementation v1 — approved for implementation in this work session** · 2026-09-13

## Files

```text
NEW     src/ik_equations/symbolic/TrigonometricSimplifier.hpp
        src/ik_equations/symbolic/TrigonometricSimplifier.cpp
        tests/test_trigonometric_simplifier.cpp
CHANGE  src/ik_equations/builders/EquationSimplifier.hpp/.cpp
        CMakeLists.txt
        tests/CMakeLists.txt
```

## Public surface

```cpp
struct TrigonometricRules
{
    static RewriteRule cosineEven();
};

class TrigonometricSimplificationSession
{
public:
    explicit TrigonometricSimplificationSession(ExpressionFactory factory);
    Expression simplify(const Expression& expression);
};
```

The implementation owns one `CanonicalizationSession` and one identity memo. It recursively
simplifies children, applies odd/even sign normalisation, flattens canonical `Add` roots,
finds Pythagorean square pairs with structurally equal arguments, checks the argument has no
divisor guards, rebuilds, then canonicalises again.

`EquationSimplifier::simplify` uses one trig session for the whole `IkEquationSystem`, copies
`Equation`, replaces only `lhs` and `rhs`, and recreates the system through
`IkEquationSystem::create` exactly like `normalize`.

## Tests

At minimum:

- `NormalizesSineOfNegativeArgument`
- `NormalizesCosineOfNegativeArgument`
- `CosineEvenRuleRunsThroughRewriteEngine`
- `ReducesPythagoreanPair`
- `ReducesPairInsideLargerSum`
- `MatchesCommonArgumentStructurally`
- `DoesNotReduceDifferentArguments`
- `DoesNotReducePartialArgument`
- `PreservesDivisionByZeroForPartialArgument`
- `IsIdempotent`
- `KeepsNodeIdentityInFreshSession`
- `PreservesSharingAcrossEquationSystem`
- numeric randomized equivalence

