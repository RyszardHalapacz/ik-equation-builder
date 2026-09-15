# F3.12 — C++ analytic IK code generator — implementation

**State:** implemented, generated headers compiled and property-tested in this working branch  
**Date:** 2026-09-13

## Files

```text
src/ik_equations/codegen/CppAnalyticIkGenerator.hpp
src/ik_equations/codegen/CppAnalyticIkGenerator.cpp
src/ik_equations/codegen/GeneratedIkRuntime.hpp
tests/test_code_generator.cpp
tests/generated/kr640_analytic_ik.hpp
tests/generated/kr4_analytic_ik.hpp
```

Observed standalone validation:

```text
KR640 generated backend: 1000/1000 random targets recovered
KR4 generated backend:   1000/1000 random targets recovered
KR640 generated + non-trivial TCP: 500/500 recovered
```

Both generated fixture headers were actually compiled. The emitted text contains robot constants and the lightweight runtime include; it does not reference URDF, `Expression`, or `AnalyticIkSolver`.
