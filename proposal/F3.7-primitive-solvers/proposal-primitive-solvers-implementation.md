# F3.7 — primitive solvers — implementation

**State:** implemented and smoke-tested in this working branch  
**Date:** 2026-09-13

## Files

```text
src/ik_equations/solver/PrimitiveSolvers.hpp
src/ik_equations/solver/PrimitiveSolvers.cpp
tests/test_primitive_solvers.cpp
```

The primitives are numeric helpers for the analytic family detector/solver. They return all branches, deduplicate modulo `2*pi`, and keep tolerances local to numerical geometry rather than symbolic equality.
