# F3.8 — position solver — implementation

**State:** implemented and property-tested in this working branch  
**Date:** 2026-09-13

## Files

```text
src/ik_equations/solver/AnalyticIkSolver.hpp
src/ik_equations/solver/AnalyticIkSolver.cpp
tests/test_analytic_ik_solver.cpp
```

The solver builds zero-configuration geometry, rejects unsupported chains, derives the wrist-center target, solves shoulder candidates with the sin/cos primitive, then solves the generalized 2R shoulder/elbow subproblem. Both shipped robots are handled through geometry; no KR4/KR640 link lengths or robot names are hard-coded.

Out-of-tree validation in this work session recovered a generating solution for 1000 random KR640 configurations and 1000 random KR4 configurations with zero failures; all returned regular branches were independently checked with quaternion FK.
