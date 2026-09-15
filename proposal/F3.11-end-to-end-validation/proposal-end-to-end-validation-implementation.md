# F3.11 — analytic IK end-to-end validation — implementation

**State:** direct compilation + standalone property validation passed; full CTest pending external environment  
**Date:** 2026-09-13

Observed in this work session:

```text
KR640 analytic solver: 1000/1000 random generating configurations recovered
KR4 analytic solver:   1000/1000 random generating configurations recovered
returned regular branches: independently checked with quaternion FK
non-trivial TCP smoke: passed
wrist singular family: passed relation check
```

`IkEquationBuilder::solveAnalyticIk(PoseTarget)` exposes the solver through the facade and preserves the nested typed solver error.
