# F3.6 — analytic IK solution model — implementation

**State:** implemented in this working branch  
**Date:** 2026-09-13

## Files

```text
src/ik_equations/model/AnalyticIkSolution.hpp
```

The implementation is a data-only IR. It intentionally contains no solving logic. Regular solutions use `JointSolutionValue`; singular families use `FreeJointParameter` plus optional `CoupledFreeParameter`. The first solver stages consume this model directly.
