# F3.5 — IK pattern detector — implementation

**State:** implemented in this working branch  
**Date:** 2026-09-13

## Files

```text
src/ik_equations/model/IkPattern.hpp
src/ik_equations/solver/IkPatternDetector.hpp
src/ik_equations/solver/IkPatternDetector.cpp
tests/test_ik_pattern_detector.cpp
```

The detector consumes `KinematicGeometry`; it never reparses a URDF and never traverses symbolic expressions. The combined support pattern is intentionally stricter than `SphericalWrist` alone because it is a capability claim for the next solver stages.
