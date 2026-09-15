# F3.4 — Kinematic geometry model — implementation

**State:** implemented in this working branch  
**Date:** 2026-09-13

## Files

```text
src/ik_equations/model/KinematicGeometry.hpp
src/ik_equations/builders/KinematicGeometryBuilder.hpp
src/ik_equations/builders/KinematicGeometryBuilder.cpp
tests/test_kinematic_geometry.cpp
```

## Implementation notes

The builder keeps a numeric rigid transform `T_base_parent` while walking the chain. For every joint it computes:

```text
R_base_joint = R_base_parent * R_origin
p_base_joint = p_base_parent + R_base_parent * p_origin
axis_base     = normalize(R_base_joint * axis_joint)
```

Then the zero child frame becomes `(R_base_joint, p_base_joint)`. This mirrors the fixed part of `JointTransformBuilder` without involving the symbolic layer.

Geometry predicates operate only on normalized directions. Non-parallel line distance uses the scalar triple product; parallel distance uses the cross product to avoid an unstable division by a tiny cross magnitude.
