# F3.9 — spherical-wrist orientation solver — architecture

**State:** approved for implementation in this working branch  
**Date:** 2026-09-13

## Goal

After `q1..q3` are known, solve `q4..q6` without assuming that the URDF joint frames are aligned with world XYZ.

## Method

The solver forms the residual wrist orientation from product-of-exponentials geometry:

```text
R456 = R123^T * R_target * R_home^T
```

A basis built from the actual space axes maps the supported wrist to an XYX-equivalent decomposition. Both regular wrist branches are returned. Axis sign is derived from geometry instead of robot identity.

## Scope

The first implementation requires axis 4 parallel to axis 6 and axis 5 perpendicular to them. A different spherical-wrist arrangement is a future family, not a hidden fallback.
