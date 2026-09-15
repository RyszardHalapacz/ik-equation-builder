# F3.8 — position solver for the first supported 6R family — architecture

**State:** approved for implementation in this working branch  
**Date:** 2026-09-13

## Supported family

The first solver is intentionally not `arbitrary URDF -> analytic IK`. It supports exactly six actuated rotary joints with:

1. joint axes 2 and 3 parallel;
2. joints 4–6 intersecting at a spherical wrist center;
3. the additional wrist structure required by F3.9 (axis 4 parallel to axis 6, axis 5 perpendicular).

Unsupported geometry must fail explicitly.

## Position decoupling

A full `PoseTarget` is required because a non-zero chain-tip/TCP offset must be removed using target orientation. At the home configuration the solver derives the wrist center relative to the constrained target frame, then reconstructs the target wrist center as:

```text
C_target = p_target + R_target * wrist_in_target_frame
```

The shoulder equation is obtained from the actual base-frame axis geometry. For every shoulder candidate the target is inverse-rotated around joint 1 and joints 2/3 are solved as a generalized planar 2R problem using measured axis/link geometry rather than robot-name constants.

## Verification policy

Every regular candidate is substituted through an independent numeric FK path before it is exposed. Candidate generation is therefore fail-closed: a branch that does not reproduce the target is discarded.
