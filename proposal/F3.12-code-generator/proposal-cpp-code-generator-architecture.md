# F3.12 — C++ analytic IK code generator — architecture

**State:** approved for implementation in this working branch  
**Date:** 2026-09-13

## Goal

Emit a robot-specialized C++23 analytic solver after geometry/pattern detection so runtime no longer needs URDF parsing, symbolic expressions, or the general analytic solver object model.

## Boundary

The generator computes robot-specific numeric parameters once and emits a small header that depends only on `GeneratedIkRuntime.hpp`. The runtime contains the first supported 6R algorithm and compact value types; the generated file embeds constexpr geometry/limits and exposes `solve(Pose)`.

This is intentionally a **specialized lightweight runtime**, not a single gigantic source file and not a serialized symbolic DAG.

## Determinism

For the same chain, TCP and namespace, generated text is deterministic. Invalid namespace names and unsupported geometry return typed errors.

## Limitations

The generated backend supports the same regular family and wrist singularity as the general solver. The shoulder-axis infinite family is not yet represented.
