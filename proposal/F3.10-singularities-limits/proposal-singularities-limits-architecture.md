# F3.10 — singularities, degeneracies and limits — architecture

**State:** partially complete for the first 6R family  
**Date:** 2026-09-13

## Joint limits

Regular rotary solutions are expanded by `2*pi` and intersected with each finite joint range; non-finite candidate values are rejected. Continuous joints use a principal representative.

## Wrist singularity

At the supported XYX wrist singularity, `q4` and `q6` are coupled. The solver must return an infinite family rather than silently fixing one joint. `SolutionBranch` therefore carries a free interval and the affine modulo-`2*pi` coupling relation.

## Known incomplete case

When the wrist center lies on the shoulder axis, `q1` becomes a free parameter and the remaining position subproblem depends on that choice. The current solver **does not fabricate a representative**. It reports `UnsupportedKinematicStructure` for that singular family. This is the principal F3.10 gap in the working branch.

The generated lightweight backend has the same limitation; its compact shoulder primitive currently maps the degenerate equation to no branch.
