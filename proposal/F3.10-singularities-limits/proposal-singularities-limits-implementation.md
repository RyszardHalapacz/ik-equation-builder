# F3.10 — singularities, degeneracies and limits — implementation

**State:** wrist singularity + joint limits implemented; shoulder-axis family deferred  
**Date:** 2026-09-13

Regular solutions are expanded into every equivalent value inside declared limits. Wrist-aligned/anti-aligned cases produce `InfiniteFamily` branches containing a free `q4` interval and a coupled `q6` relation. Tests cover the relation and limit intersection.

The shoulder-axis singularity is deliberately fail-closed and is recorded as a release limitation rather than hidden by an arbitrary value.
