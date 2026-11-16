# Katalog dokumentów — KinemaForge / IkEquationBuilder

**Stan:** 2026-09-13  
**Źródło prawdy o kodzie:** [`../STATUS.md`](../STATUS.md)

Ten katalog rozdziela zamknięte etapy archiwalne od proposalów należących do bieżącego release candidate Phase 3. Proposale F3.2–F3.12 pozostają poza `done/` do czasu przejścia pełnego CMake/CTest gate'u na normalnym środowisku developerskim.

## Dokumenty żywe

| Dokument | Rola |
|---|---|
| [`../README.md`](../README.md) | publiczny opis projektu i aktualnego pipeline'u |
| [`../STATUS.md`](../STATUS.md) | co faktycznie istnieje, co uruchomiono, znane ograniczenia |
| [`KinemaForge_Phase2_Phase3_Roadmap.md`](KinemaForge_Phase2_Phase3_Roadmap.md) | historyczna roadmapa F2/F3 + aktualny aneks stanu |

## Zamknięte etapy

`proposal/done/` zawiera zatwierdzone i zakończone prace do F3.1 włącznie:

```text
Phase 1   URDF -> symbolic FK
F2.1-2    TCP transform
F2.3      equation/target model
F2.3.R    review remediation
F2.4      PositionOnly constraints
F2.5      FullPose constraints
F2.5Q     quaternion target
F2.6      directional constraints
F2.7      facade integration
F3.1      rewrite engine
```

## Phase 3 — working release candidate

| Etap | Dokumenty | Stan kodu 2026-09-13 |
|---|---|---|
| F3.2 | [`F3.2-canonical-form/`](F3.2-canonical-form/) | implemented; property-checked |
| F3.3 | [`F3.3-trigonometric-simplification/`](F3.3-trigonometric-simplification/) | implemented; intentionally small domain-safe rule set |
| F3.4 | [`F3.4-kinematic-geometry/`](F3.4-kinematic-geometry/) | implemented |
| F3.5 | [`F3.5-pattern-detector/`](F3.5-pattern-detector/) | implemented for first analytic family |
| F3.6 | [`F3.6-analytic-solution-model/`](F3.6-analytic-solution-model/) | implemented |
| F3.7 | [`F3.7-primitive-solvers/`](F3.7-primitive-solvers/) | implemented + smoke-tested |
| F3.8 | [`F3.8-position-solver/`](F3.8-position-solver/) | implemented; KR640/KR4 random property gate passed |
| F3.9 | [`F3.9-wrist-solver/`](F3.9-wrist-solver/) | implemented; regular + flip branches |
| F3.10 | [`F3.10-singularities-limits/`](F3.10-singularities-limits/) | joint limits + wrist singular family implemented; shoulder-axis family open |
| F3.11 | [`F3.11-end-to-end-validation/`](F3.11-end-to-end-validation/) | facade + standalone round-trip validation passed; full CTest pending |
| F3.12 | [`F3.12-code-generator/`](F3.12-code-generator/) | generated C++23 backend implemented, compiled and property-tested |
| F3.R2 | [`F3.R2-audit-followup/`](F3.R2-audit-followup/) | implemented 2026-11-15 on the user's go-ahead; 613/613 in Release, Debug and Werror |

## Release rule

Nie przenosić F3.2–F3.12 do `proposal/done/` wyłącznie na podstawie tego workspace. Najpierw:

1. clean configure/build w normalnym środowisku z dostępnym pugixml/googletest;
2. pełne `ctest --output-on-failure`;
3. naprawa ewentualnych integration-only problemów;
4. decyzja, czy shoulder-axis singularity jest wymagane do v1.0;
5. dopiero wtedy archiwizacja proposalów i tag release.
