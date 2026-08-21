# Katalog dokumentacji KinemaForge

**Wygenerowany:** 2026-08-21 · **dokumentów:** 46 · **łącznie:** ~27 900 linii, 1,4 MB

Spis wszystkiego, co w tym repo jest dokumentem, a nie kodem. Proces jest stały:
**proposal architektoniczny → review → proposal implementacyjny → review →
implementacja → clean build → pełne testy → aktualizacja STATUS.md → przeniesienie
do `done/`**. Kod żyje w `.md` do momentu zatwierdzenia, więc te pliki są zapisem
decyzji, a nie streszczeniem kodu — w wielu miejscach zawierają uzasadnienia
wariantów odrzuconych, których w kodzie już nie widać.

Stany: **ZAIMPLEMENTOWANE** — kod w repo · **APPROVED** — zatwierdzone, część
pary z implementacją · **DO REVIEW** — czeka na werdykt · **żywy** — dokument
aktualizowany na bieżąco, bez wersji.

Ścieżki są względne wobec tego pliku, czyli wobec `proposal/`.

## Dokumenty żywe (aktualizowane na bieżąco)

| Dokument | Temat | Stan | Linii |
|---|---|---|---|
| [`../README.md`](../README.md) | Opis pipeline'u i wizji projektu | żywy | 202 |
| [`../STATUS.md`](../STATUS.md) | Co naprawdę istnieje, znane braki, następny krok | żywy | 732 |
| [`../COMMANDS.md`](../COMMANDS.md) | Komendy build/test, nota o MinGW i PATH | żywy | 67 |
| [`KinemaForge_Phase2_Phase3_Roadmap.md`](KinemaForge_Phase2_Phase3_Roadmap.md) | Roadmapa Phase 2/3: etapy F2.1–F2.7, F2.3.R i F2.5Q | żywy | 1278 |
| [`done/README.md`](done/README.md) | Indeks archiwum zamkniętych etapów | żywy | 45 |
| [`KATALOG.md`](KATALOG.md) | Ten spis | żywy | — |

## Etap bieżący — F2.7, integracja z fasadą

Ostatni etap Fazy 2.

| Dokument | Temat | Stan | Linii |
|---|---|---|---|
| [`F2.7-facade-integration/proposal-phase2-facade-integration.md`](F2.7-facade-integration/proposal-phase2-facade-integration.md) | Cel jako argument zamiast stanu, wybór transformacji, DoD Fazy 2 | **DO REVIEW (v1)** | 475 |

## Analizy poprzedzające proposale

| Dokument | Temat | Stan | Linii |
|---|---|---|---|
| [`analysis-ik-pipeline-constraint-builder.md`](analysis-ik-pipeline-constraint-builder.md) | Gdzie w pipelinie żyją ograniczenia zadaniowe | analiza — pytanie 4 wciąż otwarte, zasila F2.6 | 116 |

## Faza 1 — URDF → symboliczne FK (zamknięta)

Folder: [`done/phase1-urdf-to-symbolic-fk/`](done/phase1-urdf-to-symbolic-fk/)

| Dokument | Temat | Stan | Linii |
|---|---|---|---|
| [`proposal-loader-test-coverage.md`](done/phase1-urdf-to-symbolic-fk/proposal-loader-test-coverage.md) | Rozszerzenie testów loadera o KR4/KR640 | ZAIMPLEMENTOWANE | 340 |
| [`proposal-urdf-geometry-validation-architecture.md`](done/phase1-urdf-to-symbolic-fk/proposal-urdf-geometry-validation-architecture.md) | Walidacja geometrii URDF — projekt | APPROVED | 746 |
| [`proposal-urdf-geometry-validation-implementation.md`](done/phase1-urdf-to-symbolic-fk/proposal-urdf-geometry-validation-implementation.md) | Walidacja geometrii URDF — kod | ZAIMPLEMENTOWANE | 1295 |
| [`proposal-kinematic-chain-builder-architecture.md`](done/phase1-urdf-to-symbolic-fk/proposal-kinematic-chain-builder-architecture.md) | KinematicChainBuilder — projekt | APPROVED | 275 |
| [`proposal-kinematic-chain-builder-implementation.md`](done/phase1-urdf-to-symbolic-fk/proposal-kinematic-chain-builder-implementation.md) | KinematicChainBuilder — kod | ZAIMPLEMENTOWANE | 759 |
| [`proposal-symbolic-layer-architecture.md`](done/phase1-urdf-to-symbolic-fk/proposal-symbolic-layer-architecture.md) | Expression, ExpressionFactory, SymbolicMatrix — projekt | APPROVED | 1094 |
| [`proposal-symbolic-layer-implementation.md`](done/phase1-urdf-to-symbolic-fk/proposal-symbolic-layer-implementation.md) | Warstwa symboliczna — kod | ZAIMPLEMENTOWANE | 1485 |
| [`proposal-joint-transform-builder-architecture.md`](done/phase1-urdf-to-symbolic-fk/proposal-joint-transform-builder-architecture.md) | JointTransformBuilder — projekt | APPROVED | 816 |
| [`proposal-joint-transform-builder-implementation.md`](done/phase1-urdf-to-symbolic-fk/proposal-joint-transform-builder-implementation.md) | JointTransformBuilder — kod | ZAIMPLEMENTOWANE | 1068 |
| [`proposal-forward-kinematics-builder-architecture.md`](done/phase1-urdf-to-symbolic-fk/proposal-forward-kinematics-builder-architecture.md) | Kompozycja FK — projekt | APPROVED (v2) | 528 |
| [`proposal-forward-kinematics-builder-implementation.md`](done/phase1-urdf-to-symbolic-fk/proposal-forward-kinematics-builder-implementation.md) | Kompozycja FK — kod | ZAIMPLEMENTOWANE | 1188 |
| [`proposal-expression-evaluator-architecture.md`](done/phase1-urdf-to-symbolic-fk/proposal-expression-evaluator-architecture.md) | ExpressionEvaluator — projekt | APPROVED (v2) | 432 |
| [`proposal-expression-evaluator-implementation.md`](done/phase1-urdf-to-symbolic-fk/proposal-expression-evaluator-implementation.md) | ExpressionEvaluator — kod | ZAIMPLEMENTOWANE | 937 |
| [`proposal-numeric-fk-validation-architecture.md`](done/phase1-urdf-to-symbolic-fk/proposal-numeric-fk-validation-architecture.md) | Niezależna kontrola FK kwaternionami — projekt | APPROVED (v2) | 367 |
| [`proposal-numeric-fk-validation-implementation.md`](done/phase1-urdf-to-symbolic-fk/proposal-numeric-fk-validation-implementation.md) | Referencja kwaternionowa + 17 testów | ZAIMPLEMENTOWANE | 1057 |
| [`proposal-ik-equation-builder-architecture.md`](done/phase1-urdf-to-symbolic-fk/proposal-ik-equation-builder-architecture.md) | Fasada IkEquationBuilder — projekt | APPROVED | 239 |
| [`proposal-ik-equation-builder-implementation.md`](done/phase1-urdf-to-symbolic-fk/proposal-ik-equation-builder-implementation.md) | Fasada + 17 testów — kod | ZAIMPLEMENTOWANE | 731 |

## F2.1 / F2.2 — stała transformacja TCP (zamknięte)

Folder: [`done/F2.1-F2.2-tcp-transform/`](done/F2.1-F2.2-tcp-transform/)

| Dokument | Temat | Stan | Linii |
|---|---|---|---|
| [`proposal-tcp-transform-architecture.md`](done/F2.1-F2.2-tcp-transform/proposal-tcp-transform-architecture.md) | Stała transformacja tip→TCP — projekt | APPROVED (v3) | 609 |
| [`proposal-tcp-transform-implementation.md`](done/F2.1-F2.2-tcp-transform/proposal-tcp-transform-implementation.md) | TCP + 21 testów — kod | ZAIMPLEMENTOWANE | 1391 |

## F2.3 — model równań i celów (zamknięte)

Folder: [`done/F2.3-ik-equation-model/`](done/F2.3-ik-equation-model/)

| Dokument | Temat | Stan | Linii |
|---|---|---|---|
| [`proposal-ik-equation-model-architecture.md`](done/F2.3-ik-equation-model/proposal-ik-equation-model-architecture.md) | Equation, IkTarget, IkEquationSystem — projekt | APPROVED (v3) | 920 |
| [`proposal-ik-equation-model-implementation.md`](done/F2.3-ik-equation-model/proposal-ik-equation-model-implementation.md) | Model równań + 31 testów — kod | ZAIMPLEMENTOWANE | 1593 |

## F2.3.R — remediacja po review kodu z 2026-08-17 (zamknięte 2026-08-18)

Folder: [`done/F2.3.R-review-remediation/`](done/F2.3.R-review-remediation/README.md)

| Dokument | Temat | Stan | Linii |
|---|---|---|---|
| [`README.md`](done/F2.3.R-review-remediation/README.md) | Plan etapu: zakres, kolejność, gate | indeks etapu | 75 |
| [`proposal-warning-policy.md`](done/F2.3.R-review-remediation/proposal-warning-policy.md) | R1 — polityka ostrzeżeń, `-Werror` w CI | ZAIMPLEMENTOWANE | 678 |
| [`proposal-loader-topology-architecture.md`](done/F2.3.R-review-remediation/proposal-loader-topology-architecture.md) | R2 — topologia loadera, dwa błędy HIGH | ZAIMPLEMENTOWANE | 654 |
| [`proposal-loader-limits-strict-parsing-architecture.md`](done/F2.3.R-review-remediation/proposal-loader-limits-strict-parsing-architecture.md) | R3 — ścisły profil `<limit>` — projekt | APPROVED | 569 |
| [`proposal-loader-limits-strict-parsing-implementation.md`](done/F2.3.R-review-remediation/proposal-loader-limits-strict-parsing-implementation.md) | R3 — kod | ZAIMPLEMENTOWANE | 690 |
| [`proposal-legacy-robot-model-boundary.md`](done/F2.3.R-review-remediation/proposal-legacy-robot-model-boundary.md) | R4 — granica starej warstwy `mt::kinematics` | ZAIMPLEMENTOWANE | 412 |
| [`proposal-expression-factory-finite-invariant-architecture.md`](done/F2.3.R-review-remediation/proposal-expression-factory-finite-invariant-architecture.md) | R5 — inwariant skończoności w ExpressionFactory — projekt | APPROVED (v2) | 614 |
| [`proposal-expression-factory-finite-invariant-implementation.md`](done/F2.3.R-review-remediation/proposal-expression-factory-finite-invariant-implementation.md) | R5 — kod | ZAIMPLEMENTOWANE | 658 |

## F2.4 — ConstraintBuilder: PositionOnly (zamknięte)

Folder: [`done/F2.4-constraint-builder-position/`](done/F2.4-constraint-builder-position/)

| Dokument | Temat | Stan | Linii |
|---|---|---|---|
| [`proposal-constraint-builder-architecture.md`](done/F2.4-constraint-builder-position/proposal-constraint-builder-architecture.md) | ConstraintBuilder PositionOnly — projekt | APPROVED (v2) | 506 |
| [`proposal-constraint-builder-implementation.md`](done/F2.4-constraint-builder-position/proposal-constraint-builder-implementation.md) | ConstraintBuilder — kod + 12 testów | ZAIMPLEMENTOWANE | 790 |

## F2.6 — constrainty kierunkowe (zamknięte 2026-08-21)

Folder: [`done/F2.6-directional-constraints/`](done/F2.6-directional-constraints/)

| Dokument | Temat | Stan | Linii |
|---|---|---|---|
| [`proposal-directional-constraints-architecture.md`](done/F2.6-directional-constraints/proposal-directional-constraints-architecture.md) | Pięć nazw z roadmapy to dwie rodziny równań; osie główne, DOF, przebudowa reguł IkEquationSystem | APPROVED (v4) | 1098 |
| [`proposal-directional-constraints-a-implementation.md`](done/F2.6-directional-constraints/proposal-directional-constraints-a-implementation.md) | F2.6a: rodzina kierunku, kod + 31 testów | ZAIMPLEMENTOWANE | 1272 |
| [`proposal-directional-constraints-b-implementation.md`](done/F2.6-directional-constraints/proposal-directional-constraints-b-implementation.md) | F2.6b: rodzina kąta, pierwsze liczone lhs, kod + 14 testów | ZAIMPLEMENTOWANE | 763 |

## F2.5Q — target kwaternionowy (zamknięte 2026-08-21)

Folder: [`done/F2.5Q-quaternion-target/`](done/F2.5Q-quaternion-target/)

| Dokument | Temat | Stan | Linii |
|---|---|---|---|
| [`proposal-quaternion-target-architecture.md`](done/F2.5Q-quaternion-target/proposal-quaternion-target-architecture.md) | Gdzie wchodzi kwaternion, własna tolerancja normy, granice tego, co da się sprawdzić | APPROVED (v2) | 857 |
| [`proposal-quaternion-target-implementation.md`](done/F2.5Q-quaternion-target/proposal-quaternion-target-implementation.md) | Kod + 21 testów | ZAIMPLEMENTOWANE | 1057 |

## F2.5 — ConstraintBuilder: FullPose (zamknięte)

Folder: [`done/F2.5-constraint-builder-fullpose/`](done/F2.5-constraint-builder-fullpose/)

| Dokument | Temat | Stan | Linii |
|---|---|---|---|
| [`proposal-constraint-builder-fullpose-architecture.md`](done/F2.5-constraint-builder-fullpose/proposal-constraint-builder-fullpose-architecture.md) | FullPose — projekt (9 równań rotacji) | APPROVED (v2) | 401 |
| [`proposal-constraint-builder-fullpose-implementation.md`](done/F2.5-constraint-builder-fullpose/proposal-constraint-builder-fullpose-implementation.md) | FullPose — kod + 12 testów | ZAIMPLEMENTOWANE | 747 |

## Mapa: uwaga z review → dokument

Review z 2026-08-17 zgłosiło 10 uwag, wszystkie odtworzone eksperymentalnie
przed przyjęciem. Wszystkie dziesięć jest dziś zamkniętych.

| Uwaga | Waga | Dokument | Stan |
|---|---|---|---|
| 1, 2, 3 + znalezisko | **HIGH** | R2 `proposal-loader-topology-architecture.md` | zamknięta |
| 4 | MEDIUM/HIGH | R3 `proposal-loader-limits-strict-parsing-*.md` | zamknięta |
| 5, 6 | MEDIUM | R4 `proposal-legacy-robot-model-boundary.md` | zamknięta |
| 7 | MEDIUM | R5 `proposal-expression-factory-finite-invariant-*.md` | zamknięta |
| 8, 9 | MEDIUM | R1 `proposal-warning-policy.md` | zamknięta |
| 10 | MEDIUM | bez własnego proposala — warunek kontraktowy F2.4 | zamknięta w F2.4 |

## Czego tu nie ma

- **`proposal-loader-topology-implementation.md`** — pominięty świadomie:
  implementacja R2 powstała wprost z zatwierdzonej architektury, na polecenie.
- **R4 jako para** — jeden dokument zamiast dwóch, precedens R1: jedna decyzja
  projektowa, a kod po niej to kilkanaście usuniętych linii.
- **Proposal implementacyjny F2.5Q** — powstanie po zatwierdzeniu architektury.
- **Proposale F2.6 i F2.7** — jeszcze nie powstały; zakres każdego opisuje roadmapa.
