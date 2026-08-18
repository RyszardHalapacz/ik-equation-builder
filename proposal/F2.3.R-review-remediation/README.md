# F2.3.R — remediacja po review kodu

**Etap:** F2.3.R (między F2.3 i F2.4) · **status:** bieżący · **utworzono:** 2026-08-17

Ten folder zawiera proposale powstałe w reakcji na pełne review kodu z
2026-08-17. Trzyma je osobno od `proposal/`, bo nie są kolejnym krokiem
funkcjonalnym roadmapy — są naprawą znalezisk, i warto, żeby dało się je czytać
jako jedną spójną paczkę.

Kontekst, dowody i Definition of Done etapu:
[`../KinemaForge_Phase2_Phase3_Roadmap.md`](../KinemaForge_Phase2_Phase3_Roadmap.md),
sekcja **F2.3.R**.

## Zakres

Review zgłosiło 10 uwag. **Wszystkie 10 zostało odtworzone eksperymentalnie**
przed przyjęciem — nie na podstawie samej lektury kodu. Dwie mają wagę HIGH i
obie siedzą w starej warstwie `mt::kinematics`, czyli w kodzie sprzed właściwego
pipeline'u.

Review nie zgłosiło żadnej uwagi do FK, warstwy symbolicznej, transformacji TCP
ani modelu równań. **Nic z F2.1–F2.3 nie jest cofane ani przepisywane.**

## Dokumenty

| # | Dokument | Uwagi z review | Waga | Stan |
|---|---|---|---|---|
| R1 | `proposal-warning-policy.md` | 8, 9 | MEDIUM (proces) | v3 APPROVED — **ZAIMPLEMENTOWANE** |
| R2 | `proposal-loader-topology-architecture.md` | 1, 2, 3 + znalezisko dodatkowe | **HIGH** | v3 APPROVED — **ZAIMPLEMENTOWANE** |
| R2 | `proposal-loader-topology-implementation.md` | — | — | **pominięty** — implementacja wykonana wprost z zatwierdzonej architektury, na polecenie |
| R3 | `proposal-loader-limits-strict-parsing-architecture.md` | 4 | MEDIUM/HIGH | v3 APPROVED — **ZAIMPLEMENTOWANE** |
| R3 | `proposal-loader-limits-strict-parsing-implementation.md` | — | MEDIUM/HIGH | v3 APPROVED — **ZAIMPLEMENTOWANE** |
| R4 | `proposal-legacy-robot-model-boundary.md` | 5, 6 | MEDIUM | v2 APPROVED — **ZAIMPLEMENTOWANE** |
| R5 | `proposal-expression-factory-finite-invariant-architecture.md` | 7 | MEDIUM | do napisania |
| R5 | `proposal-expression-factory-finite-invariant-implementation.md` | — | MEDIUM | do napisania |

R1 jest jednym dokumentem, nie parą — precedens: `../proposal-loader-test-coverage.md`.

Uwaga 10 (`IkEquationSystem` przyjmuje równania na symbol spoza `unknowns`) nie
dostaje własnego proposala. Wchodzi jako warunek kontraktowy do F2.4:
`ConstraintBuilder` musi być jedynym producentem `IkEquationSystem` i sam
gwarantować spójność zbioru symboli.

## Kolejność

```text
R1  →  (R2 + R4)  →  R3  →  R5  →  F2.4
                            (R5 może iść równolegle — nie dotyka loadera)
```

**R2 i R4 są jedną jednostką releasowalną**, choć osobnymi proposalami. Ustalone
w review R2: usunięcie `trace_chain()` sprawia, że `RobotModel::chain_` jest puste
dla każdego robota, a wtedy `within_limits({})` zwraca `true` dla dowolnego —
funkcja przestaje cokolwiek znaczyć. **Nie wolno scalać remediacji do `main` z R2
bez R4.** Szczegóły: `proposal-loader-topology-architecture.md`, sekcja 7.

**R1 jest pierwsze, nie ostatnie**, i to jedyne odstępstwo od kolejności
proponowanej w review. R1 to dwie jednoliniowe poprawki plus flagi w CMake, a
jednocześnie jest to *mechanizm ochronny* dla wszystkich kolejnych zmian —
zrobione na końcu oznaczałoby, że R2–R5 pisze się bez tej ochrony. R1 zawiera
przy tym dokładnie to zabezpieczenie, które wyłapuje regresję z uwagi 9.

## Gate

F2.4 (`ConstraintBuilder`) jest **zablokowane** do zamknięcia tego etapu.
Warunki wyjścia w roadmapie, sekcja „Definition of Done etapu F2.3.R".

## Proces

Jak w całym repo: proposal architektoniczny → review → proposal implementacyjny
→ review → implementacja → clean build → pełne testy → aktualizacja
`../../STATUS.md`. Kod żyje w `.md` do momentu zatwierdzenia.
