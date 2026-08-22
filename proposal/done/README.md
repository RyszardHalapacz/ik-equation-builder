# Archiwum zamkniętych etapów

**Utworzone:** 2026-08-21 · **zaktualizowane:** 2026-08-22 · **etapów:** 10 · **proposali:** 41

Tu leżą proposale etapów, które są **zaimplementowane, zbudowane i pokryte
testami**. Przeniesienie do `done/` jest ostatnim krokiem etapu — po clean
buildzie, pełnych testach i aktualizacji `STATUS.md`. Folder `proposal/` trzyma
wtedy tylko to, co jest w toku, dzięki czemu „co jest bieżące" widać z listingu
katalogu, a nie z lektury nagłówków.

Trzy rzeczy, o których warto pamiętać przy czytaniu tych plików:

- **To zapis decyzji, nie dokumentacja kodu.** Kod w tym repo żyje w `.md` aż do
  zatwierdzenia, więc te dokumenty zawierają pełne źródła — w brzmieniu z dnia
  zatwierdzenia. Stan faktyczny opisuje [`STATUS.md`](../../STATUS.md); jeśli
  proposal i kod się różnią, prawdę mówi kod.
- **Wartość jest w wariantach odrzuconych.** Każdy z tych dokumentów niesie
  uzasadnienia rozwiązań, których w kodzie już nie widać — i to jest jedyne
  miejsce, gdzie one istnieją.
- **Nic tu nie jest przenumerowane.** Nazwy plików i etapów zostały takie, jak w
  chwili zatwierdzenia; zmieniły się wyłącznie ścieżki.

Pełny spis wszystkich dokumentów repo, razem z żywymi:
[`../KATALOG.md`](../KATALOG.md).

## Etapy

| Folder | Etap | Co zamknął | Dokumentów |
|---|---|---|---|
| [`phase1-urdf-to-symbolic-fk/`](phase1-urdf-to-symbolic-fk/) | Faza 1 | URDF → łańcuch → symboliczne FK, zweryfikowane numerycznie, za jedną fasadą | 17 |
| [`F2.1-F2.2-tcp-transform/`](F2.1-F2.2-tcp-transform/) | F2.1, F2.2 | Stała transformacja `chain_tip → TCP` i `T_base_tcp(q)` | 2 |
| [`F2.3-ik-equation-model/`](F2.3-ik-equation-model/) | F2.3 | `Equation`, `IkTarget`, `IkEquationSystem` — czym *jest* równanie IK i cel | 2 |
| [`F2.3.R-review-remediation/`](F2.3.R-review-remediation/README.md) | F2.3.R | 10 uwag z review kodu z 2026-08-17, w tym dwie o wadze HIGH | 7 (+ własny README) |
| [`F2.4-constraint-builder-position/`](F2.4-constraint-builder-position/) | F2.4 | `ConstraintBuilder` dla `PositionTarget` — trzy równania pozycji | 2 |
| [`F2.5-constraint-builder-fullpose/`](F2.5-constraint-builder-fullpose/) | F2.5 | `ConstraintBuilder` dla `PoseTarget` — dwanaście równań, w tym dziewięć rotacji | 2 |
| [`F2.7-facade-integration/`](F2.7-facade-integration/) | F2.7 | Jedno wejście: URDF + opcjonalny TCP + cel → IkEquationSystem; milestone Fazy 2 | 2 |
| [`F2.6-directional-constraints/`](F2.6-directional-constraints/) | F2.6 | Dwie rodziny ograniczeń częściowej orientacji: kierunek osi narzędzia i kąt do kierunku odniesienia | 3 |
| [`F2.5Q-quaternion-target/`](F2.5Q-quaternion-target/) | F2.5Q | Kwaternion jako wejście orientacji celu — jedne drzwi na granicy modelu, własna tolerancja normy | 2 |

## Czego tu nie ma

- [`../analysis-ik-pipeline-constraint-builder.md`](../analysis-ik-pipeline-constraint-builder.md)
  — analiza, z której wyrosło całe `ConstraintBuilder` (F2.4–F2.6). Zostaje w
  `proposal/`, bo F2.6 wciąż z niej korzysta: pytanie 4 o reprezentację
  ograniczeń kierunkowych nie zostało przez F2.5 zamknięte.
- **`proposal-loader-topology-implementation.md`** (R2) — nie powstał; implementacja
  wyszła wprost z zatwierdzonej architektury, na polecenie. Szczegóły w README etapu.
- **Proposale Fazy 3** — jeszcze nie powstały. Pierwszy to F3.1, semantyka
  rewrite engine'u.
