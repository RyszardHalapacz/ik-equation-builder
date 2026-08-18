# Proposal: granica starej warstwy `mt::kinematics::RobotModel` (R4)

## Prompt

> Review kodu, uwagi 5 i 6:
> 5. `RobotModel::within_limits()` akceptuje `NaN` — dla `v = NaN` obie relacje
>    `v < lower` i `v > upper` są fałszywe, więc funkcja zwraca `true`.
> 6. Publiczne `set_chain()` umożliwia UB: `joints_[chain_[k]]` nie sprawdza
>    indeksu. ASan łapie `heap-buffer-overflow` po `set_chain({99})`.
>
> Rekomendacja review: „Ponieważ cała ta warstwa jest praktycznie legacy i jej
> `chain()` nie jest używany przez właściwy `IkEquationBuilder`, skłaniałbym się
> raczej do ograniczenia/wycięcia tego API."

Etap `F2.3.R`, dokument **R4**. Waga MEDIUM, ale **na ścieżce krytycznej**: R2 nie
może wejść na `main` bez R4.

**Jeden dokument, nie para** architektura + implementacja. Precedens: `R1`
(`proposal-warning-policy.md`). Uzasadnienie: jedna decyzja projektowa, a kod po
niej to kilkanaście usuniętych linii.

> **v2 (2026-08-18), po review.** Decyzja architektoniczna zatwierdzona bez
> zmian: wariant A, usunąć całe stare API łańcucha. Poprawione trzy rzeczy —
> zapis kolejności uzgodniony z R2 (2.1), komentarz nad `RobotModel` przestał
> przypisywać typowi gwarancje należące do `load_urdf()` (4.1), doszedł
> bezpośredni `#include <utility>` (4.2). Rozstrzygnięcia review: 3.5 i 9.

## Status weryfikacji

### Stan po R2 i R3

> **v2.** Nagłówek mówi „po R2 i R3", bo taki jest faktyczny stan brancha, a nie
> dlatego, że R3 jest warunkiem R4. Kontrakt releasowalności ustalony w R2
> pozostaje bez zmian; rozpisany w 2.1.

Oba buildy czyste, **312/312**. Uwagi 5 i 6 odtworzone wcześniej uruchomieniem:

```text
within_limits({NaN})                        → true
RobotModel m; m.add_joint({}); m.set_chain({99}); m.within_limits({0.0})
                                            → ASan: heap-buffer-overflow
                                              robot_model.cpp:20
```

### Rozstrzygające: kto tego używa

```text
grep po src/ i tests/, z pominięciem definicji w robot_model.{hpp,cpp}

within_limits   4 trafienia — WSZYSTKIE w komentarzach
set_chain       (jw.)
chain()         (jw.)

    robot_model_loader.cpp:173   "Replaces trace_chain(), which assumed..."
    robot_model_loader.cpp:445   "...set_chain is no longer called..."
    robot_model_loader.cpp:446   "...so RobotModel::chain() stays empty..."
    robot_model_loader.cpp:448   "...within_limits() has no chain to check against."
```

Wszystkie cztery to **komentarze napisane w R2**. **Zero wywołań w kodzie
produkcyjnym i zero w testach.**

Dla kontrastu, reszta klasy ma realnych konsumentów:

```text
joints()   10 trafień      links()   4      name()   1
```

## 1. Cel i zakres

**W zakresie:** rozstrzygnięcie losu `chain_`, `chain()`, `set_chain()` i
`within_limits()` oraz usunięcie UB osiągalnego z publicznego API.

**Poza zakresem:** `Joint`, `Link`, `JointLimits`, `Vec3`, `name()`, `links()`,
`joints()`, `add_*` — mają realnych konsumentów i zostają bez zmian.

## 2. Dlaczego to jest na ścieżce krytycznej

R2 usunął `trace_chain()`, więc `set_chain()` stracił jedynego producenta. Skutek
dla dzisiejszego kodu:

```cpp
bool RobotModel::within_limits(std::span<double const> joint_values) const noexcept {
    if (joint_values.size() != chain_.size()) return false;   // 0 != 0 → false
    for (std::size_t k = 0; k < chain_.size(); ++k) { ... }    // pętla pusta
    return true;                                               // zawsze
}
```

`chain_` jest teraz puste dla każdego robota, więc **`within_limits({})` zwraca
`true` dla dowolnego modelu**, a każde niepuste wejście daje `false`. Funkcja
przestała cokolwiek znaczyć.

To jest powód, dla którego R2 nie jest samodzielnie releasowalne. Wybór **nie**
brzmi „zostawić działający kod czy go usunąć", tylko:

```text
usunąć funkcję, która po R2 nic nie znaczy
        albo
naprawić ją, nadając chain_ nowe źródło
```

A drugie wymaga odpowiedzi na pytanie, co `chain_` w ogóle miałoby zawierać — czyli
dokładnie to pytanie, o którym R2 rozstrzygnął, że loader **nie ma prawa** go
rozstrzygać, bo nie zna `baseLink`/`toolLink`.

### 2.1 Kolejność: implementacja i release to dwie różne rzeczy

> **Dodane w v2.** v1 pisało „wykonać niezwłocznie po R3", co dawało się czytać
> jako sprzeczne z wiążącym zapisem R2 (`R1 → (R2 + R4) → R3 → R5`). Sprzeczności
> nie ma, ale dokumenty muszą mówić to samo.

**Zapis wiążący, zgodny z R2 sekcja 7:** R4 musi zostać wykonane **przed
merge'em brancha zawierającego R2**. R3 może zostać zaimplementowane pomiędzy R2
i R4, bo jest od obu niezależne i samo w sobie jest releasowalne — nie zmienia to
kontraktu releasowalności pary R2+R4.

```text
kolejność zależności / release        kolejność implementacji na branchu
R1  →  (R2 + R4)  →  R5  →  F2.4      R1  →  R2  →  R3  →  R4
R3 niezależne
```

Warunek pozostaje jeden i ten sam w obu dokumentach: **`main` nie może zobaczyć
R2 bez R4**. Dopóki R2 i R3 siedzą na branchu i nic nie zostało zmergowane,
kolejność ich powstawania jest bez znaczenia.

## 3. Warianty

| # | Wariant | Ocena |
|---|---|---|
| **A** | Usunąć `chain_`, `chain()`, `set_chain()`, `within_limits()` | **Wybrany** |
| B | Zostawić i naprawić: bounds check + `isfinite` | Odrzucony — patrz 3.1 |
| C | Zwęzić builder API (prywatne, loader jako `friend`) | Odrzucony — patrz 3.2 |
| D | Przedefiniować `within_limits` na wszystkie jointy zamiast łańcucha | Odrzucony — patrz 3.3 |

### 3.1 Dlaczego nie B (naprawić w miejscu)

Naprawa usuwa objawy, ale zostawia funkcję, która:

- nie ma **żadnego** wywołania w repo,
- po R2 nie ma źródła danych (`chain_` zawsze puste), więc „naprawiona" wersja
  nadal zwracałaby `true` na pusty wektor i `false` na wszystko inne,
- utrwala w API pojęcie „łańcucha przechowywanego w modelu robota", które R2
  właśnie usunął jako błędne.

Dodanie bounds checku do funkcji bez wywołań i bez danych to konserwacja martwego
kodu. Gorzej: sprawia, że wygląda na utrzymywaną.

### 3.2 Dlaczego nie C (zwęzić builder API)

Wariant C rozwiązuje tylko uwagę 6 (UB przez `set_chain`), i to pośrednio. Nie
odpowiada na to, że `chain_` nie ma producenta. Zostawiłby prywatne pole, którego
nikt nie wypełnia, i publiczne `chain()` zwracające zawsze pustkę.

`add_link`/`add_joint`/`set_name` **zostają publiczne**: mają realnego
konsumenta (loader) i nie prowadzą do UB — dopisują do wektorów, nie indeksują.

### 3.3 Dlaczego nie D (sprawdzać wszystkie jointy)

Kuszące: zachować użyteczną funkcję, zmieniając kontrakt na „`joint_values` ma
tyle elementów, ile `joints()`, sprawdź każdy aktuowany". Odrzucone z dwóch
powodów:

1. **Kolejność jointów w pliku nie jest kolejnością osi robota.** Po R2 wiadomo, że
   plik opisuje drzewo; `joints()` to kolejność deklaracji w XML, a nie `q1..qn`
   wzdłuż jakiejś ścieżki. Wektor wartości indeksowany kolejnością XML nie ma
   fizycznego znaczenia.
2. **Właściwym miejscem takiego sprawdzenia jest warstwa `ik`, nie ta.** Solver
   będzie odrzucał rozwiązania poza limitami, a pracuje na `KinematicChain`, gdzie
   kolejność jest wybraną ścieżką i gdzie `KinematicJoint::variable` nadaje
   jointom `q1..qn`. Limity są tam dostępne — `KinematicJoint::limits` jest
   kopiowane z `UrdfJoint::limits` (`KinematicChainBuilder.cpp`, `copyJointData`).

Czyli D przeniósłby użyteczną funkcję do warstwy, w której i tak nie zostanie
użyta.

### 3.4 Czego wariant A **nie** naprawia

Ważne, żeby nie zapisać tego jako „uwaga 5 zamknięta". Usunięcie funkcji nie jest
naprawieniem błędu `NaN` — jest usunięciem miejsca, w którym on występował.
**Lekcja musi podróżować dalej:**

> Sprawdzenie limitów napisane kiedykolwiek w warstwie `ik` (najpewniej przy
> solverze, F3.10) musi odrzucać `NaN` jawnym `!std::isfinite(v)`. Porównania
> `v < lower || v > upper` są dla `NaN` **oba fałszywe**, więc naiwna
> implementacja powtórzy dokładnie ten sam błąd i uzna `NaN` za wartość w
> limitach.

Zapisane jako wiersz w „Known gaps" w `STATUS.md`, żeby nie zginęło razem z
usuniętym kodem.

### 3.5 Nieznana: konsumenci spoza repo

`mt::kinematics` jest warstwą publiczną w sensie języka — `robot_model.hpp` nie
ma nic prywatnego przed `IkEquationBuilder`. **Nie wiem, czy MotionBridge używa
`within_limits()`**; to osobne, prywatne repo i nie mam do niego wglądu.

Argument za usunięciem mimo tej niewiedzy: po R2 ta funkcja zwraca `true` na pusty
wektor i `false` na każdy inny, niezależnie od robota. Konsument, który jej używa,
**już jest zepsuty** — a błąd kompilacji jest lepszym sposobem dowiedzenia się o
tym niż cicha zmiana odpowiedzi.

> **Rozstrzygnięte w review (v2): wariant A mimo tej niewiedzy.** Uzasadnienie
> przyjęte z review: po R2 ewentualny zewnętrzny konsument `within_limits()` już
> dostaje semantycznie bezwartościowe wyniki, a błąd kompilacji jest
> bezpieczniejszy niż pozostawienie API, które wygląda na działające.
>
> **Zamiast tego — gate integracyjny.** Jeśli przed merge'em da się zbudować
> MotionBridge z nową KinemaForge, to build wchodzi do warunków wyjścia (10).
> Jeśli się nie da, brak wglądu zostaje odnotowany, a nie ukryty: usunięcie i tak
> jest właściwą reakcją, bo alternatywa to utrzymywanie martwego API.

**Niewiadoma zamknięta przy implementacji (2026-08-18).** Sprawdzone w
`C:/Users/Komputer/CLionProjects/MotionBrigde`:

```text
MotionBrigde ma WŁASNĄ kopię mt::kinematics
    include/kinematics/robot_model.hpp      własne within_limits/set_chain/chain()
    src/kinematics/robot_model_loader.cpp   własne trace_chain() -> set_chain()

CMakeLists.txt MotionBrigde  →  zero odwołań do builder_rownan / KinemaForge
                                (zależności: googletest, pugixml, yaml-cpp,
                                 Eigen3, orocos_kdl)
```

Czyli **R4 nie może zepsuć builda MotionBridge** — to nie jest konsument tej
biblioteki, tylko osobna kopia tego samego starego kodu. Build integracyjny jako
gate odpada: nie ma czego integrować. Za to ich kopia **wciąż ma oba bugi**
(NaN w `within_limits`, brak bounds-checku w `set_chain`) i realnych konsumentów
`chain()` w `fk`/`ik`/`jacobian`/`selector`/`feasibility`. To osobna sprawa,
poza zakresem tego repo i tego etapu — odnotowana, nie naprawiana tutaj.

## 4. Proponowana zmiana

### 4.1 `robot_model.hpp`

```cpp
class RobotModel {
public:
    [[nodiscard]] std::string_view name() const noexcept;

    [[nodiscard]] std::span<Link const>  links()  const noexcept;
    [[nodiscard]] std::span<Joint const> joints() const noexcept;

    // Builder API — used by the loader.
    void set_name(std::string n);
    void add_link(Link l);
    void add_joint(Joint j);

private:
    std::string        name_;
    std::vector<Link>  links_;
    std::vector<Joint> joints_;
};
```

Usunięte: `chain()`, `within_limits()`, `set_chain()`, pole `chain_`.

Nad klasą, jako zapis decyzji:

```cpp
// Storage model used by the URDF loader.
//
// When produced by load_urdf(), links and joints correspond to the validated
// URDF declarations and preserve file order. The type itself guarantees neither:
// set_name(), add_link() and add_joint() are public, so a caller can build one
// by hand. Validation is the loader's contract, not this aggregate's.
//
// This type deliberately does NOT hold a kinematic chain. It used to: chain_
// carried the indices of one traced linear path, filled by trace_chain() in the
// loader. That was removed because URDF describes a tree, so selecting one path
// needs a base and a tool link, which only a caller knows -- see
// KinematicChainBuilder, which makes that choice and understands branching.
//
// within_limits() went with it. It checked a joint configuration against the
// chain, so with no chain it had nothing to check against; it also had no caller
// anywhere in this repository. A limits check belongs where joint order is
// meaningful -- the ik layer, against KinematicChain -- and whoever writes it
// must reject NaN explicitly: `v < lower || v > upper` is false for NaN, which
// is how the removed version accepted it.
```

> **v2, po review.** Pierwsze zdanie brzmiało „A parsed URDF: links and joints
> exactly as declared, in file order." — a to nieprawda dla typu, który zostawia
> publiczne `set_name`/`add_link`/`add_joint`. Ta sama subtelność co wcześniej z
> `RobotDescription`: gwarancje walidacji należą do `load_urdf()`, nie do
> agregatu. Nowe brzmienie jest prawdziwe również przy publicznym builder API.

Include `<cstdint>` i `<vector>` zostają (`Joint`, `links_`, `joints_`).
`<span>` zostaje (`links()`, `joints()`).

### 4.2 `robot_model.cpp`

```cpp
#include "kinematics/robot_model.hpp"

#include <utility>  // std::move

namespace mt::kinematics {

std::string_view RobotModel::name() const noexcept                 { return name_; }
std::span<Link  const>        RobotModel::links()  const noexcept  { return links_; }
std::span<Joint const>        RobotModel::joints() const noexcept  { return joints_; }

void RobotModel::set_name(std::string n)                { name_ = std::move(n); }
void RobotModel::add_link(Link l)                       { links_.push_back(std::move(l)); }
void RobotModel::add_joint(Joint j)                     { joints_.push_back(std::move(j)); }

}  // namespace mt::kinematics
```

Plik kurczy się z 25 do 17 linii; znika jedyna w nim pętla i jedyne indeksowanie.

> **Dodane w v2, po review.** `#include <utility>` jest nowy — dziś plik używa
> `std::move` i polega na tym, że `<utility>` wjeżdża tranzytywnie przez
> `<string>`/`<vector>` z nagłówka. To ten sam brak bezpośredniego include'a, co
> wcześniej w loaderze. Trzy `std::move` zostają po R4, więc deklaracja zostaje
> jawna.

### 4.3 `robot_model_loader.cpp` — komentarz do poprawienia

Komentarz z R2 (linie 444–448) odwołuje się do rzeczy, które przestaną istnieć:

```cpp
    // Last, deliberately: every joint must be parsed before the graph they form
    // can be judged. Nothing here selects a chain -- selecting one needs a base
    // and a tool link, which only the caller knows (see KinematicChainBuilder).
```

Trzy zdania o `set_chain`, `chain()` i `within_limits` wypadają razem z API.

## 5. Model błędów

**Bez zmian.** R4 nie dodaje ani nie usuwa żadnego `LoadErrorCode`. To jedyny
dokument etapu, który nie dotyka diagnostyki.

## 6. Testy

**Zero nowych.** Świadomie, i warto powiedzieć dlaczego, zamiast zostawić lukę
bez komentarza:

- usunięte funkcje nie miały testów (sprawdzone: zero trafień w `tests/`),
- nie da się napisać testu na nieistnienie metody — robi to kompilator,
- `within_limits` nie zyskuje zamiennika w tym proposalu, więc nie ma nowego
  zachowania do przypięcia.

Wszystkie **312** istniejących testów muszą przejść **bez edycji**. To jest
kryterium tej zmiany: skoro API nie miało konsumentów, usunięcie go nie może
niczego dotknąć. Konieczność zmiany choćby jednego testu oznaczałaby, że analiza
z „Statusu weryfikacji" była błędna.

## 7. Plan zmian w plikach

```text
src/kinematics/robot_model.hpp    -4 składowe, +18 linii komentarza decyzji
src/kinematics/robot_model.cpp    -10 linii (within_limits, set_chain)
                                  +2 linie  (#include <utility>)
src/kinematics/robot_model_loader.cpp   ~4 linie komentarza
STATUS.md                          wiersz w Known gaps (NaN, 3.4)
```

Zero zmian w: warstwie `ik`, FK, symbolice, TCP, modelu równań, testach.

## 8. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| MotionBridge woła `within_limits()` | **nieznane** | 3.5 — jedyne pytanie, na które nie odpowiem z repo. Po R2 funkcja i tak jest zepsuta, więc błąd kompilacji jest lepszy od cichej złej odpowiedzi |
| Usunięcie publicznej metody to zmiana API | średnie | Świadome; alternatywą jest utrzymywanie martwego kodu, który wygląda na żywy |
| Lekcja o `NaN` zginie razem z kodem | **realne** | 3.4 — wiersz w `STATUS.md`, a nie tylko w tym proposalu |
| `chain_` okaże się potrzebne w przyszłości | niskie | `KinematicChain` w warstwie `ik` już tę rolę pełni, lepiej: niesie `q1..qn` i wybraną ścieżkę |

## 9. Rozstrzygnięte w review (v2)

| # | Pytanie z v1 | Rozstrzygnięcie |
|---|---|---|
| 1 | Wariant A (usunąć) czy B (naprawić) wobec nieznanego konsumenta zewnętrznego — 3.5 | **A.** Usunąć. Compile error jest bezpieczniejszy niż API, które wygląda na działające. Jeśli da się zbudować MotionBridge — do gate'u integracyjnego |
| 2 | Czy `add_link`/`add_joint`/`set_name` zostają publiczne, czy zwęzić do `friend load_urdf` | **Zostają publiczne.** Bez `friend`. R4 ma usunąć błędny koncept łańcucha, a nie przeprojektowywać sposób konstrukcji `RobotModel` — zwężanie mutatorów poszerzyłoby zakres. Konsekwencja dla komentarza nad klasą: 4.1 |

Otwarte pytania: brak.

> **Korekta gate'u, po review v2.** Poprzednie brzmienie tego wiersza — „grep
> `within_limits|set_chain|chain()` → zero trafień" — było błędne: R4 celowo
> zostawia teksty o `within_limits()` i `chain_` w komentarzu nad `RobotModel`
> oraz wiersz w `STATUS.md`, więc zero trafień nie może wyjść. Warunkiem jest
> brak deklaracji, definicji i wywołań, nie brak wzmianek. Korekta gate'u, nie
> zmiana implementacji.

## 10. Rekomendacja końcowa

Wykonać przed merge'em brancha zawierającego R2 — **R2 czeka w branchu na tę
zmianę** (2.1). W praktyce: teraz, bo R1–R3 są już zaimplementowane. Po R4 cała
paczka R1–R4 jest spójna i może wejść na `main`.

Gate:

```text
build (Release)        0 ostrzeżeń
build-werror           0 ostrzeżeń
ctest                  312/312 w obu, BEZ EDYCJI żadnego istniejącego testu
brak deklaracji, definicji i wywołań:
                       RobotModel::within_limits
                       RobotModel::set_chain
                       RobotModel::chain
                       (komentarze w robot_model.hpp i wiersz w STATUS.md
                       zostają celowo — grep po samych nazwach NIE da zera)
STATUS.md              Known gaps zawiera wiersz o NaN dla przyszłego
                       sprawdzania limitów w warstwie ik
MotionBridge           NIE DOTYCZY — osobna kopia mt::kinematics, zero
                       zależności od KinemaForge; sprawdzone, patrz 3.5
```
