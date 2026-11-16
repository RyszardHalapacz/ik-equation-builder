# Proposal: naprawy po drugim audycie — architektura (F3.R2, v1)

## Prompt

Drugi audyt z 2026-11-15 (po remediacji siedmiu defektów z `AUDYT_2026-11-15.md`)
wskazał pięć kolejnych problemów. Review tych ustaleń, przekazane przez
użytkownika z prośbą o dokończenie:

> Uwagi są trafne. Punkty 1 i 2 blokują wydanie, a wcześniejsze 579 zielonych
> testów nie obejmowało tych przypadków.
>
> 1. Szerokie limity — potrzebne są dwa zabezpieczenia. Deduplikację należy
>    przenieść przed rozwijanie pełnych obrotów, gdzie operuje na kilku gałęziach
>    geometrycznych. Osobno trzeba ograniczyć rozmiar wyniku: policzyć liczbę
>    kombinacji przed alokacją i zwrócić jawny `ResourceLimitExceeded`. Samo
>    przyspieszenie deduplikacji nie rozwiąże eksplozji liczby wyników. Docelowo
>    gałąź bazowa z zakresami `k` będzie lepszym API. Nie wolno po prostu ucinać
>    listy i raportować sukcesu.
> 2. Niespójność tolerancji potwierdziłem niezależnie. Dla KR640 odtworzyłem
>    dokładnie 8 rozwiązań / 755 NoSolution / 237 odrzuceń. Zaokrąglenie wyłącznie
>    orientacji daje ten sam wynik; zaokrąglenie wyłącznie pozycji daje 1000/1000
>    sukcesów. Ortonormalizacja jest rozsądnym rozwiązaniem, ale wymaga świadomej
>    zmiany kontraktu: obecnie dokumentacja obiecuje brak naprawiania macierzy.
>    Normalizacja długości kwaternionu zachowuje reprezentowaną rotację; projekcja
>    macierzy wybiera pobliską rotację. Korekta powinna być ograniczona,
>    udokumentowana i dostępna do sprawdzenia przez wywołującego. Samo poluzowanie
>    kontroli FK byłoby słabszą naprawą.
> 3. Trygonometria — potwierdzone. […] Brak obsługi współczynników jest
>    ograniczeniem reguły, natomiast konieczność drugiego przebiegu to błąd
>    zachowania upraszczacza. Naprawa powinna rozpoznawać znak i wspólny czynnik
>    oraz dochodzić do stabilnej postaci, zachowując warunki dziedziny.
> 4. Kanonizacja — rozdzieliłbym dwa zagadnienia. Różne postacie po wcześniejszym
>    zaokrągleniu stałych nie muszą oznaczać błędu: kanonizator nie odzyska
>    informacji utraconej przez fabrykę. Natomiast zamiana dzielenia przez zero w
>    skończony wynik jest istotną zmianą zachowania. Trzeba jasno określić, czy
>    gwarancja dotyczy dziedziny algebraicznej, czy również ewaluacji w double.
>    Obecnie nie gwarantuje tego drugiego.
>
> Przekroczenia limitów warto przycinać wyłącznie w dopuszczonym marginesie
> numerycznym, przed końcową kontrolą FK.

Etap `F3.R2`. To jest dokument **architektury**: decyzje, kształt API, plan
testów, kompromisy. Pełny kod trafi do osobnego dokumentu implementacyjnego po
zatwierdzeniu tego.

## Stan po implementacji (2026-11-15)

Użytkownik polecił implementację bezpośrednio („dobra implementuj"), bez osobnego
dokumentu implementacyjnego. Decyzje z rozdziału 11 przyjęto zgodnie z
rekomendacjami: wariant A (projekcja z raportowaną korektą), budżet 4096,
`kMaxOrientationCorrection = 2e-8`, pełny rozkład wieloznaczny w trygonometrii.

Zmierzone po zmianie: 613/613 testów w Release, Debug i `KINEMAFORGE_WERROR=ON`.
Największa korekta orientacji na 100 000 losowych macierzy zaokrąglonych do
1e-8 i przyjętych przez walidację mieści się poniżej 1e-8 (test
`NearestRotationTest.ProjectsEveryAcceptedMatrixWithinTheDocumentedBound`).

Odstępstwo od planu testów: dla celów zaokrąglonych do 1e-8 konfiguracja
generująca jest odzyskiwana z tolerancją 1e-4, nie 3e-7 — przesunięcie celu o
5e-9 jest wzmacniane w przestrzeni przegubów w pobliżu osobliwości. Zgodność FK
każdej gałęzi z celem zrzutowanym pozostaje sprawdzana do 2e-9.

Reszta dokumentu opisuje projekt w brzmieniu sprzed implementacji.

## Status weryfikacji

Liczby opisujące **stan obecny** pochodzą z pomiarów audytu (programy w
`CLionProjects\_claude_scratch\audit2\`: `probe.cpp`, `wide.cpp`, `fuzz_sym.cpp`,
`mini.cpp`) na Release, MinGW GCC 13.1.0, przy 579/579 zielonych testach.

Wszystko, co dotyczy **stanu docelowego**, jest projektem. Na potrzeby tego
dokumentu nie skompilowano ani nie uruchomiono żadnego nowego kodu. Bramki z
rozdziału 9 są deklaracją, nie pomiarem.

## 1. Zakres

| # | Problem | Waga | Rozdział |
|---|---|---|---|
| R2-1 | Szerokie limity osi zawieszają solver (eksplozja pełnych obrotów + deduplikacja O(n²)) | blokuje wydanie | 3 |
| R2-2 | Cel przechodzi walidację (1e-8), a kontrola FK (2e-9) odrzuca każdą gałąź | blokuje wydanie | 4 |
| R2-3 | Upraszczacz trygonometryczny pomija pary ze znakiem i współczynnikiem; nie jest idempotentny | błąd zachowania | 5 |
| R2-4 | Nieokreślony zakres gwarancji dziedziny w kanonizacji | kontrakt/dokumentacja | 6 |
| R2-5 | Zwracane kąty mogą przekraczać limit o ~1,6e-13 | drobne | 7 |

**Poza zakresem:** osobliwość osi barku, ładowanie URDF `continuous`, API „gałąź
bazowa + zakresy `k`" (patrz 3.4), porządki w repo (`zip/`, `.idea/`, zaślepka
`src/Kinematics.*`, zduplikowane `tests/generated/`), ścieżka symboliczna
`ConstraintBuilder` (patrz 4.5).

## 2. Stan obecny

### 2.1 Rozwijanie pełnych obrotów — `AnalyticIkSolver.cpp`

- `equivalentsWithinLimits()` (`:434`) materializuje wektor **wszystkich**
  `q + 2πk` w limitach. Jedynym ograniczeniem jest zakres `int64` w
  `detail::turnsWithin()`.
- `expandRegularBranch()` (`:452`) tworzy iloczyn kartezjański sześciu takich
  wektorów, po jednym `SolutionBranch` (z sześcioma `std::string`) na kombinację.
- Pętla końcowa (`:711`) dla każdej gałęzi wykonuje `branchMatches()` (FK) i
  `duplicateRegular()` względem wszystkich już przyjętych — O(n²).
- `expandSingularWrist()` (`:491`) robi to samo dla rodzin: iloczyn odpowiedników
  `q1,q2,q3,q5` razy zakres `k`.
- `GeneratedIkRuntime.hpp`: `equivalents()` (`:208`), `appendRegular()` (`:222`),
  `appendSingular()` (`:234`) — to samo wyliczanie, bez deduplikacji.

Pomiar (KR640, cel z `q=[0.4,-0.5,0.7,0.8,-0.9,1.1]`):

| Limity | Wynik |
|---|---|
| ±100 rad na osiach 4 i 6 | 4096 gałęzi, 0,02 s |
| ±1000 rad na osiach 4 i 6 | brak wyniku po 120 s |
| ±100 000 rad na osi 6 | brak wyniku po 120 s |

### 2.2 Tolerancje orientacji

- `TargetValidation.hpp:47` — `kOrientationTolerance = 1e-8` dla `|RᵀR − I|` i
  `|det − 1|`. Komentarz obiecuje: walidacja sprawdza, nigdy nie przepisuje;
  naprawa nieortogonalnej macierzy należy do wywołującego.
- `AnalyticIkSolver.cpp:577` i `GeneratedIkRuntime.hpp:283` — `poseMatches()`
  wymaga zgodności **każdego elementu** macierzy z dokładnością 2e-9 względem
  macierzy wejściowej, której solver nie ortonormalizuje.
- `QuaternionTarget.cpp` — ścieżka kwaternionowa normalizuje długość przed
  budową macierzy.

Pomiar (1000 celów na robota, macierz i pozycja zaokrąglone do siatki):

| Siatka | KR640 ok / NoSolution / odrzucone | KR4 |
|---|---|---|
| 1e-9 | 1000 / 0 / 0 | 1000 / 0 / 0 |
| 1e-8 | 8 / 755 / 237 | 5 / 761 / 234 |
| 3e-8 | 0 / 7 / 993 | 0 / 9 / 991 |

Review potwierdziło niezależnie, że winna jest wyłącznie orientacja.

### 2.3 Upraszczacz trygonometryczny — `TrigonometricSimplifier.cpp`

- `trigSquare()` (`:84`) rozpoznaje wyłącznie węzeł `Multiply(sin(u), sin(u))`
  albo `Multiply(cos(u), cos(u))` — bez znaku, bez współczynnika, bez innych
  czynników.
- `simplifySum()` (`:203`) najpierw upraszcza dzieci, potem szuka par.
- `simplifyNode()`, gałąź `Negate` (`:317`): `Negate(Add)` jest zostawiane bez
  zmian jako „bariera reprezentowalności"; negacja wepchnięta do sumy przez
  końcowe `canonicalize()` nie jest już ponownie analizowana.
- `rebuildSum()` (`:164`): gdy po uproszczeniu zostają same stałe, których nie da
  się zsumować dokładnie, zwracane jest **oryginalne, nieuproszczone** poddrzewo.

Pomiar (`mini.cpp`):

| Wejście | Po 1. przebiegu | Po 2. przebiegu |
|---|---|---|
| `a − (sin²b + cos²b)` | bez zmian | bez zmian |
| `2·sin²b + 2·cos²b` | bez zmian | bez zmian |
| `a·sin²b + a·cos²b` | bez zmian | bez zmian |
| `−((a − (sin²b+cos²b))·(sin²c+cos²c))` | `−a + sin²b + cos²b` | `1 − a` |

Fuzzer: 22 nieidempotentne przypadki na 300 000 wyrażeń.

### 2.4 Kanonizacja — `ExpressionCanonicalizer.cpp`, `ExpressionFactory.cpp`

- `ExpressionFactory::add/multiply` skleja dwie sąsiednie stałe zawsze, z
  zaokrągleniem. `buildSum()/buildProduct()` łączą stałe tylko wtedy, gdy wynik
  jest dokładny. Skutek: `(x + 1.91) + 0.1` i `x + 2.01` mają różne postacie.
- Kanonizator zachowuje zbiór dzielników (`hasSameDomainGuards`), ale zmienia
  kolejność sumowania. `b / (1 − (sin²a + cos²a))` potrafi przed kanonizacją dać
  `DivisionByZero`, a po niej skończoną liczbę rzędu 1e16 (i odwrotnie). Fuzzer:
  40 „poszerzeń" i 31 „zawężeń" na 300 000 wyrażeń, wszystkie tej natury.

## 3. R2-1 — szerokie limity

### 3.1 Decyzja: trzy fazy zamiast „rozwiń wszystko, potem sprzątaj"

```text
faza A  gałęzie geometryczne      ≤ 8 regularnych (+ rodziny osobliwe)
        deduplikacja modulo 2π    tutaj, na kilku elementach
        kontrola FK reprezentanta jak dziś (:705)

faza B  liczenie                  dla każdej gałęzi: iloczyn liczb obrotów
        suma > budżet             -> ResourceLimitExceeded, bez alokacji

faza C  rozwijanie                tylko gdy faza B przeszła
        przycięcie do limitów     R2-5
        końcowa kontrola FK       każdej zwracanej gałęzi (liniowo)
```

Deduplikacja po rozwinięciu znika: dwie gałęzie geometryczne różne modulo 2π dają
rozłączne rozwinięcia, a w obrębie jednej gałęzi kombinacje różnią się co
najmniej jednym `k`. Końcowa kontrola FK zostaje dla każdej gałęzi — jej koszt
jest liniowy i ograniczony budżetem.

### 3.2 Liczenie bez materializacji

Liczba odpowiedników osi wynika wprost z `detail::TurnRange`
(`last − first + 1`), bez budowania wektora. Iloczyn sześciu liczników i suma po
gałęziach są liczone z kontrolą przepełnienia (nasycenie do `SIZE_MAX`). Dla
rodzin osobliwych licznik to iloczyn odpowiedników `q1,q2,q3,q5` razy długość
zakresu `k`.

Liczenie jest górnym oszacowaniem: pojedyncze wartości na brzegu mogą odpaść przy
kontroli limitu albo FK. Budżet dotyczy oszacowania, nie liczby ostatecznie
zwróconych gałęzi. To jest zamierzone — decyzja zapada przed alokacją.

### 3.3 Kształt API

Solver ogólny:

```cpp
struct AnalyticIkSolverOptions
{
    // Górna granica liczby gałęzi (regularnych i rodzin) w jednym wyniku.
    std::size_t maxBranches{4096};
};

enum class AnalyticIkSolverErrorCode
{
    InvalidTarget,
    InvalidKinematicGeometry,
    UnsupportedKinematicStructure,
    NonFiniteComputation,
    ResourceLimitExceeded          // nowe
};

struct AnalyticIkSolverError
{
    AnalyticIkSolverErrorCode code{};
    std::string message;
    std::optional<TargetValidationError> targetError;
    std::optional<std::size_t> requiredBranches;   // nowe; nasycone przy przepełnieniu
};

solve(chain, target, tcp = nullopt, options = {});
```

Wygenerowany runtime — parametr wywołania, nie pole `Parameters`, żeby format
generowanego nagłówka się nie zmienił:

```cpp
enum class Status { Solutions, NoSolution, InfiniteFamily, InvalidTarget,
                    ResourceLimitExceeded };          // nowe

struct Result
{
    Status status{Status::NoSolution};
    std::vector<Branch> branches;      // puste przy ResourceLimitExceeded
    std::size_t requiredBranches{};    // nowe
};

Result solve(const Parameters&, const Pose&, std::size_t maxBranches = 4096);
```

Zasady:

- Przekroczenie budżetu to **błąd**, nigdy częściowy sukces. Lista nie jest
  ucinana.
- Błąd niesie wymaganą liczbę gałęzi, więc wywołujący może świadomie podnieść
  budżet albo zawęzić limity.
- Domyślne 4096 odpowiada zmierzonemu przypadkowi „±100 rad na dwóch osiach" i
  jest dwa rzędy wielkości powyżej fabrycznego KR640 (do 32 gałęzi).

### 3.4 Świadomie odłożone: gałąź bazowa z zakresami `k`

Docelowe API (jedna gałąź geometryczna + zakres `k` na oś) usuwa eksplozję u
źródła, ale zmienia `SolutionBranch`, `Branch`, wszystkie testy regresyjne i
kontrakt z przyszłym konsumentem (MotionBridge). To osobny etap. Faza B tego
proposalu jest z nim zgodna: liczniki obrotów, które tu powstają, są dokładnie
tymi zakresami.

## 4. R2-2 — orientacja celu

### 4.1 Warianty

| Wariant | Opis | Ocena |
|---|---|---|
| **A. Ograniczona projekcja w solverze** | solver rzutuje zwalidowaną macierz na najbliższą rotację, rozwiązuje i weryfikuje względem niej, raportuje wielkość korekty | **rekomendowany** |
| B. Ostrzejsza tolerancja na wejściu solvera | solver odrzuca cel z `InvalidTarget`, gdy odchyłka przekracza to, co kontrola FK jest w stanie odtworzyć | spójne z dzisiejszym „nie naprawiamy", ale odrzuca dane 8-cyfrowe, które walidacja celowo dopuszcza |
| C. Poluzowanie kontroli FK | próg FK ≥ tolerancja walidacji | słabsze: kontrola FK przestaje wykrywać błędy wyprowadzenia rzędu 1e-8 |

### 4.2 Decyzja (wariant A)

1. `validate(RotationMatrix3)` i `kOrientationTolerance` **bez zmian**. Walidacja
   nadal niczego nie przepisuje.
2. Nowa publiczna funkcja, dostępna także dla wywołującego:

   ```cpp
   struct OrientationProjection
   {
       RotationMatrix3 rotation;   // najbliższa rotacja (norma Frobeniusa)
       double correction{};        // max |rotation_ij − input_ij|
   };

   // Warunek wstępny: validate(input) przeszło.
   [[nodiscard]] OrientationProjection nearestRotation(const RotationMatrix3& input);
   ```

   Algorytm: czynnik ortogonalny rozkładu biegunowego iteracją Newtona
   `X ← ½(X + X⁻ᵀ)`. Dla wejścia odległego od rotacji o ≤ 1e-8 zbieżność jest
   kwadratowa (2–3 kroki); nie potrzeba SVD ani zewnętrznej biblioteki.
3. Solver (oba backendy) po walidacji wykonuje projekcję, **rozwiązuje i
   weryfikuje FK względem macierzy zrzutowanej**, a wynik niesie korektę:

   ```cpp
   struct AnalyticIkSolution { …; double orientationCorrection{}; };
   struct Result             { …; double orientationCorrection{}; };
   ```
4. Korekta jest ograniczona: `kMaxOrientationCorrection`. Dla macierzy, która
   przeszła walidację, korekta jest tego samego rzędu co tolerancja walidacji;
   przekroczenie granicy oznacza błąd wewnętrzny i daje `InvalidTarget`, a nie
   ciche rozwiązanie. Dokładną wartość stałej (propozycja: 2e-8) dokument
   implementacyjny uzasadni pomiarem na 500 000 losowych macierzy, tak jak
   zrobiono to dla `kOrientationTolerance`.

### 4.3 Zmiana kontraktu — nazwana wprost

- Dziś: „solver zwraca gałęzie, których FK zgadza się z **podanym** celem do 2e-9".
- Po zmianie: „solver zwraca gałęzie, których FK zgadza się do 2e-9 z
  **najbliższą rotacją** podanego celu; odległość tej rotacji od wejścia jest
  zwracana w `orientationCorrection` i nie przekracza `kMaxOrientationCorrection`".

Różnica względem kwaternionu zostaje zapisana w komentarzu: normalizacja długości
nie zmienia reprezentowanej rotacji, projekcja macierzy **wybiera** pobliską.
Dlatego korekta jest raportowana, a nie ukryta. Dla czystego wejścia (macierz z
FK w double) korekta jest rzędu 1e-16.

Komentarz przy `kOrientationTolerance` („the right fix is an explicit
orthonormalisation on the caller's side") zostaje uzupełniony: nadal obowiązuje
dla danych po `float` (~1e-6, odrzucane), natomiast w paśmie akceptacji solver
analityczny wykonuje ograniczoną, raportowaną projekcję sam.

### 4.4 Komunikat `NoSolution`

Tekst „geometric branches exist but none satisfies joint limits and pose
round-trip" zostaje, ale po tej zmianie nie będzie już wywoływany przez
nieortogonalność wejścia.

### 4.5 Poza zakresem: ścieżka symboliczna

`ConstraintBuilder` wstawia macierz celu do równań bez projekcji, więc układ
równań dla celu z odchyłką 1e-8 pozostaje niespełnialny dokładnie. To zachowanie
istniejące i udokumentowane w `TargetValidation.hpp`; `nearestRotation()` daje
wywołującemu narzędzie, ale ten proposal nie zmienia `ConstraintBuilder`.

## 5. R2-3 — trygonometria

### 5.1 Uogólnione dopasowanie pary

Każdy składnik sumy w postaci kanonicznej jest rozkładany na:

```text
składnik = znak · współczynnik · Π(pozostałe czynniki) · t(u) · t(u)
           znak ∈ {+1, −1}; współczynnik: stała dodatnia; t ∈ {sin, cos}
```

Dwa składniki tworzą parę, gdy mają ten sam znak, ten sam współczynnik
(porównanie dokładne), strukturalnie równe listy pozostałych czynników, ten sam
argument `u` i przeciwne `t`. Para jest zastępowana jednym składnikiem
`znak · współczynnik · Π(pozostałe czynniki)`.

Gdy składnik ma kilka możliwych rozkładów (np. `sin²u·cos²u`), sprawdzane są
wszystkie; liczba kandydatów jest ograniczona liczbą różnych kwadratów w
składniku.

### 5.2 Dziedzina

- Istniejący warunek zostaje: tożsamość nie jest stosowana, gdy `u` zawiera
  dzielnik (`divisorsOf(u)` niepuste).
- Pozostałe czynniki **zostają w wyniku**, więc ich dzielniki są zachowane.
- Współczynniki nie są łączone arytmetycznie (`2·sin² + 3·cos²` zostaje bez
  zmian) — nie powstaje żadna nowa zaokrąglona stała.

### 5.3 Stabilna postać

Dwa źródła nieidempotencji i ich usunięcie:

1. **Negacja wepchnięta do sumy po analizie dzieci.** Po 5.1 zanegowane pary są
   rozpoznawane od razu. Dodatkowo `simplifyNode()` powtarza analizę sumy, gdy
   końcowa kanonizacja zwróciła **nowy** węzeł `Add`, aż do punktu stałego.
   Pętla jest skończona: każde zastąpienie pary ściśle zmniejsza liczbę węzłów.
2. **Dzieci upraszczane przed szukaniem pary.** Gdy argument `u` upraszcza się do
   stałej, `sin(u)·sin(u)` zwija się do liczby i wzorzec znika, a suma dwóch
   niedokładnie sumowalnych stałych trafia na barierę reprezentowalności.
   Kolejność zmienia się na: dopasuj pary na składnikach wejściowych → uprość
   dzieci pozostałych → dopasuj ponownie → powtarzaj do punktu stałego.

Kontrakt po zmianie: `simplify(simplify(e))` jest strukturalnie równe
`simplify(e)`. Jest to własność **testowana** (rozdział 9), nie dowiedziona.
Bariera reprezentowalności w `rebuildSum()` zostaje; jeżeli test własności wykaże
pozostałe przypadki nieidempotencji na barierze, dokument implementacyjny musi je
rozstrzygnąć przed zatwierdzeniem, a nie wyłączyć z testu.

## 6. R2-4 — zakres gwarancji kanonizacji

### 6.1 Stałe zaokrąglone przez fabrykę — nie jest to błąd

Postać kanoniczna jest zdefiniowana dla wyrażenia **takiego, jakie zbudowano**.
`ExpressionFactory` skleja sąsiednie stałe ze stratą; kanonizator tej informacji
nie odzyska. Bez zmian w kodzie. Zachowanie zostaje opisane w
`ExpressionCanonicalizer.hpp` i przypięte testem dokumentującym
(`(x + 1.91) + 0.1` kontra `x + 2.01`).

### 6.2 Dziedzina — gwarancja algebraiczna, nie ewaluacyjna

Kontrakt zostaje zapisany wprost:

- **Gwarantowane:** zbiór dzielników wyrażenia jest zachowany
  (`hasSameDomainGuards`). Wyrażenie, którego dzielnik jest algebraicznie zerem,
  nie stanie się po kanonizacji wyrażeniem bez tego dzielnika.
- **Niegwarantowane:** identyczny wynik `ExpressionEvaluator` w `double`.
  Zmiana kolejności działań może zmienić zaokrąglenie, w tym to, czy dzielnik
  wychodzi dokładnie zerem. `DivisionByZero` przed kanonizacją może stać się
  dużą skończoną liczbą po niej i odwrotnie.

Zmiany: komentarz kontraktu w `ExpressionCanonicalizer.hpp` i
`TrigonometricSimplifier.hpp`, wpis w `STATUS.md` (sekcje F3.2/F3.3 i tabela
ograniczeń), test przypinający przykład `b / (1 − (sin²a + cos²a))`.

Wzmocnienie do gwarancji ewaluacyjnej wymagałoby rezygnacji z przestawiania
składników albo progu „bliskie zera" w ewaluatorze. Oba rozwiązania zmieniają
założenia F3.1/F3.2 i nie są częścią tego etapu.

## 7. R2-5 — przycinanie do limitów

Wartość osi leżąca poza limitem, ale w dopuszczonym marginesie numerycznym
(istniejące 1e-12), jest ustawiana dokładnie na limit. Przycięcie następuje w
fazie C **przed** końcową kontrolą FK, więc zwracana wartość jest tą, która
została zweryfikowana. Wartości poza marginesem są odrzucane jak dziś.

Dotyczy obu backendów i końców przedziału parametru swobodnego. Wartość osi
zależnej w rodzinie osobliwej jest wyliczana z relacji
`offset + multiplier·free`, więc nie da się jej przyciąć bez zmiany relacji — tu
margines 1e-12 zostaje i jest opisany w komentarzu `CoupledFreeParameter`.

## 8. Wpływ na istniejący kod

| Plik | Zmiana |
|---|---|
| `solver/AnalyticIkSolver.hpp/.cpp` | opcje, nowy kod błędu, fazy A/B/C, projekcja, przycinanie |
| `solver/detail/AnalyticIkNumerics.hpp` | licznik obrotów, `nearestRotation` w wersji numerycznej współdzielonej z runtime |
| `codegen/GeneratedIkRuntime.hpp` | to samo co solver ogólny; nowy `Status`, pola `Result`, parametr `maxBranches` |
| `codegen/CppAnalyticIkGenerator.cpp` | bez zmian formatu; `tests/generated/*.hpp` pozostają ważne |
| `model/AnalyticIkSolution.hpp` | `orientationCorrection` |
| `model/TargetValidation.hpp/.cpp` | `nearestRotation`, `kMaxOrientationCorrection`, uzupełniony komentarz kontraktu |
| `symbolic/TrigonometricSimplifier.hpp/.cpp` | uogólnione dopasowanie, pętla do punktu stałego |
| `symbolic/ExpressionCanonicalizer.hpp` | tylko komentarz kontraktu |
| `STATUS.md`, `README.md` | nowe zachowania, zaktualizowana tabela ograniczeń |

Zmiany łamiące zgodność źródłową: nowe wartości dwóch enumów (każdy wyczerpujący
`switch` po nich przestanie się kompilować pod `-Wswitch -Werror` — to pożądane)
oraz nowe pola w strukturach wyniku. Sygnatury `solve()` zyskują parametr z
wartością domyślną.

## 9. Plan testów i bramki

**R2-1**

- ±1000 rad na osiach 4 i 6 oraz ±100 000 rad na osi 6: oba backendy zwracają
  `ResourceLimitExceeded` z `requiredBranches` > budżet, w czasie < 1 s.
- ±100 rad na osiach 4 i 6: 4096 gałęzi przy domyślnym budżecie; z budżetem 4095
  — błąd, nie 4095 gałęzi.
- Budżet podniesiony do wartości z błędu daje pełny wynik, każda gałąź zgodna z FK.
- Fabryczne KR640/KR4: liczba i zawartość gałęzi identyczna jak przed zmianą
  (istniejące testy regresyjne bez modyfikacji oczekiwań).

**R2-2**

- Cele zaokrąglone do 1e-8 (1000 na robota, KR640 i KR4, oba backendy): każdy cel,
  który przeszedł walidację, ma rozwiązanie; `orientationCorrection` ≤
  `kMaxOrientationCorrection`; FK każdej gałęzi zgodne z macierzą zrzutowaną do
  2e-9.
- Cele zaokrąglone do 1e-9 i czyste: wynik jak dziś; korekta < 1e-9, odpowiednio
  < 1e-14.
- `nearestRotation`: wynik spełnia `|RᵀR − I|` < 1e-14 i `det` = 1; idempotencja;
  pomiar maksymalnej korekty na 500 000 losowych macierzy w paśmie akceptacji.

**R2-3**

- Cztery przypadki z tabeli 2.3 upraszczają się w **jednym** przebiegu do: `a − 1`,
  `2`, `a`, `1 − a` (w postaci kanonicznej).
- `2·sin²b + 3·cos²b` oraz para z dzielnikiem w argumencie pozostają bez zmian.
- Test własności na losowych wyrażeniach (generator z `fuzz_sym.cpp`
  przeniesiony do testów, stałe ziarno): 0 przypadków nieidempotencji; zbiór
  dzielników zachowany; wartość zgodna w punktach ogólnych.

**R2-4**

- Dwa testy dokumentujące (6.1, 6.2), opisane jako przypięcie kontraktu.

**R2-5**

- Konfiguracje z osią dokładnie na limicie (3000 losowych + 64 narożniki na
  robota): żadna zwrócona wartość nie leży poza `[lower, upper]`; konfiguracja
  generująca nadal odzyskiwana.

**Bramka etapu:** pełny CTest w Release, Debug i `KINEMAFORGE_WERROR=ON`; liczba
testów i wynik zostaną wpisane do dokumentu implementacyjnego po uruchomieniu,
nie wcześniej.

## 10. Podział dalszej pracy

Dwa dokumenty implementacyjne, niezależne od siebie:

1. `proposal-audit-followup-solver-implementation.md` — R2-1, R2-2, R2-5
   (blokują wydanie).
2. `proposal-audit-followup-symbolic-implementation.md` — R2-3, R2-4.

## 11. Decyzje do podjęcia przez użytkownika

1. **R2-2: wariant A (projekcja z raportowaną korektą) czy B (odrzucenie na
   wejściu solvera)?** Rekomendacja: A. B jest prostsze i nie zmienia kontraktu
   „nie naprawiamy", ale odrzuca dane 8-cyfrowe.
2. **Domyślny budżet gałęzi: 4096?** Mniejsza wartość (np. 1024) wcześniej
   sygnalizuje źle ustawione limity; większa rzadziej wymaga jawnej opcji.
3. **`kMaxOrientationCorrection`: wyznaczyć pomiarem w dokumencie
   implementacyjnym (propozycja 2e-8), czy narzucić wartość teraz?**
4. **R2-3: czy rozkład wieloznaczny (`sin²u·cos²u` jako czynnik wspólny) ma wejść
   teraz, czy wystarczy znak + współczynnik + czynniki niebędące kwadratami
   `sin`/`cos` tego samego argumentu?** Rekomendacja: pełna wersja, bo inaczej
   idempotencja zależy od postaci wejścia.
