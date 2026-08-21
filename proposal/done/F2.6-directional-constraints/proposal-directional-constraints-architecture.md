# Proposal: constrainty kierunkowe (F2.6), architektura

**Etap:** F2.6 · **v4 — APPROVED** · 2026-08-21 · część 1 z 3

> **v4, po trzecim review.** Rdzeń (rodziny, model zadań, DOF, ordering, podział
> a/b) zatwierdzony i nietknięty. Dwie poprawki merytoryczne i dwie kosmetyczne:
>
> 1. **Kto normalizuje.** v3 pisała „normalizowany przy walidacji", a
>    `validate()` bierze `const&` i nie ma jak niczego kanonizować. Kontrakt jest
>    teraz jednoznaczny: **walidacja tylko sprawdza, builder kanonizuje** (7.1,
>    7.5, 9). Cel przechowuje **promień**, nie wektor jednostkowy, i tak jest
>    opisany w typie — inaczej konsument przeczytałby w komentarzu gwarancję,
>    której obiekt nie daje.
> 2. **`hypot` na surowych składowych nie pokrywa deklarowanego kontraktu.** Dla
>    `{DBL_MAX, DBL_MAX, 0}` norma to `inf`, a naiwne dzielenie daje `(0,0,0)` —
>    legalne wejście zniszczone po cichu. Normalizacja jest **skalowana** (7.4),
>    co przy okazji naprawia drugi, gorszy przypadek, którego review nie
>    wymieniło: dla `{denorm_min, denorm_min, 0}` naiwna wersja zwraca `(1,1,0)`,
>    czyli „kierunek jednostkowy" o normie `sqrt(2)`.
> 3. Tabela ryzyk miała dwie pozostałości po v2 (`expected` zamiast `optional`,
>    `1.05e-8` jako próg) — wyczyszczone.
> 4. Dopisany test podrobionego `CartesianComponent` **w źródle kierunku**, nie
>    tylko pozycji (10).
>
> **v3, po drugim review.** Rdzeń F2.6 zatwierdzony i nietknięty. Pięć korekt,
> wszystkie przyjęte:
>
> 1. **Blocker: podział F2.6a/F2.6b zostawiał model niespójny.** v2 dawała do
>    F2.6a wszystkie cztery cele i sześć `IkTaskKind`, ale źródło równania kąta
>    dopiero do F2.6b — czyli istniałby legalny `IkTaskKind::DirectionAngle`, dla
>    którego żaden poprawny system nie da się skonstruować. Gorzej, moje zdanie
>    „dwie pozostałe są odrzucane jawnie przez `SystemRejected`" było **nieprawdą**:
>    bez przeciążenia `build()` taki kod się **nie kompiluje**, więc żaden błąd
>    środowiskowy nie powstaje. Podział przecięty po rodzinach (11).
> 2. `toolAxisColumn` zwraca **`std::optional`**, nie `expected` — `ToolAxis.hpp`
>    nie ma prawa znać żadnego z dwoch modeli błędu, które go używają (3.2).
> 3. Funkcja DOF dostaje kontrakt dla podrobionego `IkTaskKind` — też
>    `std::optional`; **oraz** `Pose` zostaje domknięte do dokładnie dziewięciu
>    równań orientacji, bo inaczej ta funkcja kłamałaby o systemie, który model
>    dopuszcza (6.1, 8.1).
> 4. Próg degeneracji kąta przestaje być częścią kontraktu — odrzucamy najpierw
>    dokładne `0` i `pi`, potem `|cos| == 1`; `1.05e-8` to obserwacja z tej
>    platformy, nie publiczna tolerancja (7.2).
> 5. **Uzasadnienie kolejności w 7.1 było błędne.** Dla kierunku `inf` i `NaN`
>    **nie** wypadają jako `ZeroDirection`, bo `norm == 0` jest dla nich fałszem.
>    Kolejność zostaje, ale z innym — i mocniejszym — powodem (7.1).
>
> **v2, po pierwszym review.** Rdzeń utrzymany i potwierdzony przez review: redukcja pięciu
> nazw do dwóch rodzin (2), `FreeRotationAroundToolAxis` jako własność a nie
> zadanie (2.3), `DirectionEquationSource` osobno od `OrientationEquationSource`
> (5.2), podział na F2.6a i F2.6b (11). Cztery blockery, wszystkie przyjęte:
>
> 1. **Cichy overconstraint w rodzinie II.** v1 miała tylko wariant z pozycją,
>    czyli dokładnie to, przed czym gate F2.6 ma chronić. Argument „sam kąt
>    zostawia 5 DOF, więc to nie zadanie" był **niespójny**, bo `ToolDirection`
>    zostawia 4 i był przyjmowany. Wycofany. Obie rodziny występują teraz z
>    pozycją i bez niej — cztery alternatywy, zero luk (4).
> 2. **Zmiana w `IkEquationSystem` była opisana za wąsko.** Poza kompletnością
>    pozycji w `checkEquationContent` jest jeszcze `checkEquationOrder`, który
>    twardo zakłada „sloty 0..2 to pozycja, reszta to orientacja" i ma własny
>    warunek `size() < 3`. Sekcja 8 przepisana: **pełna macierz dozwolonej
>    zawartości** i **ogólny porządek po kluczu źródła**, bez arytmetyki slotów.
> 3. **`α = 0` i `α = π` nie mają 4 DOF, tylko 5.** Stożek zwija się do prostej,
>    a równanie skalarne staje się singularne. Przyjęty przedział jest teraz
>    **otwarty**, a końce kierowane do rodziny I, która już tę geometrię opisuje.
>    Degeneracja ma próg **zmierzony**, nie wybrany: `cos(α)` jest dokładnie
>    `±1.0` dla `α ≤ 1.05e-8 rad` (7.2).
> 4. **`enum class` nie chroni przed `static_cast<ToolAxis>(99)`.** `-Wswitch`
>    pilnuje dodania enumeratora, nie wartości podrobionej — a model broni się
>    przed dokładnie tym przy `CartesianComponent`. Dochodzi walidacja osi w obu
>    warstwach oraz reguła, że wszystkie równania jednego systemu dotyczą **tej
>    samej** osi (3.2, 8.2).
>
> Poza tym: kierunek nie dostaje progu normy — przyjmujemy każdy niezerowy wektor
> i normalizujemy (7.1, odejście od analogii do kwaternionu, uzasadnione);
> nazwy bez słowa „surface" i bez „axis-angle" (5.1); `AxisAlignment` zdefiniowany
> **ze znakiem** (2.5); `taskSpaceConstrainedDegreesOfFreedom` zamiast krótszej
> nazwy, żeby nie mylić z rangą jakobianu (6); rachunek kodów błędu poprawiony
> (7.4).

Poprzednik: [`../done/F2.5Q-quaternion-target/proposal-quaternion-target-architecture.md`](../F2.5Q-quaternion-target/proposal-quaternion-target-architecture.md)
(F2.5Q, zaimplementowane). Tamten etap zamknął pytanie **czym jest cel**; ten
otwiera jedyne pytanie, którego model z F2.3 świadomie nie tknął: **co jeszcze
można ograniczyć, nie narzucając pełnej pozy**.

## Prompt

> Roadmapa, F2.6 — obsłużyć zadania istotne dla narzędzi technologicznych:
>
> ```text
> FixedToolDirection
> PositionAndAxis
> AxisAlignment
> FreeRotationAroundToolAxis
> FixedAngleToSurfaceNormal
> ```
>
> ```text
> Z_tcp(q) = desired_direction
> dot(Z_tcp(q), surface_normal) = cos(required_angle)
> ```
>
> Przy swobodnym obrocie wokół osi dyszy nie wolno przypadkiem narzucić pełnej
> orientacji.
>
> Gate: każdy constraint ma jednoznaczną semantykę, policzalną liczbę stopni
> swobody, testy odróżniające go od `FullPose` i brak nadmiarowych ograniczeń
> narzucanych przez przypadek.

---

## 0. Stan wyjściowy

```text
IkTarget          variant<PositionTarget, PoseTarget>            F2.3, zbiór zamknięty
IkTaskKind        { Position, Pose }                             F2.3
EquationSource    variant<PositionEquationSource,
                          OrientationEquationSource>             F2.3
ConstraintBuilder dwa przeciążenia, 3 i 12 równań                F2.4 / F2.5
wejście orientacji  macierz albo kwaternion, jedne drzwi         F2.5Q
```

365/365 testów w obu konfiguracjach. Model celu jest **domknięty**: F2.5Q był
ostatnim etapem, który dokładał *reprezentację*. F2.6 dokłada **rodzaje zadań** —
różnica gatunkowa, nie stopnia. Dlatego tym razem `IkTarget`, `IkTaskKind` i
`EquationSource` **muszą** urosnąć, mimo że F2.5Q chwalił się, że nie urosły:
kwaternion opisywał to samo ograniczenie innym zapisem, a kierunek narzędzia
opisuje **inne ograniczenie**.

---

## 1. Cel i zakres

**Cel:** umieć zbudować układ równań dla zadań, w których orientacja narzędzia
jest ograniczona **częściowo** — bo tak wygląda większość realnych procesów:
spawanie, natrysk, klejenie, cięcie. Narzędziu każe się patrzeć w określoną
stronę, a obrót wokół własnej osi zostaje wolny, bo dysza jest osiowo
symetryczna.

**W zakresie:**

```text
równanie kierunku osi narzędzia        R(q)·e_t = d
równanie kąta do kierunku odniesienia  dot(R(q)·e_t, n) = cos(alfa)
cztery nowe alternatywy IkTarget i cztery nowe IkTaskKind
dwa nowe źródła równań
przebudowa reguł zawartości i porządku w IkEquationSystem
jawna, policzalna liczba stopni swobody na rodzaj zadania
```

**Poza zakresem, świadomie:**

```text
oś narzędzia inna niż główna (X, Y, Z ramki końcowej)  -> 3.1, rozwiązywane przez TCP
ograniczenia nierównościowe (kąt <= alfa, stożek)      -> to nie jest równanie
ograniczenia na pozycję inne niż punkt (linia, płaszczyzna)  -> osobny etap
kompozycje dowolnych ograniczeń w locie                -> 4.3
rozwiązywanie i rangowanie układu                      -> Faza 3
integracja z fasadą                                    -> F2.7
```

---

## 2. Pięć nazw z roadmapy to **dwie** rodziny równań

Najważniejsza sekcja tego dokumentu. Roadmapa wymienia pięć nazw; po rozpisaniu
ich na równania zostają **dwie** rodziny, a pozostałe trzy nazwy to kompozycja,
synonim i własność.

### 2.1 Rodzina I — równość kierunku

```text
R(q)·e_t = d          e_t: oś narzędzia w ramce końcowej (jednostkowa)
                      d:   żądany kierunek w układzie bazy (jednostkowy)
```

Dla osi głównej `e_t = e_z` lewa strona to **trzecia kolumna bloku rotacji**,
czyli komórki `T(0,2)`, `T(1,2)`, `T(2,2)` — a więc, tak jak w F2.4 i F2.5,
**surowe komórki FK**, bez budowania nowego wyrażenia.

### 2.2 Rodzina II — kąt do kierunku odniesienia

```text
dot(R(q)·e_t, n) = cos(alfa)      n: kierunek odniesienia (niezerowy)
                                  alfa: wymagany kąt, przedział OTWARTY (7.2)
```

Jedno równanie skalarne. Nazwa typu **nie** mówi „normalna powierzchni": model
nie wie i nie ma jak wiedzieć, że `n` przyszło z normalnej. To jest kąt między
osią narzędzia a zadanym kierunkiem, a normalna powierzchni jest jednym z jego
źródeł — i dlatego typy nazywają się `DirectionAngle...`, a nie `SurfaceAngle...`
(v1 miała tu sprzeczność z własnym argumentem).

Ta rodzina jest pierwsza w całym projekcie, w której **lewa strona równania nie
jest komórką macierzy**, tylko wyrażeniem liczonym:

```text
lhs = n_x·T(0,2) + n_y·T(1,2) + n_z·T(2,2)
```

I to jest dokładnie ta zmiana, na którą F2.5 się przygotowało, przenosząc
kolektor symboli z komórek transformacji na **równania**. Gwarancja „każdy symbol
w równaniach jest zadeklarowaną niewiadomą" działa tu bez jednej linijki zmiany.

### 2.3 `FreeRotationAroundToolAxis` **nie jest ograniczeniem**

To własność rodziny I, nie osobne zadanie. `R(q)·e_t = d` przybija dwa stopnie
swobody obrotu i **z definicji zostawia trzeci wolny** — obrót wokół osi
narzędzia. Nie ma czego dodawać do układu; przeciwnie, dodanie czegokolwiek by to
zepsuło.

Osobna alternatywa `IkTarget` dałaby dwa różne `IkTaskKind` produkujące
**identyczny** układ równań, a solver z Fazy 3 musiałby zgadywać, czy różnica coś
znaczy. Zapis roadmapy „nie wolno przypadkiem narzucić pełnej orientacji" jest
więc **warunkiem testowym**, nie typem (10).

### 2.4 `PositionAndAxis` to kompozycja

Pozycja (3 równania z F2.4) plus rodzina I (3 równania). Nic nowego poza
sklejeniem — ale sklejenie musi być **nazwane**, patrz 4.

### 2.5 `AxisAlignment` — ze znakiem, i to trzeba powiedzieć wprost

Redukcja `AxisAlignment` do rodziny I jest poprawna **tylko** przy jednej
interpretacji, więc zapisujemy ją jako część kontraktu:

> **W F2.6 „alignment" znaczy zgodność z kierunkiem zorientowanym.** `R(q)·e_t = d`
> ustawia oś narzędzia **w stronę** `d`. Ustawienie przeciwne to inny cel:
> `-d`, nie ten sam.

Gdyby „alignment" miało znaczyć równoległość bez rozróżnienia `d` i `−d`, byłoby
to **trzecie** ograniczenie o innej geometrii (dwa rozłączne rozwiązania,
`|dot(u, d)| = 1`) — i nie należałoby do rodziny I. Nierozstrzygnięte, ta
redukcja byłaby cichym założeniem; rozstrzygnięte, jest decyzją z testem
(`AntiParallelDirectionIsADifferentTarget`).

### Podsumowanie

| Nazwa z roadmapy | Czym jest w tym proposalu |
|---|---|
| `FixedToolDirection` | rodzina I, bez pozycji |
| `AxisAlignment` | rodzina I — ta sama, kierunek zorientowany (2.5) |
| `PositionAndAxis` | pozycja + rodzina I |
| `FreeRotationAroundToolAxis` | **własność** rodziny I, sprawdzana testem |
| `FixedAngleToSurfaceNormal` | rodzina II — z pozycją albo bez (4.1) |

---

## 3. Oś narzędzia

### 3.1 Tylko osie główne

```cpp
enum class ToolAxis { X, Y, Z };
```

Domyślnie `Z`. Oś **dowolna** (np. dysza odchylona o 15°) jest poza zakresem i to
nie jest ucieczka, tylko wskazanie miejsca, w którym problem już jest
rozwiązany: **`FixedRigidTransform` z F2.1/F2.2**. Narzędzie odchylone opisuje się
obróconym TCP, a nie obróconą osią w celu zadania — i wtedy w ramce TCP oś znowu
jest osią główną.

Trzy skutki, wszystkie dobre:

1. lewa strona rodziny I zostaje **surową komórką FK**, więc `sameNode` dalej
   działa jako narzędzie testowe;
2. nie powstaje drugie miejsce, w którym koduje się geometrię narzędzia — a dwa
   takie miejsca prędzej czy później zaczęłyby się różnić;
3. wartości są trzy i da się je wyczerpać jawnym `switch`.

### 3.2 Oś musi być walidowana w czasie działania

`enum class` **nie** wystarcza i v1 twierdziła inaczej. `-Wswitch` pilnuje
dodania enumeratora przy kompilacji, ale nie chroni przed
`static_cast<ToolAxis>(99)` — a model broni się przed dokładnie tym przypadkiem
dla `CartesianComponent`, komentarzem, który mówi to wprost:

```cpp
// ...forged with static_cast<CartesianComponent>(99) is a valid object of the
// enum type but has no enumerator, and indexing with it writes out of bounds.
```

Obniżenie tego standardu dla `ToolAxis` byłoby regresem, więc:

```cpp
// Zamknięte przejście oś -> kolumna bloku rotacji. Jedyne miejsce, w którym
// ToolAxis zamienia się w indeks; wartość spoza enumeratorów daje nullopt,
// nie indeksuje.
[[nodiscard]] std::optional<std::size_t> toolAxisColumn(ToolAxis axis) noexcept;
```

**`optional`, nie `expected`** — v2 miała tu `expected`, co jest błędem warstwy.
Ta funkcja żyje w `ToolAxis.hpp`, a błąd musi być mapowany **inaczej w dwóch
miejscach**: na `TargetValidationErrorCode::InvalidToolAxis` przy walidacji celu i
na `IkEquationSystemErrorCode::InvalidToolAxis` przy walidacji systemu. Gdyby
`toolAxisColumn` zwracalo `expected`, musiałoby wybrać jeden z tych słowników —
i `ToolAxis.hpp` przestałby być neutralnym typem modelu, a stałby się zależny od
walidacji celu **albo** od modelu układu równań. `nullopt` nie niesie żadnego
słownika, więc każda warstwa tłumaczy go na swój.

Walidacja jest potrzebna w **dwóch** warstwach, bo oś pojawia się w dwóch
miejscach: w celu (`TargetValidation`, kod `InvalidToolAxis`) i w źródle równania
(`IkEquationSystem`, patrz 8.2). Dodatkowo obowiązuje reguła spójności: **wszystkie
równania jednego systemu dotyczą tej samej osi**. Bez niej „X, Y, Z obecne"
spełniają trzy równania mówiące o trzech różnych osiach narzędzia — układ
wewnętrznie sprzeczny, który przeszedłby walidację kompletności.

### 3.3 Czyja to oś

Precyzyjnie, bo to jest ta sama klasa niejednoznaczności co znany brak w
`IkTarget` („cel nie zna swojej ramki"):

> `ToolAxis` nazywa oś **ramki końcowej przekazanego `SymbolicTransform`**. Jeśli
> transformacja kończy się na TCP — czyli jest wynikiem `tcpForwardKinematics()`
> — jest to oś TCP. Jeśli kończy się na końcu łańcucha, jest to oś ostatniego
> ogniwa. `ConstraintBuilder` nie wie, którą dostał, i nie może wiedzieć.

Oś zostaje **w celu**, nie w fasadzie: `ConstraintBuilder` jest używalny bez
fasady i ma mieć komplet semantyki. F2.7 może dobierać oś domyślną z konfiguracji
narzędzia, ale własność core'owego celu nie przenosi się do stanu fasady.

---

## 4. Model celu

### 4.1 Cztery nowe alternatywy `IkTarget` — obie rodziny, z pozycją i bez

```cpp
// direction: skończony i niezerowy. NIE musi być jednostkowy i typ tego nie
// obiecuje -- długość nie niesie żadnej semantyki, bo kierunek jest promieniem.
// Kanonizacja należy do buildera, nie do celu i nie do walidacji (7.5).
struct ToolDirectionTarget
{
    Vector3 direction;          // w bazie
    ToolAxis axis{ToolAxis::Z};
};

struct PositionAndToolDirectionTarget
{
    Vector3 position;           // metry
    Vector3 direction;          // skończony, niezerowy; długość bez znaczenia
    ToolAxis axis{ToolAxis::Z};
};

struct DirectionAngleTarget
{
    Vector3 reference;          // skończony, niezerowy; długość bez znaczenia
    double angle{};             // radiany, przedział otwarty (7.2)
    ToolAxis axis{ToolAxis::Z};
};

struct PositionAndDirectionAngleTarget
{
    Vector3 position;
    Vector3 reference;          // skończony, niezerowy; długość bez znaczenia
    double angle{};
    ToolAxis axis{ToolAxis::Z};
};

using IkTarget = std::variant<PositionTarget,
                              PoseTarget,
                              ToolDirectionTarget,
                              PositionAndToolDirectionTarget,
                              DirectionAngleTarget,
                              PositionAndDirectionAngleTarget>;
```

**Symetria jest regułą, nie ozdobą.** v1 miała rodzinę II wyłącznie z pozycją i
uzasadniała to tym, że sam kąt zostawia pięć stopni swobody, więc „to nie
zadanie". Argument jest niespójny: `ToolDirectionTarget` zostawia cztery i jest
przyjmowany. **Liczba wolnych DOF nie może być kryterium tego, czy coś jest
zadaniem** — inaczej `PositionTarget` (zostawia trzy) też by nim nie był. Co
gorsza, wersja z v1 dokładała trzy równania pozycji do ograniczenia, które
roadmapa zapisała jako jedno równanie skalarne: to jest przypadkowy
overconstraint, czyli dokładnie to, przed czym gate F2.6 ma chronić.

Rodzina bez pozycji ma zresztą konkretnego konsumenta w Fazie 3: podproblem
nadgarstka (F3.9) jest zadaniem orientacyjnym bez pozycji.

### 4.2 Czego tu nie ma

**Nie ma `std::optional<Vector3> position` w jednym typie na rodzinę.** Precedens
F2.3 jest wiążący: „struktura z optional dopuściłaby cel, który nie ogranicza
niczego". Optional zamieniłby też rodzaj zadania w wartość runtime'ową, a
`IkTaskKind` ma klasyfikować, nie relacjonować.

### 4.3 Dlaczego nie lista dowolnych ograniczeń

Kuszące i **odrzucone**:

```cpp
struct IkTask { std::vector<Constraint> constraints; };   // NIE
```

1. dopuszcza kombinacje bez sensu — pozycję dwa razy, pełną pozę plus kierunek
   (nadmiarowe), kąt do osi, której nikt nie ograniczył;
2. reguły zawartości (8.1) musiałyby zniknąć albo stać się arbitralne;
3. **solver z Fazy 3 rozpoznaje strukturę po rodzaju zadania.** Lista przenosi tę
   robotę na solver, i to w najgorszym momencie — po tym, jak informacja o
   zamiarze caller'a już wyparowała.

---

## 5. Równania i ich źródła

### 5.1 `EquationSource` rośnie o dwie alternatywy

```cpp
struct DirectionEquationSource
{
    CartesianComponent component{};   // składowa kierunku w bazie
    ToolAxis axis{ToolAxis::Z};       // której osi narzędzia dotyczy

    friend bool operator==(const DirectionEquationSource&,
                           const DirectionEquationSource&) noexcept = default;
};

// Jedno równanie skalarne — nie ma czego indeksować. Nazwa celowo bez słowa
// "axis-angle": tamto jest reprezentacją obrotu i pomyłka byłaby kosztowna.
struct DirectionAngleEquationSource
{
    ToolAxis axis{ToolAxis::Z};

    friend bool operator==(const DirectionAngleEquationSource&,
                           const DirectionAngleEquationSource&) noexcept = default;
};
```

`EquationKind` dostaje `Direction` i `DirectionAngle`, a
`detail::EquationKindVisitor` — jawny zestaw przeciążeń — **przestanie się
kompilować**, dopóki ktoś nie dopisze obu. Dokładnie to zachowanie, dla którego
F2.3 wybrało wariant zamiast pary `{kind, optional<cell>}`.

### 5.2 Dlaczego osobne źródło, skoro dla osi Z to te same komórki

Dla `ToolAxis::Z` równanie kierunku dotyczy komórek `(0,2)`, `(1,2)`, `(2,2)` —
czyli komórek, które `OrientationEquationSource` też potrafi nazwać. Kusi, żeby
nie dodawać typu.

**Nie.** Źródło mówi, **co jest ograniczane**, a nie **którą komórkę przypadkiem
odczytano**. Trzy równania orientacji na kolumnie Z w zadaniu `Pose` są fragmentem
ograniczenia o dziewięciu komórkach; te same trzy komórki w zadaniu
`ToolDirection` są **całym** ograniczeniem, o innej liczbie stopni swobody i
innym sposobie rozwiązywania. Solver, który tego nie odróżni, policzy DOF błędnie.
Dodatkowo dla osi innej niż główna (3.1, kiedyś) lewa strona przestanie być
komórką, a źródło ma pozostać tym samym pojęciem.

---

## 6. Liczba równań a liczba stopni swobody

| `IkTaskKind` | równań | DOF odbierane | co zostaje wolne |
|---|---|---|---|
| `Position` | 3 | 3 | cała orientacja |
| `Pose` | 12 | 6 | nic |
| `ToolDirection` | 3 | **2** | pozycja (3) + obrót wokół osi narzędzia |
| `PositionAndToolDirection` | 6 | **5** | obrót wokół osi narzędzia |
| `DirectionAngle` | 1 | **1** | pozycja (3) + stożek (1) + obrót wokół osi |
| `PositionAndDirectionAngle` | 4 | **4** | stożek (1) + obrót wokół osi narzędzia |

Dwa wiersze wymagają rachunku.

**`ToolDirection`: trzy równania, dwa stopnie swobody.** Kierunek jednostkowy w
`R³` ma dwa stopnie swobody, bo leży na sferze. Trzecie równanie jest zależne na
zbiorze rozwiązań: `|R(q)·e_t| = 1` zachodzi tożsamościowo dla sztywnego FK, a
`|d| = 1` jest wymuszone **kanonizacją w `ConstraintBuilder`** (7.4, 7.5) — nie
walidacją, która niczego nie zmienia. Ta sama lekcja, co
dziewięć równań na trzy DOF w F2.5, w przypadku na tyle małym, że mieści się w
jednym zdaniu — i dlatego właśnie **tu** trafia do dokumentacji typu.

**`DirectionAngle`: jedno równanie, jeden stopień swobody** — ale **wyłącznie**
przy kącie z przedziału otwartego. Dla `alfa = 0` lub `alfa = pi` zbiór
`u·n = ±1` nad wektorami jednostkowymi zwija się do `u = n` albo `u = −n`, czyli
ograniczenie odbiera **dwa** stopnie swobody, nie jeden, a równanie robi się
singularne (pochodna po orientacji znika w rozwiązaniu). Dlatego końce przedziału
są **odrzucane** i kierowane do rodziny I (7.2) — i dopiero to sprawia, że tabela
wyżej jest prawdziwa dla każdego dopuszczonego celu.

### 6.1 `taskSpaceConstrainedDegreesOfFreedom(IkTaskKind)`

```cpp
[[nodiscard]] std::optional<std::size_t>
taskSpaceConstrainedDegreesOfFreedom(IkTaskKind kind) noexcept;
```

**`optional`, bo `static_cast<IkTaskKind>(99)` jest tym samym problemem, co
`static_cast<ToolAxis>(99)`** — a model już dziś się przed nim broni w
`IkEquationSystem`. Przy zwykłym `std::size_t` wszystkie wyjścia są złe: `0`
wygląda jak legalne „zero DOF", `assert` znika w Release, a `throw` kłóci się z
`noexcept`. `nullopt` znaczy dokładnie tyle, ile trzeba: to nie jest żaden ze
znanych rodzajów zadania.

Nazwa jest długa celowo: mówi o **przestrzeni zadania**, a nie o randze jakobianu
robota — te dwie liczby bywają różne i mylenie ich byłoby drogie. Funkcja jest
uczciwa, bo liczba DOF odbieranych przez rodzaj zadania jest własnością geometrii
ograniczenia i nie zależy ani od robota, ani od konfiguracji. **Rangi układu ta
funkcja nie zwraca i nie może** — to zostaje przy solverze, tak jak zapisano w
`IkEquationSystem`.

Zależność wyłącznie od `IkTaskKind` (a nie od celu) jest legalna **dzięki**
rozstrzygnięciu z 7.2: gdyby końce przedziału były dopuszczone, ta sama wartość
`IkTaskKind` miałaby raz 4, raz 5 DOF i funkcja musiałaby przyjmować cel.

### 6.2 `Pose` musi wreszcie znaczyć pełną pozę

Wiersz `Pose -> 6 DOF` jest prawdziwy tylko wtedy, gdy orientacja jest
**kompletna**. Tymczasem model dopuszcza dziś dla `Pose` „XYZ + co najmniej jedno
równanie orientacji" — kontrakt z F2.3, świadomie wtedy luźny, bo liczba równań
rotacji była decyzją F2.5. Po dodaniu **publicznej funkcji DOF** ta luźność
przestaje być niewinna: dla legalnie skonstruowanego systemu `Pose` z jednym
równaniem orientacji funkcja zwróciłaby 6, a ograniczone byłyby cztery stopnie
swobody. Funkcja kłamałaby o czymś, co model dopuszcza — a to jest gorsze niż
brak funkcji.

Dlatego F2.6a domyka kontrakt zgodnie z decyzją, która i tak już zapadła w F2.5:

```text
Pose -> dokładnie dziewięć równań orientacji, komplet komórek (0,0)..(2,2)
```

Komplet wynika z liczby plus istniejącej reguły o duplikatach: dziewięć różnych
źródeł spośród dziewięciu możliwych to wszystkie.

**Koszt jest realny i policzalny**, więc podaję go wprost: `ConstraintBuilder`
produkuje dziewięć od F2.5, więc **kod produkcyjny nie zmienia się wcale**, ale
dwa istniejące testy modelu budują `Pose` z dwoma równaniami orientacji, żeby
sprawdzić duplikat źródła i zły porządek. Po zaostrzeniu złapią najpierw
`TaskEquationMismatch`, więc ich fixture'y muszą urosnąć do kompletu dziewięciu z
jednym duplikatem albo jedną parą przestawioną. **Intencja testów zostaje, zmienia
się rusztowanie** — i to jest jedyne miejsce w F2.6, w którym istniejące testy są
dotykane.

Alternatywa — zostawić luźny kontrakt i opisać funkcję DOF jako „nominalną
semantykę rodzaju zadania" — jest odrzucona: skoro częściowa orientacja ma od
teraz **własne rodzaje zadań**, `Pose` nie ma już powodu znaczyć czegokolwiek
innego niż pełna poza.

---

## 7. Walidacja nowych celów

### 7.1 Kierunek: niezerowy i normalizowany, **bez** progu normy

v1 przenosiła tu wprost schemat z F2.5Q (wąskie pasmo wokół 1 + normalizacja).
**To był błąd** i review słusznie go wytknęło: v1 sama pisze, że kierunek jest
promieniem, a potem odrzuca `(0,0,5)` za to, że nie ma normy 1. Kwaternion
jednostkowy jest **deklaracją** caller'a — obiecuje, że to jest obrót, więc norma
1.7 dowodzi błędu. Kierunku nikt jednostkowym nie deklaruje: normalna powierzchni
z iloczynu wektorowego ma długość zupełnie dowolną i jest całkowicie poprawnym
wejściem.

```text
1. wszystkie składowe skończone     -> NonFiniteDirection
2. wektor niezerowy                 -> ZeroDirection
```

Krok 2 sprawdza się na **największej co do modułu składowej**, nie na normie:
wektor jest zerowy dokładnie wtedy, gdy `max(|x|, |y|, |z|) == 0`. To jest test
bez żadnej arytmetyki pośredniej, więc nie ma tu jak wystąpić nadmiar ani
niedomiar — a ta sama wartość jest potem użyta jako skala przy kanonizacji (7.4).

**Walidacja normalizacji nie wykonuje**, i to jest poprawka z v4 — patrz 7.5.

Kolejność 1 przed 2 zostaje, ale **nie z tego powodu, który podała v2**.
Twierdzenie, że odwrócenie dałoby `ZeroDirection` dla `inf` albo `NaN`, jest
nieprawdziwe: porównanie z zerem jest dla obu **fałszem**, więc żadne z nich nie
zostałoby zaklasyfikowane jako kierunek zerowy. Analogia do progu normy
kwaternionu — gdzie porównanie `> tolerancja` naprawdę zmieniało kod błędu — tu
nie zachodzi.

Powód jest inny i prostszy: **wszystko, co następuje po kroku 1, zakłada liczby.**
`max(|x|, |y|, |z|)` z `NaN` w składowych daje wynik zależny od kolejności
argumentów, bo każde porównanie z `NaN` jest fałszem — a to ta sama wartość, która
zaraz posłuży za skalę przy kanonizacji (7.4). Dzielenie przez skalę wziętą z
`NaN` nie ma sensu, więc krok 1 nie jest kwestią kodu błędu, tylko warunkiem
wstępnym całej reszty.

Na marginesie, jako argument za tym, żeby na wartościach niefinitowych nie
polegać nigdzie: `std::hypot(inf, 0, 0)` zwraca na tym toolchainie **`NaN`**, choć
C99 (F.10.4.3) każe `+inf` nawet dla `y = NaN`. Nas to nie dotyczy, bo do `hypot`
docierają wyłącznie wartości skończone i przeskalowane — ale dotknęłoby każdego,
kto spróbuje klasyfikować wejście po normie.

Próg „bliskie zeru" **nie** istnieje: jedyną wartością, przy której normalizacja
nie ma sensu, jest dokładne zero.

### 7.2 Kąt: przedział otwarty, z progiem zmierzonym

```text
1. alfa skończone                       -> NonFiniteAngle
2. 0 <= alfa <= pi                      -> AngleOutOfRange
3. alfa != 0 oraz alfa != pi            -> DegenerateAngle   (matematyka)
4. |cos(alfa)| != 1                     -> DegenerateAngle   (arytmetyka)
```

Kroki 3 i 4 są rozdzielone celowo i to jest poprawka z v3. Krok 3 odrzuca **końce
matematyczne**, niezależnie od implementacji `std::cos` — jest przenośny i nie
opiera się na niczyjej bibliotece. Krok 4 dobiera to, co numerycznie **zapadło
się** do końców, i jego szerokość może się różnić między platformami.

Krok 2 jest konieczny, bo do równania wchodzi `cos(alfa)`: `alfa = 3pi` byłoby
przyjęte i po cichu znaczyło `pi`. Caller dostałby inny kąt, niż zamówił — klasa
błędu, którą projekt konsekwentnie odrzuca zamiast naprawiać.

Kroki 3 i 4 realizują rozstrzygnięcie z 6: końce przedziału to **nie jest**
rodzina II. Krok 4 bada wielkość, która naprawdę wchodzi do równania jako prawa
strona, a jego szerokość na tym toolchainie zmierzyłem:

```text
cos(alfa) == 1.0  dokładnie dla alfa      <= 1.05e-8 rad   (6.0e-7 stopnia)
cos(alfa) == -1.0 dokładnie dla pi - alfa <= 1.05e-8 rad
```

**To jest obserwacja, nie kontrakt.** `1.05e-8` nie trafia do żadnej stałej, do
dokumentacji publicznej ani do testu z konkretną wartością progu — zależy od
implementacji `std::cos` i na innej platformie wyjdzie inne. Kontraktem jest samo
zdanie: **kąt, dla którego policzony cosinus jest nieodróżnialny od `±1`, nie
należy do rodziny II.**

Poniżej tego kąta stała po prawej stronie **jest** jedynką, więc równanie
dosłownie brzmi `u·n = 1` — geometria zwinięta, dwa DOF zamiast jednego,
singularna pochodna.

Komunikat błędu nie może mówić „kąt 0 albo pi", bo odrzucone bywa też `1e-9`:

> `DegenerateAngle`: angle is numerically degenerate — cos(angle) is
> indistinguishable from +/-1; use ToolDirectionTarget with direction =
> +/- reference.

Kąty **niewiele** większe od progu są przyjmowane i to jest świadome: `alfa = 1e-6`
to stożek o promieniu 1 µm na metr — geometrycznie poprawny, numerycznie
paskudny. Czułość równania skaluje się jak `sin(alfa)` (zmierzone: `sin(1e-6) = 1e-6`),
więc **uwarunkowanie** jest problemem solvera z Fazy 3, natomiast **degeneracja**
jest nasza i tu się kończy.

### 7.3 Oś narzędzia w celu

```text
axis jest jednym z X, Y, Z          -> InvalidToolAxis
```

Sprawdzane przez `toolAxisColumn` (3.2), nie przez rzutowanie na indeks.

### 7.4 Kanonizacja: skala, potem `hypot`, potem dzielenie

Deklarowany kontrakt brzmi: **każdy skończony, niezerowy wektor jest legalnym
kierunkiem**. v3 chciała go zrealizować przez `std::hypot(x, y, z)` na surowych
składowych i to **nie wystarcza** — review ma rację. Zmierzone:

```text
wejście                       hypot(x,y,z)   naiwne x/norm            skalowane
{DBL_MAX, DBL_MAX, 0}         inf            (0, 0, 0)                (0.707107, 0.707107, 0)
{denorm_min, denorm_min, 0}   4.94e-324      (1, 1, 0)                (0.707107, 0.707107, 0)
{1e200, 1e200, 0}             1.41e+200      (0.7071, 0.7071, 0)      (0.707107, 0.707107, 0)
```

Pierwszy wiersz to przypadek z review: prawdziwa długość przekracza `DBL_MAX`,
`hypot` daje `inf`, a dzielenie **cicho niszczy** kierunek jednoznaczny co do
ostatniego bitu. Drugi wiersz znalazł się przy sprawdzaniu i jest **gorszy**:
`hypot` niedomiarowuje do `denorm_min`, dzielenie daje `(1, 1, 0)`, czyli wynik,
który **udaje kierunek jednostkowy**, mając normę `sqrt(2)`. Pierwszy błąd jest
widoczny, drugi przechodzi dalej i psuje równania.

Kanonizacja jest więc skalowana:

```text
scale = max(|x|, |y|, |z|)          po walidacji: scale w (0, DBL_MAX]
s     = (x, y, z) / scale           s w [-1, 1]^3, |s|max = 1
norm  = hypot(s)                    zmierzone: norm w [1.000000, 1.732051]
wynik = s / norm
```

Zakres normy po przeskalowaniu jest zmierzony i wynosi dokładnie `[1, sqrt(3)]`,
więc **ani nadmiar, ani niedomiar nie ma gdzie wystąpić** — niezależnie od tego,
jak ekstremalne było wejście. `scale` jest przy tym tą samą wielkością, którą
walidacja porównuje z zerem (7.1), więc dzielenie przez nią jest bezpieczne z
konstrukcji.

Jedno ograniczenie zapisane wprost: dla wejścia **subnormalnego** iloraz
`s = v/scale` traci część cyfr znaczących, bo tyle ich w subnormalu było.
Kierunek jest odtwarzany z tą precyzją, którą wejście faktycznie niosło — i
żadna metoda nie da więcej.

### 7.5 Kto normalizuje: **builder, nie walidacja**

v3 pisała w kilku miejscach „normalizowany przy walidacji", co jest sprzeczne z
kontraktem `TargetValidation` obowiązującym w repo:

```cpp
std::expected<void, TargetValidationError> validate(const PoseTarget& target);
```

`validate()` dostaje `const&` i zwraca `void` — **nie ma jak** niczego
kanonizować, i nie powinno mieć: jego zadaniem jest orzekanie, nie
przekształcanie. Podział jest więc taki:

```text
TargetValidation     skończoność, niezerowość, oś, kąt          -> tylko orzeka
ConstraintBuilder    normalizedDirection(...) przed emiterami  -> kanonizuje
```

Konsekwencja dla modelu jest ta, która naprawdę się liczy: **cel przechowuje
promień, nie wektor jednostkowy**, i komentarz przy polu mówi dokładnie to (4.1).
Gdyby typ obiecywał „normalized", przyszły konsument — solver, fasada, generator
kodu — czytałby gwarancję, której obiekt nie daje, bo nikt po drodze nie ma
prawa go zmienić.

Powrót do wzorca z F2.5Q (funkcja wytwórcza zwracająca skanonizowany cel) jest
tu **niepotrzebny i odrzucony**: tam kanonizacja była konieczna, bo produkt
musiał przejść `validate(PoseTarget)` później, u kogoś innego. Tu nikt poza
builderem tego kierunku nie ogląda, więc jedna funkcja prywatna w budowniczym
wystarcza i nie dokłada trzeciego typu celu.

### 7.6 Rachunek nowych kodów błędu

v1 mówiła o „trzech nowych kodach" i wymieniała cztery. Poprawnie:

| Warstwa | Nowe kody |
|---|---|
| `TargetValidationErrorCode` | `NonFiniteDirection`, `ZeroDirection`, `NonFiniteAngle`, `AngleOutOfRange`, `DegenerateAngle`, `InvalidToolAxis` — **sześć** |
| `IkEquationSystemErrorCode` | `InvalidToolAxis`, `MixedToolAxes` — **dwa** (8.2) |

`ConstraintBuilderError` **nie** dostaje nowych kodów: `InvalidTarget` niesie
`TargetValidationError`, a `SystemRejected` — `IkEquationSystemError`. Ta sama
decyzja i to samo uzasadnienie co w F2.5Q: jeden słownik błędów na warstwę.

---

## 8. `IkEquationSystem` — pełna przebudowa reguł zawartości i porządku

v1 opisywała tę zmianę jako „jedną regułę kompletności pozycji". **To było za
wąsko** i review to wychwyciło. W aktualnym kodzie są dwie niezależne funkcje i
obie zakładają świat, w którym istnieją tylko `Position` i `Pose`.

### 8.1 Macierz dozwolonej zawartości

`checkEquationContent` liczy dziś sloty pozycji i równania orientacji, a potem
żąda kompletu pozycji **w każdym zadaniu**. Nowa reguła jest tabelą, nie zdaniem:

| `IkTaskKind` | pozycja | orientacja | kierunek | kąt |
|---|---|---|---|---|
| `Position` | dokładnie X, Y, Z | 0 | 0 | 0 |
| `Pose` | dokładnie X, Y, Z | **dokładnie 9, komplet** (6.2) | 0 | 0 |
| `ToolDirection` | **0** | 0 | dokładnie X, Y, Z | 0 |
| `PositionAndToolDirection` | dokładnie X, Y, Z | 0 | dokładnie X, Y, Z | 0 |
| `DirectionAngle` | **0** | 0 | 0 | dokładnie 1 |
| `PositionAndDirectionAngle` | dokładnie X, Y, Z | 0 | 0 | dokładnie 1 |

Bez pełnej tabeli, a z samą poprawką kompletności pozycji, `Position` mogłoby
nieść równanie kierunku, a `Pose` — równanie kąta. Zera w tej tabeli są równie
ważne jak komplety.

Komentarz przy dotychczasowej pętli zostaje prawdziwy w nowym brzmieniu: zadanie
wiążące dwie z trzech współrzędnych nadal byłoby **innym** `IkTaskKind`, a nie
okrojonym Pose.

### 8.2 Spójność osi narzędzia

Dwie reguły ponad tabelą, obie sprawdzane w `checkEquationContent`:

```text
każde źródło kierunku i kąta ma oś z enumeratorów      -> InvalidToolAxis
wszystkie takie źródła w jednym systemie mają TĘ SAMĄ oś -> MixedToolAxes
```

Druga jest tą, której sama tabela nie daje: trzy równania kierunku o składowych
X, Y, Z, ale o osiach narzędzia odpowiednio X, Y i Z, spełniają „dokładnie X, Y, Z"
i są układem wewnętrznie sprzecznym.

### 8.3 Porządek: jeden klucz zamiast arytmetyki slotów

`checkEquationOrder` w obecnej postaci:

```cpp
if (equations.size() < 3) ...                       // twardy warunek
for (slot = 0..2)  wymaga PositionEquationSource     // sloty na sztywno
for (slot = 3..N)  wymaga OrientationEquationSource  // reszta to orientacja
```

Dla `ToolDirection` (trzy równania kierunku, zero pozycji) odrzucenie następuje
już na slocie 0, i to z komunikatem o pozycji — funkcja nie ma jak być prawdziwa
w nowym świecie. Propozycja: **jeden ogólny klucz porządkowy**.

```text
klucz(source) = (ranga rodzaju, indeks wewnątrz rodzaju)

ranga:   Position 0 < Orientation 1 < Direction 2 < DirectionAngle 3
indeks:  Position       X < Y < Z
         Orientation    row-major, (0,0) ... (2,2)
         Direction      X < Y < Z
         DirectionAngle jeden, brak indeksu
```

Reguła: **klucze muszą rosnąć ściśle**. To jedno zdanie zastępuje trzy pętle,
znosi warunek `size() < 3` (istniał wyłącznie przez indeksowanie slotów) i
obejmuje wszystkie sześć rodzajów zadania bez rozgałęzień. Zachowuje przy tym
dotychczasowe zachowanie co do joty: pozycje najpierw w X, Y, Z, potem orientacja
row-major.

Klucz musi być liczony **defensywnie**, tak jak dziś `componentIndex`:
`sourceOrderKey` nie ma prawa zakładać, że kontrola zawartości już odrzuciła
podrobiony `CartesianComponent` albo `ToolAxis`. Kolejność wywołań w `create()`
jest dziś właściwa, ale zamiana dwóch linii nie może zamieniać błędu walidacji w
dostęp poza zakres — komentarz w istniejącym kodzie mówi to wprost i zostaje w
mocy. Mapowanie zwraca więc porażkę, nie surowe rzutowanie.

`DuplicateEquationSource` zostaje osobno i **przed** porządkiem, tak jak dziś —
ścisły wzrost wykryłby duplikat, ale zaraportowałby go jako złą kolejność, co
jest prawdą bezużyteczną. To dokładnie ten sam argument, którym F2.3 uzasadniło
kolejność `DuplicateUnknownIndex` przed `UnorderedUnknowns`.

Relacja `Orientation < Direction` zostaje w kluczu, choć **żaden legalny rodzaj
zadania nie zawiera dziś obu naraz** — macierz z 8.1 to wyklucza, więc taki system
odpadłby na `TaskEquationMismatch`, zanim porządek zdążyłby cokolwiek orzec. Jest
to element klucza na przyszłość, a nie zachowanie do przetestowania: test musiałby
przejść przez system nielegalny z innego powodu, a wtedy sprawdzałby kolejność
diagnoz, nie porządek równań. Testowany jest więc porządek obserwowalny dla
legalnego zadania — pozycja przed kierunkiem (10).

### 8.4 Kolejność kodów w `create()`

Dokumentowana lista kodów rośnie o dwa; nowe wchodzą **przed**
`DuplicateEquationSource`, bo dotyczą pojedynczego równania, a nie relacji między
równaniami:

```text
NoEquations, NoUnknowns, EmptyUnknownName, DuplicateUnknownName,
DuplicateUnknownIndex, UnorderedUnknowns, InvalidToolAxis, MixedToolAxes,
TaskEquationMismatch, DuplicateEquationSource, UnorderedEquations
```

---

## 9. API `ConstraintBuilder`

Cztery nowe przeciążenia, wszystkie przez ten sam kręgosłup `assemble()` z F2.5:

```cpp
build(transform, chain, const ToolDirectionTarget&)              -> 3 równania
build(transform, chain, const PositionAndToolDirectionTarget&)   -> 6
build(transform, chain, const DirectionAngleTarget&)             -> 1
build(transform, chain, const PositionAndDirectionAngleTarget&)  -> 4
```

Prywatnie dochodzą dwa emitery, symetryczne do `positionEquations` i
`appendOrientationEquations`:

```cpp
void appendDirectionEquations(std::vector<Equation>&, const SymbolicTransform&,
                              ToolAxis, const Vector3& normalizedDirection) const;

void appendDirectionAngleEquation(std::vector<Equation>&, const SymbolicTransform&,
                                  ToolAxis, const Vector3& normalizedReference,
                                  double angle) const;
```

Emitery dostają kierunek **już skanonizowany**, bo to builder go kanonizuje
(7.5). Wewnątrz `build()` kolejność jest stała i wynika z 7.4:

```text
validate(target)                       -> orzeka, nie zmienia
normalizedDirection(target.direction)  -> skala, hypot, dzielenie
appendDirectionEquations(..., d)       -> emituje równania
```

Kompozycja jest wtedy dosłowna: `PositionAndToolDirection` to `positionEquations`
+ `appendDirectionEquations`, i nic więcej. Emiter rodziny II jako jedyny woła
`factory_` do zbudowania sumy iloczynów — reszta przekłada komórki.

---

## 10. Testy

```text
rodzina I — kształt
  BuildsThreeDirectionEquations              ToolDirection: 3, IkTaskKind zgodny
  UsesToolAxisColumnCells                    lhs to sameNode z (0,2),(1,2),(2,2)
  UsesXToolAxisColumnWhenAsked               ToolAxis::X -> kolumna 0
  DirectionEquationsAreOrderedXYZ
  UsesNormalizedDirectionOnRightHandSide     (0,0,5) daje te same rhs co (0,0,1)
  ToolDirectionSystemHasNoPositionEquations  reguła z 8.1
  AntiParallelDirectionIsADifferentTarget    kontrakt znaku z 2.5

rodzina I — kompozycja
  BuildsSixEquationsForPositionAndDirection
  PositionEquationsMatchPositionOnlyBuild    ten sam test co w F2.5, nowy cel

rodzina II
  BuildsOneDirectionAngleEquation
  AngleLhsIsDotProductOfAxisColumn           struktura wyrażenia, nie tylko wynik
  UsesCosineOfAngleOnRightHandSide
  BuildsFourEquationsForPositionAndAngle
  AngleOnlySystemHasNoPositionEquations

„nie narzucaj za dużo" — gate roadmapy
  DirectionTargetDoesNotConstrainFullOrientation   zero równań orientacji
  DirectionTargetLeavesSpinFree              patrz niżej
  DiffersFromFullPoseForSameTransform        12 kontra 3 równania na tym samym FK

odrzucenia
  RejectsNonFiniteDirection
  RejectsNaNDirectionInsteadOfAcceptingIt
  RejectsZeroDirection
  AcceptsUnnormalizedDirection               (0,0,5) przechodzi — kontrakt z 7.1
  AcceptsExtremeMagnitudeDirection           1e200 i 1e-200
  NormalizesMaxFiniteDirectionWithoutOverflow  {DBL_MAX, DBL_MAX, 0} -> rhs
                                             (1/sqrt2, 1/sqrt2, 0); naiwny hypot
                                             daje tu (0,0,0) — granica kontraktu
  NormalizesSubnormalDirection               {denorm_min, denorm_min, 0} -> to samo;
                                             naiwny hypot daje (1,1,0), czyli wektor
                                             o normie sqrt(2) udający jednostkowy
  RejectsAngleBelowZero / RejectsAngleAbovePi
  RejectsNonFiniteAngle
  RejectsDegenerateAngleZero / RejectsDegenerateAnglePi
  AcceptsAngleJustAboveDegeneracyThreshold   alfa = 1e-7, przechodzi
  RejectsInvalidToolAxis                     static_cast<ToolAxis>(99)
  RejectsUndeclaredSymbolInDirectionEquations

model równań (F2.6a, w test_ik_equation_model.cpp)
  RejectsPoseWithIncompleteOrientation       osiem komórek to za mało (6.2)
  AcceptsPoseWithCompleteOrientation         dziewięć, komplet
  RejectsForgedTaskKindInDofQuery            nullopt, nie zero (6.1)
  RejectsPositionEquationInDirectionTask
  RejectsDirectionEquationInPositionTask
  RejectsMixedToolAxesInOneSystem
  RejectsInvalidToolAxisInEquationSource
  RejectsForgedCartesianComponentInDirectionSource   static_cast<...>(99) w źródle
                                             kierunku, nie tylko pozycji
  AcceptsDirectionSystemWithoutPositionEquations
  OrdersPositionBeforeDirection              w PositionAndToolDirection: kierunek
                                             przed pozycją -> UnorderedEquations

end-to-end
  BuildsKr640PositionAndDirectionSystem
  BuildsKr4PositionAndAngleSystem
```

**`DirectionTargetLeavesSpinFree` zasługuje na komentarz**, bo to jedyny test w
tym etapie sprawdzający **nieobecność** ograniczenia, a nieobecność łatwo
udowodnić przypadkiem. Kształt: weź konfigurację, obróć ostatni joint pokrywający
się z osią narzędzia, podstaw obie przez `ExpressionEvaluator` — wszystkie trzy
równania kierunku muszą dać tę samą wartość, mimo że orientacja się zmieniła.

To jest test, który naprawdę realizuje zdanie z promptu: „przy swobodnym obrocie
wokół osi dyszy nie wolno przypadkiem narzucić pełnej orientacji". Test liczący
równania tego nie pokazuje — pokazuje to dopiero podstawienie.

---

## 11. Plan zmian i podział na proposale implementacyjne

```text
model/ToolAxis.hpp                    NOWY, enum + toolAxisColumn + konwencja (3)
model/IkTarget.hpp                    +4 alternatywy, +4 wartości IkTaskKind
model/Equation.hpp                    +2 źródła, +2 EquationKind, visitor
model/TargetValidation.{hpp,cpp}      +6 kodów, +2..3 przeciążenia validate
model/IkEquationSystem.{hpp,cpp}      +2 kody, macierz zawartości (8.1),
                                      spójność osi (8.2), porządek po kluczu (8.3)
model/IkTaskDegreesOfFreedom.{hpp,cpp}  NOWY (6)
builders/ConstraintBuilder.{hpp,cpp}  +4 przeciążenia, +2 emitery,
                                      +normalizedDirection (prywatna, 7.4/7.5)
tests/                                ~37 testów w trzech plikach
```

**Dwa proposale implementacyjne, nie jeden:**

```text
F2.6a  rodzina I — kierunek
         ToolAxis, toolAxisColumn
         IkTarget:   2 -> 4   (ToolDirectionTarget, PositionAndToolDirectionTarget)
         IkTaskKind: 2 -> 4
         DirectionEquationSource
         przebudowa IkEquationSystem z 8 (macierz, oś, porządek) + Pose z 6.2
         taskSpaceConstrainedDegreesOfFreedom
         dwa przeciążenia ConstraintBuilder
         ~24 testy

F2.6b  rodzina II — kąt
         IkTarget:   4 -> 6   (DirectionAngleTarget, PositionAndDirectionAngleTarget)
         IkTaskKind: 4 -> 6
         DirectionAngleEquationSource
         walidacja kąta z 7.2
         pierwsze liczone lhs w projekcie
         dwa pozostałe przeciążenia ConstraintBuilder
         ~13 testów
```

Cięcie idzie **po rodzinach**, nie po warstwach, i to jest poprawka z v3. v2
dawała do F2.6a wszystkie cztery cele i sześć rodzajów zadania, zostawiając
źródło równania kąta na później — czyli po zamknięciu F2.6a istniałby legalny
`IkTaskKind::DirectionAngle`, dla którego tabela z 8.1 wymaga jednego równania
kąta, a `EquationSource` nie potrafi takiego równania przechować. Model
niespójny na własne żądanie.

Moje uzasadnienie tamtego podziału było przy tym **wprost nieprawdziwe**: pisałem,
że dwa brakujące zadania będą „odrzucane jawnie przez `SystemRejected`". Nie
będą — bez przeciążenia `build(..., const DirectionAngleTarget&)` takie wywołanie
**nie kompiluje się** i żaden błąd środowiskowy nie powstaje. Do tego jawny
wizytor w `TargetValidation` musiałby i tak obsłużyć obie nowe alternatywy już w
F2.6a, choć walidacja kąta miała przyjść dopiero w F2.6b.

Argument „dwie przebudowy `-Wswitch` zamiast jednej" też upada, i to jest właściwe
odwrócenie: **po to `-Wswitch` istnieje.** Dwa zatrzymania kompilatora przy dwóch
świadomych rozszerzeniach modelu są zaletą, nie kosztem. Cena za uniknięcie ich —
półgotowy rodzaj zadania w modelu — jest znacznie wyższa.

Po cięciu po rodzinach **model jest spójny na końcu każdego z dwoch etapów**: każdy
`IkTaskKind`, który istnieje, ma komplet źródeł, walidację, przeciążenie buildera i
testy. Powodu objętościowego nie ma — jest powód gotowości.

---

## 12. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Trzy równania kierunku czytane jako trzy DOF | **realne** | Tabela w 6, `taskSpaceConstrainedDegreesOfFreedom`, komentarz w typie, test `DirectionTargetLeavesSpinFree` |
| Reguły zawartości rozszerzone niekompletnie | **realne, złapane w review v1** | Pełna macierz (8.1) plus zera; test na każdą niedozwoloną kombinację |
| `static_cast<ToolAxis>(99)` | **realne, złapane w review v1** | `toolAxisColumn` zwracające `optional`, tłumaczone na kod błędu osobno w każdej warstwie (3.2, 7.3) |
| Trzy równania o trzech różnych osiach | **realne** | `MixedToolAxes` (8.2) — sama kompletność X, Y, Z tego nie łapie |
| `alfa` na końcu przedziału | **realne, złapane w review v1** | Odrzucane dwustopniowo: dokładne `0` i `pi` przenośnie, potem `\|cos\| == 1`; żadnego publicznego progu (7.2) |
| Kierunek z iloczynu wektorowego odrzucony za normę | **realne, złapane w review v1** | Brak progu normy; niezerowość + kanonizacja w builderze (7.1, 7.5) |
| Kierunek na granicy zakresu `double` zniszczony przez normalizację | **realne, złapane w review v3** | Skalowanie przed `hypot` (7.4); `{DBL_MAX, DBL_MAX, 0}` i `{denorm_min, denorm_min, 0}` mają własne testy |
| Typ obiecuje „normalized", czego nie gwarantuje | **realne, złapane w review v3** | Cel przechowuje promień i tak jest opisany; kanonizuje builder (4.1, 7.5) |
| `FreeRotationAroundToolAxis` wraca jako typ | realne (nazwa jest w roadmapie) | 2.3 rozstrzyga; roadmapa do zaktualizowania razem z tym proposalem |
| Pierwsze liczone `lhs` psuje gwarancję symboli | niskie | Kolektor F2.5 chodzi po równaniach, nie po komórkach — dlatego akurat tu nie ma zmiany |
| Eksplozja alternatyw `IkTarget` | świadome | 4.3: zbiór rośnie o zadania zamawiane, nie o kombinacje możliwe |
| Etap kończy się modelem, który sam siebie nie potrafi obsłużyć | **realne, złapane w review v2** | Cięcie po rodzinach (11): każdy istniejący `IkTaskKind` ma komplet źródeł, walidację i przeciążenie buildera |
| Funkcja DOF kłamie o `Pose` z niepełną orientacją | **realne, złapane w review v2** | `Pose` domknięte do dziewięciu komórek (6.2); koszt to dwa fixture'y testowe, zero zmian w produkcji |
| Próg degeneracji kąta wpisany w kontrakt | **realne, złapane w review v2** | Dokładne `0` i `pi` odrzucane przenośnie; `1.05e-8` zostaje obserwacją (7.2) |

---

## 13. Rozstrzygnięte w review (v2)

| # | Pytanie / zarzut z v1 | Rozstrzygnięcie |
|---|---|---|
| 1 | Rodzina II tylko z pozycją | **Blocker przyjęty.** Obie rodziny z pozycją i bez; argument „za mało DOF, żeby być zadaniem" wycofany jako niespójny (4.1) |
| 2 | „Jedna reguła kompletności pozycji" | **Blocker przyjęty.** Pełna macierz zawartości + porządek po kluczu; `checkEquationOrder` przebudowany (8) |
| 3 | DOF przy `alfa = 0, pi` | **Blocker przyjęty.** Przedział otwarty, końce do rodziny I, próg zmierzony (6, 7.2) |
| 4 | `ToolAxis` bez walidacji runtime | **Blocker przyjęty.** `toolAxisColumn` + `InvalidToolAxis` w obu warstwach + `MixedToolAxes` (3.2, 8.2) |
| 5 | Tolerancja normy kierunku (13.1 z v1) | **Bez progu.** Skończony + niezerowy; kanonizacja skalowana po stronie buildera; analogia do kwaternionu była tu fałszywa i jest wycofana (7.1, 7.4, 7.5) |
| 6 | Gdzie mieszka `ToolAxis` (13.2 z v1) | **W celu.** Plus precyzyjne zdanie, że to oś ramki końcowej przekazanej transformacji (3.3) |
| 7 | Czy `constrainedDegreesOfFreedom` w tym etapie (13.3 z v1) | **Tak**, pod nazwą `taskSpaceConstrainedDegreesOfFreedom` (6) |
| 8 | `PositionAndSurfaceAngleTarget` kontra własny argument o nazwie | **Nazwy zmienione** na `DirectionAngle...`; źródło to `DirectionAngleEquationSource`, nie `AxisAngle...` (5.1) |
| 9 | `AxisAlignment` bez znaku | **Zdefiniowany ze znakiem**, z testem (2.5) |
| 10 | „trzy nowe kody", a wymienione cztery | Rachunek poprawiony: sześć w `TargetValidation`, dwa w `IkEquationSystem` (7.4) |

## 13a. Rozstrzygnięte w drugim review (v3)

| # | Zarzut z v2 | Rozstrzygnięcie |
|---|---|---|
| 1 | Podział zostawiał `IkTaskKind` bez źródła równania | **Blocker przyjęty.** Cięcie po rodzinach; moje „odrzucane przez `SystemRejected`" było nieprawdą — taki kod się nie kompiluje (11) |
| 2 | `toolAxisColumn` zwracające `expected` | **`optional`.** `ToolAxis.hpp` nie może znać żadnego z dwóch słowników błędu (3.2) |
| 3 | Brak kontraktu dla podrobionego `IkTaskKind` | **`optional`** — `0` udawałoby legalne zero DOF (6.1) |
| 3b | `Pose` nadal dopuszcza niepełną orientację | **Domknięte do dziewięciu.** Inaczej publiczna funkcja DOF kłamałaby o systemie, który model dopuszcza; koszt policzony (6.2) |
| 4 | `1.05e-8` w kontrakcie degeneracji | **Wycofane z kontraktu.** Przenośne odrzucenie dokładnych końców + `|cos| == 1`; komunikat przepisany (7.2) |
| 5 | Fałszywa analogia w kolejności kontroli kierunku | **Uzasadnienie wymienione.** `inf`/`NaN` nie dają `ZeroDirection`; prawdziwy powód to niezależność od zachowania `hypot` na nieskończonościach — zmierzone: `hypot(inf,0,0)` zwraca tu `NaN`, wbrew C99 (7.1) |

## 13c. Nota do F2.6a — kolejność diagnoz przy `Pose`

Zapisane tutaj, bo to pułapka implementacyjna, a nie decyzja architektoniczna:
zaostrzenie `Pose` do dziewięciu równań (6.2) **nie może** raportować „brakująca
komórka orientacji" wcześniej niż dotychczasowe kody. Test z duplikatem źródła
przestałby wtedy docierać do `DuplicateEquationSource` — a on jest po to, żeby
diagnozować dwa równania na tę samą komórkę.

Kompletność ma wynikać z trzech reguł, które już istnieją albo dochodzą,
sprawdzanych w dotychczasowej kolejności:

```text
dziewięć równań orientacji            (liczba, w checkEquationContent)
+ poprawne OrientationEquationSource  (0..2 x 0..2, przez create())
+ brak duplikatu źródła               (DuplicateEquationSource, bez zmian)
= komplet dziewięciu komórek
```

## 13b. Rozstrzygnięte w trzecim review (v4)

| # | Zarzut z v3 | Rozstrzygnięcie |
|---|---|---|
| 1 | „Normalizowany przy walidacji", a `validate()` bierze `const&` | **Blocker przyjęty.** Walidacja orzeka, builder kanonizuje; cel przechowuje **promień** i tak jest opisany w typie (4.1, 7.5) |
| 2 | `hypot` na surowych składowych nie pokrywa kontraktu | **Blocker przyjęty.** Kanonizacja skalowana; zmierzone `{DBL_MAX, DBL_MAX, 0}` → naiwnie `(0,0,0)`, a dodatkowo znaleziony gorszy przypadek `{denorm_min, denorm_min, 0}` → naiwnie `(1,1,0)` (7.4) |
| 3 | Pozostałości po v2 w tabeli ryzyk | Wyczyszczone: `optional` zamiast `expected`, brak publicznego progu kąta (12) |
| 4 | Podrobiony `CartesianComponent` w źródle kierunku | Dopisany test (10) |

Otwarte pytania: **brak**.

---

## 14. Rekomendacja

Przyjąć redukcję pięciu nazw do dwóch rodzin (2), symetryczny zbiór czterech
nowych zadań (4.1), przebudowę reguł modelu z 8 i walidację z 7. Po zatwierdzeniu
— **dwa** proposale implementacyjne, F2.6a i F2.6b (11).

Po F2.6 zostaje **F2.7**: integracja z fasadą, Definition of Done Fazy 2 i
release `v0.2-ik-equations`.
