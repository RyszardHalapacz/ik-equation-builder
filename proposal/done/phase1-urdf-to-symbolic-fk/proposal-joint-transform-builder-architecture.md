# Proposal: `JointTransformBuilder` — architektura

> **Rewizja v2 po review.** Siedem uwag naniesionych, wszystkie zweryfikowane uruchomieniem na rzeczywistej warstwie symbolicznej. Trzy z nich to były błędy w v1, w tym dwa matematyczne w planie testów. Najważniejsze odkrycie rewizji: **pełne mnożenie macierzy 4×4 niszczy kanoniczną reprezentację ostatniego wiersza** (§14.2) — to zmienia konstrukcję z opcjonalnej optymalizacji w wymóg zachowania kanonicznej postaci. Werdykt zmieniony z `requires architectural decision` na `approve` (§19), bo decyzja o Pakiecie A została podjęta.

## 1. Cel

Przekształcić pojedynczy `KinematicJoint` w symboliczną jednorodną macierz 4×4 reprezentującą `T_parent_child(q)` — transformację z układu linku rodzica do układu linku dziecka, sparametryzowaną zmienną złączową.

To pierwszy etap, w którym model łańcucha i warstwa symboliczna spotykają się w rzeczywistej reprezentacji kinematyki. Do tej pory obie istniały osobno: `KinematicChain` wiedział *które* jointy i w jakiej kolejności, `Expression` potrafił reprezentować dowolną formułę — ale nic nie zamieniało geometrii jointu na formułę.

## 2. Stan obecny repozytorium

Wszystko poniżej **odczytane z kodu i zweryfikowane uruchomieniem**, nie z README ani `STATUS.md`.

### 2.1 Istniejący nagłówek — jedyne, co jest

```cpp
// src/ik_equations/builders/JointTransformBuilder.hpp
class JointTransformBuilder
{
public:
    SymbolicTransform build(const KinematicJoint& joint) const;
};
```

Brak `.cpp`, brak wpisu w `CMakeLists.txt`, brak testów. Klasa bezstanowa, metoda `const` — zgodnie z konwencją repo (`UrdfModelLoader`, `KinematicChainBuilder`).

### 2.2 Model danych wejściowych

`KinematicJoint` (`model/KinematicChain.hpp`) — agregat, bez zachowania:

| Pole | Typ | Uwaga |
|---|---|---|
| `index` | `std::size_t` | pozycja w łańcuchu, 0-based |
| `name`, `parentLink`, `childLink` | `std::string` | |
| `type` | `JointType` | `Fixed`/`Revolute`/`Continuous`/`Prismatic` |
| `origin` | `JointOrigin` | `{Vector3 translation; Vector3 rpy;}` |
| `axis` | `Vector3` | `{double x{}, y{}, z{};}` |
| `limits` | `JointLimits` | **nieużywane przez ten komponent** |
| `variable` | `std::optional<JointVariable>` | `JointVariable{std::string name; std::size_t index;}` |

### 2.3 Warstwa symboliczna — dostępne operacje

`ExpressionFactory` (bezstanowa klasa, metody `const`): `constant`, `symbol`, `add`, `subtract`, `multiply`, `divide`, `negate`, `sin`, `cos`.

Normalizacja stosowana przy budowie: **zwijanie stałych** oraz **elementy neutralne** (`x+0→x`, `x·1→x`, `x/1→x`). **Anihilator `x·0 → 0` świadomie NIE jest stosowany** — kasowałby informację o dziedzinie. To ma bezpośredni, mierzalny wpływ na ten komponent (§14).

`SymbolicMatrix<R,C>`: `operator()(row, col)`, `zeros()`, `identity()` (tylko kwadratowe), wolna funkcja `multiply(lhs, rhs, factory)` — left-fold po rosnącym `k`. `SymbolicTransform = SymbolicMatrix<4,4>`.

### 2.4 Co faktycznie dociera do buildera — pomiar

Uruchomiłem `UrdfModelLoader` + `KinematicChainBuilder` na obu URDF-ach i wypisałem rzeczywiste wartości:

```
=== kr640.urdf ===
joint_a1   Revolute  axis=[0 0 1]  |axis|=1.0000  rpy=[0.0000 0.0000 0.0000]  q1
joint_a2   Revolute  axis=[0 1 0]  |axis|=1.0000  rpy=[0.0000 0.0000 0.0000]  q2
joint_a4   Revolute  axis=[1 0 0]  |axis|=1.0000  rpy=[0.0000 0.0000 0.0000]  q4
joint_a6_to_tool0  Fixed  axis=[0 0 1]  |axis|=1.0000  rpy=[0 0 0]  -

=== kr4_r600.urdf ===
joint_1    Revolute  axis=[0 0 1]  |axis|=1.0000  rpy=[3.1416 0.0000  0.0000]  q1
joint_4    Revolute  axis=[0 0 1]  |axis|=1.0000  rpy=[1.5708 0.0000 -1.5708]  q4
joint_5    Revolute  axis=[0 0 1]  |axis|=1.0000  rpy=[0.0000 1.5708  1.5708]  q5
link6-tool0        Fixed  axis=[0 0 1]  |axis|=1.0000  rpy=[3.1416 0.0000 3.1416]  -
```

Cztery wnioski, każdy istotny dla decyzji poniżej:

1. **`rpy` jest w radianach** — `3.1416` = π, `1.5708` = π/2. Loader nie konwertuje, przepisuje surowo z URDF (który specyfikuje radiany).
2. **Wszystkie osie w obu robotach są już jednostkowe** — ale to właściwość *tych* plików, nie gwarancja systemu (patrz 3).
3. **Nigdzie w kodzie nie ma normalizacji.** `grep -rn "normaliz\|sqrt\|length\|magnitude" src/` nie znajduje nic poza komentarzem o normalizacji wyrażeń. Komentarz w `robot_model.hpp` mówi *„unit vector in parent's local frame"*, ale **nic tego nie egzekwuje** — `parse_xyz` przepisuje surowe liczby.
4. **Fixed jointy mają `axis = [0,0,1]`, nie `[0,0,0]`.** URDF nie ma dla nich elementu `<axis>`, więc zostaje domyślna wartość z `mt::kinematics::Joint{ Vec3 axis = {0,0,1}; }`. Oznacza to, że „oś zerowa" **nie** pojawia się naturalnie dla fixed jointów.

### 2.5 Walidacja w loaderze — czego brakuje

`robot_model_loader.cpp` sprawdza `std::isfinite` **wyłącznie dla limitów** (`j.limits.lower/upper`). Nie waliduje ani `axis`, ani `origin`.

Co gorsza, sam parser wektorów po cichu naprawia uszkodzone dane. `parse_xyz` przerywa pętlę na pierwszym błędzie `from_chars` i zwraca częściowo wypełniony wektor. Uruchomiłem go na zestawie wejść (kopia funkcji, bez zmian):

| wejście | wynik | problem |
|---|---|---|
| `"1 2 3"` | `[1, 2, 3]` | poprawne |
| `"1 abc 3"` | `[1, 0, 0]` | ciche obcięcie po błędnym tokenie |
| `"1 2"` | `[1, 2, 0]` | brakująca składowa → 0 |
| `"1"` | `[1, 0, 0]` | jedna wartość akceptowana |
| `"1 2 3 4 5"` | `[1, 2, 3]` | śmieci po trzeciej wartości ignorowane |
| `"nan 0 0"` | `[nan, 0, 0]` | **`from_chars` akceptuje `"nan"`** |
| `"1e400 0 0"` | `[0, 0, 0]` | **przepełnienie zeruje CAŁY wektor** |

Ostatni wiersz jest najgroźniejszy: `from_chars` zwraca `errc::result_out_of_range`, pętla robi `break` na **pierwszej** składowej, więc wszystkie trzy zostają zerami. Dla osi daje to oś zerową, dla origin — ciche przesunięcie do początku układu.

**To podważa założenie z promptu**, że `KinematicJoint` pochodzi „z wcześniejszego, kontrolowanego etapu". Kontrolowana jest *topologia* (`KinematicChainBuilder` waliduje ścieżkę, duplikaty rodziców, cykle) i *limity* — ale **nie geometria, ani nawet nie poprawność składniowa jej zapisu**. Konsekwencje w §11 i §15.

### 2.6 Domyślna oś niezgodna ze specyfikacją URDF

`mt::kinematics::Joint` deklaruje `Vec3 axis = {0.0, 0.0, 1.0};`. Specyfikacja URDF dla `<axis>` mówi natomiast: *„defaults to (1,0,0)"*.

Aktywny joint bez elementu `<axis>` dostanie więc w tym projekcie oś Z, podczas gdy referencyjne parsery URDF dadzą mu oś X — **cicho inną kinematykę dla tego samego pliku**. Oba testowe roboty podają osie jawnie, więc rozbieżność jest dziś niewidoczna, ale to realny błąd zgodności. Do naprawy w proposalu loadera (§15).

### 2.7 Konwencje testów i CMake

Testy: `TEST(SuiteName, TestName)`, `using`-deklaracje na górze pliku, `const ExpressionFactory factory;` lokalnie w teście, fixture'y URDF przez `KINEMAFORGE_URDF_DATA_DIR`, syntetyczne `RobotDescription` budowane ręcznie tam, gdzie plik URDF nie umie danego przypadku wyrazić. Obecnie **70 testów**, wszystkie zielone.

CMake: pliki `.cpp` wymieniane jawnie w `add_library(kinemaforge_ik STATIC ...)` i `add_executable(kinemaforge_tests ...)`.

## 3. Zakres i non-goals

**W zakresie:** `T_parent_child(q)` dla pojedynczego jointu — złożenie transformacji origin i transformacji ruchu, dla wszystkich czterech typów jointów, dla dowolnej poprawnej osi.

**Poza zakresem:** `ForwardKinematicsBuilder`, fasada `IkEquationBuilder`, ewaluator, simplifier, `ConstraintBuilder`, solver IK, generator kodu, obsługa limitów ruchu, wykrywanie osobliwości.

**Świadomie NIE robimy:** ogólnej biblioteki algebry liniowej. `SymbolicMatrix` dostaje dokładnie tyle, ile trzeba do złożenia transformacji jednorodnych — nic więcej.

## 4. Semantyka transformacji URDF

### 4.1 Trzy układy współrzędnych

```
[parent link frame]
        │
        │  T_origin  — stała, z <origin xyz rpy>
        ▼
[joint frame]  ← tu jest umocowany joint; TU wyrażona jest <axis>
        │
        │  T_motion(q)  — zmienna, zależna od typu jointu
        ▼
[child link frame]
```

Specyfikacja URDF dla `<joint><origin>`: *„the transform from the parent link to the child link. The joint is located at the origin of the child link."* Dla `<axis>`: *„The joint axis specified **in the joint frame**."*

Kluczowe: **oś jest wyrażona w układzie jointu, czyli PO zastosowaniu origin** — nie w układzie rodzica. Dlatego `T_motion` używa surowej osi z URDF, bez żadnego obracania jej przez `origin.rpy`. To dokładnie ten kontrakt, który testy loadera (`MapsKr4JointAxis`) już przypinają: oś przechodzi przez pipeline nieobrócona.

### 4.2 Kolejność — rozstrzygnięcie

```
T_parent_child(q) = T_origin · T_motion(q)
```

**`T_origin` po lewej.** Uzasadnienie z semantyki: punkt wyrażony w układzie dziecka przekształcamy do układu rodzica, idąc w górę łańcucha:

```
p_parent = T_origin · T_motion(q) · p_child
```

Czytając od prawej: najpierw ruch jointu (z układu dziecka do układu jointu), potem stałe przesunięcie/obrót origin (z układu jointu do układu rodzica).

### 4.3 Dlaczego to nie jest szczegół implementacyjny

Mnożenie macierzy nie jest przemienne, a odwrócenie kolejności daje **inną, poprawnie wyglądającą** macierz — czyli błąd, który przejdzie każdy test sprawdzający tylko „czy to jest macierz 4×4 z ostatnim wierszem [0 0 0 1]".

Konkretny kontrprzykład z rzeczywistych danych (`kr640.urdf`, `joint_a2`): `origin.translation = [0.350, 0, 0]`, `axis = [0,1,0]` (obrót wokół Y).

`T_origin · T_motion(q)` — komórka `(0,3)` (składowa X translacji wyniku):

```
T_origin  = Translation(0.35, 0, 0)
T_motion  = Ry(q)  (czysty obrót, translacja zerowa)

T_origin · T_motion  →  translacja = [0.35, 0, 0]     (niezmieniona)
T_motion · T_origin  →  translacja = Ry(q)·[0.35,0,0] = [0.35·cos q, 0, -0.35·sin q]
```

Pierwszy wynik: joint obraca się **wokół punktu odległego o 35 cm** — tak działa prawdziwe ramię. Drugi: punkt zaczepienia sam się obraca — geometrycznie bez sensu. Różnica jest widoczna w komórkach `(0,3)` i `(2,3)`, i **zależy od `q`**, czyli w wersji odwróconej te komórki przestają być stałymi. To jest sygnatura, którą test może przypiąć (§16, `CombinesOriginAndMotionInCorrectOrder`).

## 5. Konwencja RPY

### 5.1 Rozstrzygnięcie

URDF definiuje `rpy` jako *„fixed axis roll, pitch and yaw angles in radians"*. **„Fixed axis" oznacza obroty ekstrynseczne** — wokół osi układu nieruchomego, nie wokół osi obracanych po drodze.

Dla obrotów ekstrynsecznych w kolejności X → Y → Z, macierz złożona to iloczyn **w odwrotnej kolejności**:

```
R_rpy(r, p, y) = Rz(y) · Ry(p) · Rx(r)
```

| symbol | nazwa | oś | pole w `JointOrigin::rpy` |
|---|---|---|---|
| `r` | roll | X | `.x` |
| `p` | pitch | Y | `.y` |
| `y` | yaw | Z | `.z` |

(Równoważnie: to samo, co obroty intrynseczne Z-Y-X. Klasyczna tożsamość, warto ją znać, żeby nie pomylić się przy czytaniu innych źródeł.)

### 5.2 Macierze składowe

```
Rx(r) = [ 1     0        0    ]      Ry(p) = [  cos p  0  sin p ]
        [ 0   cos r   -sin r  ]              [    0    1    0   ]
        [ 0   sin r    cos r  ]              [ -sin p  0  cos p ]

Rz(y) = [ cos y  -sin y  0 ]
        [ sin y   cos y  0 ]
        [   0       0    1 ]
```

### 5.3 Zgodność z URDF — weryfikacja na rzeczywistych danych

`kr4_r600.urdf`, `joint_4`: `rpy = (π/2, 0, −π/2)` — dwie niezerowe składowe, więc **przypadek rozróżniający**. Policzone ręcznie:

```
Rz(−π/2)·Ry(0)·Rx(π/2) = [  0   0  −1 ]        (konwencja URDF)
                          [ −1   0   0 ]
                          [  0   1   0 ]

Rx(π/2)·Ry(0)·Rz(−π/2) = [  0   1   0 ]        (odwrócona — BŁĘDNA)
                          [  0   0  −1 ]
                          [ −1   0   0 ]
```

Wyniki są różne, więc ten joint nadaje się na test konwencji. **Uwaga na pułapkę:** `joint_1` (`rpy = (π,0,0)`) i `link6-tool0` (`rpy = (π,~0,π)`) dają **ten sam wynik w obu konwencjach** — przy pojedynczej niezerowej składowej kolejność nie ma znaczenia, a przy dwóch obrotach o π macierze przypadkiem komutują. Test konwencji **musi** używać `joint_4`-podobnych danych, nie tych.

### 5.4 Osadzenie w 4×4

Blok 3×3 trafia w lewy górny róg, ostatnia kolumna to translacja, ostatni wiersz to `[0 0 0 1]`:

```
T = [ R₃ₓ₃   t₃ₓ₁ ]
    [ 0 0 0    1  ]
```

Ponieważ `SymbolicMatrix::identity()` już ustawia przekątną na 1 i resztę na 0, budowa zaczyna się od `identity()` i nadpisuje tylko potrzebne komórki — ostatni wiersz zostaje poprawny **z konstrukcji**, bez ani jednej instrukcji.

## 6. Model transformacji origin

```
T_origin = Translation(origin.translation) · R_rpy(origin.rpy)
```

`Translation` po lewej. Uzasadnienie: chcemy, żeby blok obrotu w wyniku był dokładnie `R_rpy`, a kolumna translacji dokładnie `origin.translation`:

```
Translation(t) · R_rpy  =  [ R_rpy   t ]        ← to chcemy
                           [ 0 0 0   1 ]

R_rpy · Translation(t)  =  [ R_rpy   R_rpy·t ]  ← translacja obrócona: NIE to
                           [ 0 0 0      1    ]
```

URDF mówi, że `xyz` to pozycja układu dziecka w układzie rodzica — czyli wartość, która ma trafić do kolumny translacji **bez obracania**. Stąd `Translation · Rotation`.

**Translacja origin nie zależy od `q`** — to stałe z pliku URDF. Cała `T_origin` jest macierzą samych stałych i identycznie wygląda dla fixed i dla aktywnych jointów.

**Optymalizacja konstrukcyjna:** zamiast budować dwie macierze i mnożyć je (co przy braku anihilatora wygenerowałoby dziesiątki martwych węzłów `x·0`), `T_origin` składamy **bezpośrednio** — blok 3×3 z `R_rpy`, kolumna z `translation`. Wynik jest identyczny matematycznie, a drzewo minimalne. To nie jest uproszczenie algebraiczne, tylko wybór taniej konstrukcji dla znanej struktury.

## 7. Model ruchu per typ jointu

| Typ | `T_motion` | Zmienna |
|---|---|---|
| `Fixed` | `I` (identyczność) | brak (`nullopt`) |
| `Revolute` | `Rotation(axis, q)` | wymagana |
| `Continuous` | `Rotation(axis, q)` — **identycznie jak Revolute** | wymagana |
| `Prismatic` | `Translation(axis.x·q, axis.y·q, axis.z·q)` | wymagana |

**Fixed:** `T_joint = T_origin`, bez mnożenia przez identyczność. Mnożenie przez `identity()` byłoby matematycznie poprawne, ale — przy braku anihilatora — rozdęłoby drzewo o człony `x·0` (zmierzone w §14). Fixed joint **nie generuje symbolu**.

**Continuous vs Revolute:** geometrycznie nie do odróżnienia. Jedyna różnica to zakres ruchu, a **limity nie należą do tego komponentu**. Traktowane wspólnie, jednym `case`.

**Prismatic:** przesunięcie wzdłuż osi o `q`. Blok obrotu zostaje identycznością, zmienia się wyłącznie kolumna translacji. Krytyczne, żeby **nie** potraktować go jak obrotu — test z osią o dwóch niezerowych składowych to wyłapie (§16).

### 7.1 Składowa zerowa osi w prismatic — jawna decyzja konstrukcyjna

Naiwne `factory.multiply(factory.constant(axis.z), q)` dla `axis.z == 0.0` **nie da stałej zero**. Zweryfikowane:

```
multiply(constant(0), symbol("q"))  →  typ = Multiply,  drzewo = (0*q),  isZero = false
```

Bo `q` jest symbolem, więc zwijanie stałych nie ma czego policzyć, a anihilator `x·0 → 0` jest świadomie wyłączony. Test oczekujący `M(2,3) == Constant(0)` **failowałby** — v1 tego proposalu zawierała dokładnie taką sprzeczność.

Rozstrzygnięcie — jawna reguła konstrukcyjna:

```cpp
// Buduje component·q, ale nie tworzy węzła mnożenia dla zerowej składowej.
Expression scaledAxisComponent(double component, const Expression& q,
                               const ExpressionFactory& factory)
{
    if (component == 0.0)
        return factory.constant(0.0);
    return factory.multiply(factory.constant(component), q);
}
```

**Dlaczego to jest bezpieczne, skoro odrzuciliśmy `x·0 → 0`?** Bo to nie ta sama operacja. Anihilator odrzuciliśmy, bo dla *dowolnego wyrażenia* `x` (np. `1/q`) mnożenie przez zero kasuje informację o dziedzinie. Tutaj:

- nie *upraszczamy* istniejącego drzewa, tylko decydujemy, jakiego **nie budować**,
- drugi operand to zawsze **goły symbol zmiennej złączowej**, czyli funkcja totalna na ℝ — nie ma dziedziny do zgubienia,
- `component` to stała geometryczna znana w chwili budowania, nie wyrażenie.

Ta sama zasada, która uzasadnia fast path dla osi osiowych (§8.1): wybór konstrukcji na podstawie stałej wejściowej, nie przekształcenie symboliczne.

## 8. Obrót wokół dowolnej osi — wzór Rodriguesa

Dla **jednostkowej** osi `a = [x, y, z]` i kąta `q`, przy `c = cos q`, `s = sin q`, `t = 1 − c`:

```
R(a, q) = [ t·x² + c      t·x·y − s·z    t·x·z + s·y ]
          [ t·x·y + s·z   t·y² + c       t·y·z − s·x ]
          [ t·x·z − s·y   t·y·z + s·x    t·z² + c    ]
```

Sprawdzenie dla `a = [0,0,1]`: `[[c,−s,0],[s,c,0],[0,0,t+c]]`, a `t+c = 1`. Daje `Rz(q)`. ✓
Sprawdzenie dla `a = [1,0,0]`: `[[1,0,0],[0,c,−s],[0,s,c]]` = `Rx(q)`. ✓

Wszystkie potrzebne operacje istnieją w `ExpressionFactory`: `cos`, `sin`, `subtract` (dla `t`), `multiply`, `add`, `constant`. Składowe osi wchodzą jako `constant(x)` — są znanymi liczbami z URDF, nie symbolami.

### 8.1 Fast path dla osi osiowych — uzasadniony pomiarem

Prompt dopuszcza specjalizację, jeśli proposal uzasadni ją jako bezpieczną. Zmierzyłem, budując obie wersje na **rzeczywistej** zaimplementowanej warstwie symbolicznej i licząc unikalne węzły DAG:

| Konstrukcja dla `axis = [0,0,1]` | Węzły DAG |
|---|---|
| ogólny Rodrigues | **33** |
| bezpośrednie `Rz(q)` | **7** |

Ale sam rozmiar to nie główny argument. Zobaczmy, jak **wyglądają komórki** przy ogólnym wzorze dla osi `[0,0,1]`:

```
(0,0) = ((((1-cos(q1))*0)*0)+cos(q1))     zamiast  cos(q1)
(0,2) = (((1-cos(q1))*0)+(sin(q1)*0))     zamiast  0
(2,2) = ((1-cos(q1))+cos(q1))             zamiast  1
```

Matematycznie poprawne, ale **komórka (0,2) nie jest zerem** i żaden istniejący mechanizm jej nie zredukuje — anihilator `x·0 → 0` został świadomie odrzucony, a simplifiera nie ma. Konsekwencja praktyczna: test `BuildsRevoluteJointAroundZAxis` **nie mógłby** sprawdzić `isZero(M(0,2))`, mimo że matematycznie to zero.

**Dlaczego fast path jest bezpieczny:** to nie jest uproszczenie algebraiczne wyrażenia symbolicznego. Oś jest **stałą znaną w chwili budowania** (liczba z pliku URDF). Wybór między dwiema równoważnymi konstrukcjami na podstawie stałej wejściowej to zwykłe rozgałęzienie kodu, nie przekształcenie drzewa. Obie ścieżki produkują *tę samą funkcję matematyczną*; różnią się tylko rozmiarem reprezentacji.

**Zasięg:** po normalizacji, jeśli oś to dokładnie `±X`, `±Y` lub `±Z`, budujemy odpowiednio `Rx`/`Ry`/`Rz` (ze znakiem kąta odwróconym dla ujemnych). W każdym innym przypadku — ogólny Rodrigues. Pokrycie na rzeczywistych danych: **100%** (KR640 używa `[0,0,1]`, `[0,1,0]`, `[1,0,0]`; KR4 wyłącznie `[0,0,1]`).

**Ryzyko:** dwie ścieżki kodu zamiast jednej, więc dwie do przetestowania. Mitygacja: testy dla obu (`BuildsRevoluteJointAroundZAxis` trafia w fast path, `BuildsRevoluteJointAroundArbitraryAxis` w ogólną), plus test spójności — patrz §16.

## 9. Normalizacja i walidacja osi

### 9.1 Kto normalizuje — stan faktyczny

Prompt każe rozważyć trzy miejsca. Weryfikacja kodu (§2.4, punkt 3) daje jednoznaczną odpowiedź: **żadne z nich tego nie robi.** Komentarz w `robot_model.hpp` deklaruje „unit vector", ale to dokumentacja życzeniowa — `parse_xyz` przepisuje surowe liczby, a `KinematicChainBuilder` kopiuje je bez zmian (`copyJointData`).

Więc pytanie nie brzmi „które z trzech", tylko „**gdzie to dodać**".

### 9.2 Rozstrzygnięcie: normalizacja w loaderze, nie w tym builderze

**Zmiana względem v1 (uwaga z review).** Pierwsza wersja rekomendowała walidację w loaderze, ale normalizację w tym builderze — i uzasadniała to tym, że builder „i tak musi policzyć normę na potrzeby fast path". **Ten argument był błędny:** po normalizacji wybór fast path sprowadza się do porównania składowych (`x==0 && y==0 && z==1`), bez liczenia normy.

Rozdzielenie walidacji od normalizacji rozmywało odpowiedzialność: loader stwierdzałby „ta oś jest poprawna", ale przekazywał ją w postaci niekanonicznej, a każdy kolejny konsument musiałby pamiętać o normalizacji. Skoro przyjmujemy zasadę **„waliduj na granicy"**, granica ma oddawać dane **kanoniczne**.

Nowy kontrakt `RobotDescription` (do wprowadzenia w proposalu loadera, §15):

```
dla jointu aktywnego:  axis jest skończona, niezerowa i JEDNOSTKOWA
dla jointu Fixed:      axis nieistotna
```

`JointTransformBuilder` tylko **asertuje** ten inwariant i buduje macierz. Nie normalizuje.

Normalizacja w loaderze pozostaje **numeryczna, na `double`**:

```
norm = sqrt(x² + y² + z²)
a = [x/norm, y/norm, z/norm]
```

Kategorycznie **nie** budujemy symbolicznego `axis / |axis|` — oś jest stałą, więc dzielenie należy wykonać raz w `double`, a nie zapisać jako węzły `Divide` powielone w dziewięciu komórkach macierzy.

### 9.3 Oś zerowa i tolerancja

| Sytuacja | Zachowanie |
|---|---|
| oś nieznormalizowana (np. `[0,0,2]`) | normalizowana cicho — URDF nie wymaga jednostkowej, a intencja jest jednoznaczna |
| oś zerowa, joint **aktywny** | **błąd** — nie da się obracać ani przesuwać wzdłuż niczego |
| oś zerowa, joint **Fixed** | **ignorowana** — `T_motion = I`, oś nie jest w ogóle czytana |
| oś niefinitywna (`NaN`/`Inf`) | **błąd** (loader tego nie sprawdza, §2.5) |

Wszystkie te sprawdzenia należą teraz do **loadera** (§9.2). Ten builder je wyłącznie asertuje.

**Tolerancja:** oś uznajemy za zerową, gdy `norm < 1e-12`. Uzasadnienie doboru: nie jest to porównanie „bliskości" dwóch wyników obliczeń, tylko odsianie przypadku wprost zdegenerowanego. Wartości osi pochodzą z tekstu URDF; realna oś ma normę rzędu jedności, a `1e-12` jest o rzędy wielkości poniżej czegokolwiek, co dałoby sensowną geometrię po podzieleniu. Alternatywa „dokładnie `== 0.0`" jest gorsza — oś `[1e-300, 0, 0]` znormalizowałaby się do `[1,0,0]`, dając cichą, całkowicie przypadkową geometrię zamiast błędu.

**Fixed joint nie wymaga poprawnej osi** — i to nie jest teoretyczne: zmierzone dane (§2.4, punkt 4) pokazują, że fixed jointy dostają `[0,0,1]` z domyślnej wartości `mt::kinematics::Joint`, bo URDF nie ma dla nich `<axis>`. Wartość jest przypadkowa i nie powinna być czytana.

## 10. Inwariant zmiennej symbolicznej

Zweryfikowany w `KinematicChainBuilder.cpp`: `isActuated(type)` obejmuje `Revolute`, `Prismatic`, `Continuous`; zmienna nadawana jest wyłącznie im, `Fixed` dostaje `nullopt`. Test `AssignsSymbolsOnlyToActuatedJoints` to przypina.

Kontrakt dla tego buildera:

```
Fixed                             →  variable == nullopt
Revolute / Continuous / Prismatic →  variable.has_value() == true
```

Builder **nie może**:
- wygenerować identyczności dla aktywnego jointu bez zmiennej (cicho gubiąc stopień swobody),
- wymyślić własnej nazwy symbolu (np. `"q" + std::to_string(index)`) — nazwa musi pochodzić **dokładnie** z `joint.variable->name`, inaczej FK i przyszły solver operowałyby na rozjeżdżających się nazwach,
- zignorować zmiennej,
- nadać symbolu fixed jointowi.

Symbol powstaje jako `factory.symbol(joint.variable->name)`. Pole `JointVariable::index` **nie jest** używane do budowy nazwy — jest metadanymi dla późniejszych etapów.

## 11. Model błędów i preconditions

Review rozważyło dwie możliwe granice walidacji. **Przyjęto Pakiet A** — walidacja i normalizacja geometrii w loaderze, `assert` w tym builderze, sygnatura `build()` bez zmian. Odrzucona alternatywa jest krótko odnotowana w trade-offach (§17).

Punkt wyjścia dla tej decyzji: prompt zakładał, że `KinematicJoint` pochodzi z kontrolowanego etapu — a weryfikacja kodu pokazała, że **geometria nie jest walidowana nigdzie** (§2.5). To przesunęło pytanie z „jak zgłaszać naruszenie precondition" na „gdzie w ogóle umieścić brakującą walidację danych zewnętrznych".

### 11.1 Klasyfikacja błędów

| Sytuacja | Skąd pochodzi | Czy wcześniejszy etap to wyłapie? |
|---|---|---|
| aktywny joint bez zmiennej | bug w `KinematicChainBuilder` albo ręcznie zbudowany `KinematicJoint` | — (to bug) |
| fixed joint ze zmienną | j.w. | — (to bug) |
| nieobsługiwany `JointType` | nowa wartość enuma bez obsługi | — (błąd kompilacji przy `switch` bez `default`) |
| **oś zerowa dla aktywnego jointu** | **plik URDF użytkownika** | **NIE** (§2.5) |
| **`NaN`/`Inf` w `origin` lub `axis`** | **plik URDF użytkownika** | **NIE** (§2.5) |
| oś nieznormalizowana | plik URDF | nie jest błędem — normalizujemy |

Trzy pierwsze to jednoznacznie **naruszenia kontraktu wewnętrznego** → `assert`, zgodnie z linią przyjętą dla warstwy symbolicznej i `SymbolicMatrix`.

Dwa wyróżnione to co innego: **dane zewnętrzne, których żaden wcześniejszy etap nie sprawdza.** Zweryfikowałem to — loader waliduje `isfinite` tylko dla limitów, a `KinematicChainBuilder` kopiuje geometrię bez oglądania. Malformed URDF przechodzi całą drogę i ten builder jest **pierwszym miejscem, które w ogóle może to zauważyć**.

Ustalona w projekcie linia (proposal `KinematicChainBuilder`, §10/§11) brzmi: *błędy domenowe → `std::expected`; naruszenia kontraktu → `assert`*. Według tej linii oś zerowa to błąd domenowy — jest dokładnym odpowiednikiem `InvalidRobotDescription`, który `KinematicChainBuilder` zwraca dla linku z dwoma rodzicami.

### 11.2 Przyjęta granica: walidacja na wejściu danych

Geometria jest walidowana i normalizowana tam, gdzie wchodzi do systemu — w loaderze, obok istniejącej walidacji limitów. `JointTransformBuilder` dostaje dzięki temu prawdziwe precondition i zachowuje zadeklarowaną sygnaturę:

```cpp
SymbolicTransform build(const KinematicJoint& joint) const;   // bez zmian
```

- Walidacja w jednym miejscu, przy wejściu danych — nie rozsiana po pipelinie
- `ForwardKinematicsBuilder` i fasada nie muszą propagować błędów z tego poziomu
- Spójne z tym, że loader **już** waliduje limity (`isfinite`, `lower ≤ upper`)
- Kosztem: wymaga osobnego, **poprzedzającego** proposalu loadera (§15)

Ten builder wyłącznie asertuje inwarianty:

```
Fixed     → variable == nullopt
Actuated  → variable.has_value()
Actuated  → axis skończona, niezerowa i jednostkowa
```

Uzasadnienie przyjęte z review: **`JointTransformBuilder` ma opisywać matematykę poprawnego jointu, a nie naprawiać uszkodzony XML.** Rozsypanie walidacji po pipelinie oznaczałoby, że każdy kolejny konsument `RobotDescription` musi ją powtarzać albo jej ufać bez podstaw.

**Odrzucona alternatywa:** zwracanie `std::expected<SymbolicTransform, JointTransformError>` z tego buildera. Byłoby bezpieczne od razu, bez zależności od proposalu loadera, ale wykrywałoby problem z danymi trzy etapy po ich wejściu do systemu i wprowadzało drugi enum błędów dla tej samej kategorii problemu (uszkodzony opis robota).

**Kolejność prac:** najpierw proposal walidacji w loaderze (§15), potem implementacja tego komponentu. Ten dokument nie może zostać wdrożony przed tamtym — inaczej asercje opisywałyby inwariant, którego nic nie ustanawia.

## 12. Projekt API

Publiczne API bez zmian względem zadeklarowanego:

```cpp
class JointTransformBuilder
{
public:
    explicit JointTransformBuilder(ExpressionFactory factory = {});

    SymbolicTransform build(const KinematicJoint& joint) const;

private:
    ExpressionFactory factory_;
};
```

### 12.1 Własność `ExpressionFactory` — rozstrzygnięcie

Cztery warianty z promptu:

| Wariant | Ocena |
|---|---|
| lokalna fabryka w `build()` | Działa i jest **równie testowalna** jak pole — fabryka jest bezstanowa. Ukrywa jednak zależność w sygnaturze |
| parametr metody `build(joint, factory)` | Jawne, ale zaśmieca każde wywołanie; `ForwardKinematicsBuilder` musiałby przekazywać ją dalej przy każdym joincie |
| **pole ustawiane w konstruktorze** *(rekomendowane)* | Zależność widoczna w typie, ustawiana raz, `build()` zostaje jednoargumentowe |
| pole tworzone wewnątrz | To samo co lokalna, tylko raz |

**Rekomendacja: pole, wstrzykiwane przez konstruktor z wartością domyślną.**

**Sprostowanie po review — nie przeceniajmy tej decyzji.** V1 uzasadniała ją „wstrzykiwalnością", i to była przesada: `ExpressionFactory` to **konkretny typ bez interfejsu polimorficznego**, więc parametr typu `ExpressionFactory` nie pozwala podstawić niczego innego. Realna korzyść jest węższa — jeśli fabryka kiedyś zyska stan lub konfigurację, będzie ją można przekazać **bez zmiany sygnatury `build()`** ani miejsc wywołania. Testowalność nie jest tu argumentem: lokalna fabryka byłaby dokładnie tak samo testowalna.

Zostawiam pole dla spójności przyszłego API i widoczności zależności w typie — ale to decyzja o małej wadze, nie fundament projektu. Domyślny argument sprawia, że `JointTransformBuilder builder;` nadal działa, więc konwencja repo („bezstanowe komponenty konstruowane w miejscu użycia") jest zachowana.

**Odnotowanie:** to niewielkie odejście od wzorca `ForwardKinematicsBuilder::build(chain, transformBuilder)`, gdzie współpracownik idzie parametrem metody. Tam ma to sens, bo `JointTransformBuilder` jest *argumentem operacji*. Tutaj fabryka jest *zależnością komponentu* — używana w każdym wywołaniu, identycznie.

## 13. Wewnętrzny podział funkcji

Wszystkie helpery w **anonimowej przestrzeni nazw w `.cpp`**. Nie wynoszę ich do publicznego API — nie ma dziś drugiego konsumenta, a `ForwardKinematicsBuilder` będzie potrzebował całych transformacji jointów, nie ich składników.

Podział dostosowany do konstrukcji blokowej z §14.2 — helpery operują na blokach 3×3 i 3×1, a nie na pełnych transformacjach 4×4:

```cpp
SymbolicRotation buildRpyRotation(
    const Vector3& rpy,
    const ExpressionFactory& factory);

SymbolicRotation buildAxisAngleRotation(      // Rodrigues + fast path dla ±X/±Y/±Z
    const Vector3& unitAxis,
    const Expression& variable,
    const ExpressionFactory& factory);

SymbolicVector3 buildPrismaticDisplacement(   // używa scaledAxisComponent (§7.1)
    const Vector3& unitAxis,
    const Expression& variable,
    const ExpressionFactory& factory);

SymbolicTransform assembleTransform(          // R i p → 4×4, ostatni wiersz z identity()
    const SymbolicRotation& rotation,
    const SymbolicVector3& translation);
```

**Brak `normalizeAxis`.** Po wyborze Pakietu A normalizacja należy wyłącznie do loadera; parametry nazwane `unitAxis` niosą ten kontrakt w samej sygnaturze.

Główny `build()` składa przypadki bezpośrednio, **bez tworzenia ogólnego `T_motion`**:

```
Fixed:                  R = R_origin
                        p = p_origin

Revolute / Continuous:  R = R_origin · R_motion       (mnożenie 3×3)
                        p = p_origin

Prismatic:              R = R_origin
                        p = p_origin + R_origin · displacement
```

Podział spełnia kryteria z promptu:

- **łatwy do testowania** — `buildRpyRotation` i `buildAxisAngleRotation` to najgęstsza matematyka; testy trafiają w nie przez `build()` z danymi dobranymi tak, by izolować jedną ścieżkę,
- **nie tworzy biblioteki algebry** — helpery odpowiadają konkretnym pojęciom URDF (obrót z RPY, obrót wokół osi, przesunięcie prismatic), a jedyną operacją macierzową jest istniejące `multiply` na blokach,
- **nie miesza semantyki URDF z operacjami macierzowymi** — `buildRpyRotation` i `buildPrismaticDisplacement` znają URDF, `assembleTransform` zna tylko strukturę macierzy jednorodnej,
- **możliwość późniejszego reużycia** — gdyby `ConstraintBuilder` potrzebował `buildRpyRotation` (orientacja docelowa też bywa zadana w RPY), wyniesienie będzie prostym przeniesieniem. Zwracanie `SymbolicRotation` zamiast 4×4 czyni to jeszcze łatwiejszym. Nie robię tego teraz, bo to spekulacja.

## 14. Kształt drzew symbolicznych i wpływ braku simplifiera

### 14.1 Zmierzone konsekwencje

Brak anihilatora `x·0 → 0` (świadoma decyzja z warstwy symbolicznej) oznacza, że **mnożenia przez zero symbolicznie przetrwają w drzewie**. Zmierzone na rzeczywistej implementacji:

| Konstrukcja | Węzły DAG |
|---|---|
| Rodrigues dla `[0,0,1]` | 33 |
| fast path `Rz` | 7 |
| Rodrigues dla `[0.707, 0.707, 0]` | 43 |

I co ważniejsze — kształt komórek dla osi `[0,0,1]` przy ogólnym wzorze:

```
(0,2) = (((1-cos(q1))*0)+(sin(q1)*0))     matematycznie 0, ale NIE jest węzłem Constant(0)
(2,2) = ((1-cos(q1))+cos(q1))             matematycznie 1, ale NIE jest węzłem Constant(1)
```

To bezpośrednio dyktuje dwie rzeczy: uzasadnia fast path (§8.1) i wyznacza granice tego, co testy mogą asertować.

### 14.2 Pełne mnożenie 4×4 niszczy kanoniczną reprezentację ostatniego wiersza

**Doprecyzowanie po review:** poprzednie sformułowanie („niszczy jednorodny ostatni wiersz") było za mocne. Mnożenie 4×4 **zachowuje poprawność matematyczną** — wyrażenie `0·R₀₀ + 0·R₁₀ + 0·R₂₀ + 1·0` jest równe zeru. Problem jest reprezentacyjny: nie jest to **kanoniczny węzeł `Constant(0)`**, więc `isZero()` zwraca `false`, a żaden kolejny etap nie wie, że komórka jest zerem.

Decyzja pozostaje ta sama — nie wykonywać pełnego `4×4 · 4×4`, tylko składać bloki bezpośrednio.

Zmierzone dla revolute (oś `[1,2,3]` znormalizowana, origin z `joint_4` KR4):

| Konstrukcja | Węzły DAG | `(3,0)` to zero? | `(3,3)` to jedynka? |
|---|---|---|---|
| pełne mnożenie 4×4 | 111 | **NIE** (drzewo, nie `Constant(0)`) | tak |
| mnożenie tylko bloku 3×3 | 96 | **TAK** | tak |

Rozmiar to drobiazg (14%). Istotne jest to, co dzieje się z ostatnim wierszem. Przy pełnym mnożeniu:

```
A(3,0) = To(3,0)·Tm(0,0) + To(3,1)·Tm(1,0) + To(3,2)·Tm(2,0) + To(3,3)·Tm(3,0)
       =    0·Rm₀₀      +    0·Rm₁₀       +    0·Rm₂₀       +    1·0
```

`Rm` jest symboliczne, więc `0·Rm₀₀` **nie zwija się** (brak anihilatora) i cała komórka zostaje drzewem `Add(Add(Mul(0,Rm₀₀), Mul(0,Rm₁₀)), Mul(0,Rm₂₀))` zamiast `Constant(0)`.

Konsekwencja praktyczna: **wymagany przez prompt test `PreservesHomogeneousLastRow` failowałby dla każdego aktywnego jointu** — nie dlatego, że wynik jest matematycznie zły, tylko dlatego, że `isZero()` nie rozpozna drzewa jako zera. Nie da się tego obejść inaczej niż nie wykonując tego mnożenia.

**Rozstrzygnięcie: konstrukcja z wykorzystaniem struktury transformacji jednorodnej.** Dla `T_origin = [[R_o, p_o],[0,1]]`:

```
Revolute / Continuous:   T_motion = [[R_m, 0],[0,1]]
                         T = [[ R_o · R_m ,  p_o ],
                              [   0 0 0   ,   1  ]]

Prismatic:               T_motion = [[I, a·q],[0,1]]
                         T = [[   R_o    ,  p_o + R_o·(a·q) ],
                              [  0 0 0   ,         1        ]]

Fixed:                   T = T_origin
```

Mnożymy więc **wyłącznie bloki 3×3** (i dla prismatic jeden iloczyn macierz-wektor 3×1). Ostatni wiersz jest wpisywany bezpośrednio jako stałe — a właściwie przychodzi za darmo z `SymbolicMatrix::identity()`, od której zaczynamy budowę.

To nie jest uproszczenie algebraiczne, tylko **bezpośrednia konstrukcja znanej struktury**. W projekcie, który świadomie zachowuje symboliczne mnożenia przez zero, jest to szczególnie uzasadnione — inaczej sami produkowalibyśmy śmieci, których żaden istniejący mechanizm nie usunie.

**Konsekwencja dla `SymbolicRotation`:** alias `SymbolicMatrix<3,3>` przestaje być nieużywany. Bloki obrotu `R_o` i `R_m` powstają jako `SymbolicRotation`, są mnożone jako 3×3 i dopiero wynik jest osadzany w 4×4. Zmienia to odpowiedź na otwarte pytanie 4 z v1 — wtedy rekomendowałem budowanie od razu w 4×4; teraz typ pośredni ma jasne uzasadnienie.

### 14.3 Strategia asercji w testach

| Narzędzie | Kiedy używać | Kiedy NIE |
|---|---|---|
| `sameNode` | gdy oczekujemy **dokładnie tego samego obiektu** — np. że symbol w macierzy to ten sam węzeł, co zbudowany z `joint.variable->name` | do porównywania wartości; dwie niezależne `constant(1.0)` to różne węzły |
| `isZero` / `isOne` / `constantValue` | dla komórek, które **na pewno** zwinęły się do stałej — ostatni wiersz, przekątna w fast path, translacja origin | dla komórek policzonych ogólnym Rodriguesem — tam „zero" jest drzewem, nie stałą |
| `type()` | do sprawdzenia **kształtu** — czy komórka to `Sin`, `Cos`, `Multiply`, `Add` | jako jedyna asercja; typ nie mówi nic o zawartości |
| `structurallyEqual` | do porównania komórki z **oczekiwanym drzewem zbudowanym w teście z tej samej fabryki** | gdy oczekiwane drzewo jest wielkie — test staje się nieczytelny i kruchy |
| ewaluacja numeryczna | **niedostępna** — `ExpressionEvaluator` nie istnieje i jest poza zakresem | — |

**Zasada nadrzędna:** testy sprawdzają **strukturę w miejscach, które niosą semantykę** (obecność `sin(q)`/`cos(q)` we właściwych komórkach, znaki, ostatni wiersz, kolumna translacji), a **nie** całe drzewo. Test asertujący 43-węzłową macierz co do węzła byłby nieczytelny i psułby się przy każdej zmianie reguł normalizacji, nie wykrywając przy tym niczego więcej.

**Odnotowanie na przyszłość:** brak ewaluatora jest tu realnym ograniczeniem. Najmocniejszym testem obrotu byłoby podstawienie `q = π/2` i porównanie z numeryczną macierzą — ale to wymaga `ExpressionEvaluator`, który jest poza zakresem tego etapu. Wpisuję to jako otwartą kwestię (§18), bo znacząco wzmocniłoby testy tego i kolejnych komponentów.

## 15. Plan zmian w plikach

### Dodane

| Plik | Zawartość |
|---|---|
| `src/ik_equations/builders/JointTransformBuilder.cpp` | implementacja + helpery w anonimowej przestrzeni nazw |
| `tests/test_joint_transform_builder.cpp` | testy z §16 |

### Zmienione

| Plik | Zmiana |
|---|---|
| `src/ik_equations/builders/JointTransformBuilder.hpp` | konstruktor przyjmujący `ExpressionFactory`, pole `factory_`, include `ExpressionFactory.hpp`. **Sygnatura `build()` bez zmian** |
| `CMakeLists.txt` | +1 linia: `src/ik_equations/builders/JointTransformBuilder.cpp` |
| `tests/CMakeLists.txt` | +1 linia: `test_joint_transform_builder.cpp` |

### Prerequisite — osobny, poprzedzający proposal walidacji loadera

Zakres poszerzony po review. Samo dołożenie `isfinite` **nie wystarczy** — parser wektorów po cichu naprawia uszkodzone dane (§2.5), a `isfinite` na wyniku `[1,0,0]` z wejścia `"1 abc 3"` niczego nie wykryje.

| Plik | Zmiana |
|---|---|
| `src/kinematics/robot_model_loader.cpp` | `parse_xyz` → rygorystyczny `std::expected<Vec3, LoadError> parseVector3(...)`: dokładnie trzy wartości, każdy token poprawny, brak śmieci po trzeciej, skończoność. Dodatkowo: normalizacja osi, odrzucenie osi zerowej dla jointów aktywnych, domyślna oś `[1,0,0]` zgodna z URDF (§2.6) |
| `src/kinematics/robot_model.hpp` | zmiana domyślnej `axis` z `{0,0,1}` na `{1,0,0}`; komentarz „unit vector" staje się egzekwowanym kontraktem |
| `src/kinematics/robot_model_loader.hpp` | nowe wartości `LoadError` (np. `malformed_vector`, `invalid_axis`) |
| `src/ik_equations/UrdfModelLoader.cpp` | opisy nowych błędów w `describe()` |
| `tests/test_urdf_model_loader.cpp` | testy z §16.4 |

**Ten proposal nie może zostać wdrożony przed tamtym** — asercje w `JointTransformBuilder` opisywałyby inwariant, którego nic nie ustanawia.

### Bez zmian

Cała warstwa symboliczna, `KinematicChainBuilder`, model danych, `ForwardKinematicsBuilder.hpp`, `IkEquationBuilder`.

## 16. Szczegółowy plan testów

Plik `tests/test_joint_transform_builder.cpp`, suite `JointTransformBuilderTest`. Wejścia budowane jako ręczne `KinematicJoint` — plik URDF nie umie wyrazić wszystkich przypadków (oś zerowa, prismatic, continuous), a ręczna konstrukcja izoluje ten komponent od loadera.

### 16.1 Fixed i origin

| Test | Wejście | Oczekiwanie | Wykrywany błąd |
|---|---|---|---|
| `BuildsIdentityForFixedJointWithoutOrigin` | `Fixed`, origin zerowy | macierz jednostkowa: `isOne` na przekątnej, `isZero` poza | mnożenie przez coś niepustego; złe wypełnienie `identity()` |
| `BuildsTranslationFromFixedJointOrigin` | `Fixed`, `translation = [1,2,3]`, `rpy = 0` | `(0,3)=1`, `(1,3)=2`, `(2,3)=3`; blok 3×3 jednostkowy | translacja w złej kolumnie/wierszu; transpozycja |
| `BuildsRotationFromFixedJointRpy` | `Fixed`, `translation = 0`, `rpy = (0, 0, π/2)` (czysty yaw) | `(0,0)≈0`, `(0,1)≈−1`, `(1,0)≈1`, `(1,1)≈0`, `(2,2)=1` — czyli `Rz(π/2)` | pomylenie osi roll/pitch/yaw; użycie stopni zamiast radianów |
| `MapsRollPitchYawToCorrectAxes` | trzy osobne jointy: `rpy=(π/2,0,0)`, `(0,π/2,0)`, `(0,0,π/2)` | każdy daje odpowiednio `Rx`, `Ry`, `Rz` | zamiana `roll↔yaw` — najczęstsza pomyłka przy RPY, niewidoczna gdy testuje się tylko jedną oś |
| `ComposesRpyInFixedAxisOrder` | `rpy = (π/2, 0, −π/2)` (dane z `joint_4` KR4) | dokładnie `[[0,0,−1],[−1,0,0],[0,1,0]]` (§5.3) | odwrócenie kolejności na `Rx·Ry·Rz`. **Przypadek dobrany tak, by konwencje dawały różne wyniki** — `rpy=(π,0,0)` by tego nie wykrył |
| `CombinesTranslationAndRotationInCorrectOrder` | `translation = [1,0,0]`, `rpy = (0,0,π/2)` | kolumna translacji **dokładnie** `[1,0,0]`, nie `Rz(π/2)·[1,0,0] = [0,1,0]` | `Rotation · Translation` zamiast `Translation · Rotation` |

### 16.2 Revolute — osie osiowe i dowolne

**Wszystkie osie w testach tego pliku podawane są już jako jednostkowe.** Po wyborze Pakietu A jednostkowość osi jest **precondition** `JointTransformBuilder`, więc test karmiący go osią `[0,0,5]` łamałby kontrakt, który sam dokument ustanawia. Osie nieosiowe zapisujemy wprost:

```cpp
const double invSqrt14 = 1.0 / std::sqrt(14.0);
joint.axis = {1.0 * invSqrt14, 2.0 * invSqrt14, 3.0 * invSqrt14};
```

Test `NormalizesNonUnitAxis` **nie należy do tego pliku** — przeniesiony do `UrdfModelLoaderTest` (§16.4), gdzie normalizacja faktycznie się dzieje.

| Test | Wejście | Oczekiwanie | Wykrywany błąd |
|---|---|---|---|
| `BuildsRevoluteJointAroundXAxis` | `axis=[1,0,0]`, `q1` | `(1,1)`,`(2,2)` to `Cos`; `(2,1)` to `Sin`; `(1,2)` to `Negate(Sin)`; `(0,0)` to `Constant(1)` | obrót wokół złej osi |
| `BuildsRevoluteJointAroundYAxis` | `axis=[0,1,0]` | `(0,2)` to `Sin`, `(2,0)` to `Negate(Sin)` — **znak odwrotny niż przy X i Z** | skopiowanie wzoru X/Z na Y bez korekty znaku; klasyczna pomyłka, bo Ry ma inny układ minusów |
| `BuildsRevoluteJointAroundZAxis` | `axis=[0,0,1]` | `(0,0)`,`(1,1)` to `Cos`; `(1,0)` to `Sin`; `(0,1)` to `Negate(Sin)`; `(2,2)=1` | j.w. |
| `BuildsRevoluteJointAroundArbitraryAxis` | `axis = normalize([1,2,3])` = `[0.2673, 0.5345, 0.8018]` — **wszystkie trzy składowe niezerowe**, podane już jako jednostkowe | `(0,1)` to `Subtract`, `(1,0)` to `Add`, oba z członem `sin(q)·0.8018`; ostatni wiersz `[0,0,0,1]` | zła macierz Rodriguesa; pominięcie członu `s·z`; wejście w fast path dla osi nieosiowej |
| `AxisAlignedFastPathBuildsCanonicalZRotation` | oś `[0,0,1]` przez `build()`, porównana z ręcznie zbudowanym `Rz(q)` | `structurallyEqual` na wszystkich komórkach | fast path budujący coś innego niż kanoniczne `Rz` |
| `BuildsRevoluteJointAroundNegativeZAxis` | `axis=[0,0,-1]` | `(0,1)` to `Sin`, `(1,0)` to `Negate(Sin)` — znaki odwrotne niż dla `[0,0,1]` | pominięcie znaku przy fast path dla osi ujemnych (§8.1) |

**Dlaczego `[1,2,3]`, a nie `[1,1,0]` (korekta z review).** V1 proponowała oś `[1,1,0]` i twierdziła, że komórki `(0,1)` i `(1,0)` będą się różnić. To było **matematycznie błędne**: dla `z = 0` mamy `R01 = t·x·y − s·z = t·x·y` oraz `R10 = t·x·y + s·z = t·x·y` — **wartości są identyczne**.

Uruchomiłem obie osie, żeby sprawdzić, co faktycznie zobaczy test:

```
oś [1,1,0] → znorm. [0.7071, 0.7071, 0]
  R01 = ((((1-cos(q))*0.707107)*0.707107)-(sin(q)*0))
  R10 = ((((1-cos(q))*0.707107)*0.707107)+(sin(q)*0))
  structurallyEqual(R01, R10) → NIE

oś [1,2,3] → znorm. [0.2673, 0.5345, 0.8018]
  R01 = ((((1-cos(q))*0.267261)*0.534522)-(sin(q)*0.801784))
  R10 = ((((1-cos(q))*0.267261)*0.534522)+(sin(q)*0.801784))
  structurallyEqual(R01, R10) → NIE
```

Niuans, który czyni `[1,1,0]` jeszcze gorszym wyborem, niż wynikało z review: `structurallyEqual` zwraca `NIE` **w obu przypadkach** — ale dla `[1,1,0]` wyłącznie dlatego, że jeden węzeł to `Subtract`, a drugi `Add`. Człon `s·z` jest tam mnożeniem przez zero i nie niesie żadnej informacji. Test przeszedłby więc także dla implementacji, która **całkowicie pomija** człon `s·z` — czyli dawałby fałszywe poczucie bezpieczeństwa. Przy `[1,2,3]` współczynnik `0.801784` jest realnie obecny z przeciwnymi znakami i test faktycznie sprawdza wzór Rodriguesa.

Druga korekta: v1 twierdziła, że komórka `(0,0)` „zawiera stałą `0.7071`". Strukturalnie prawda (`multiply(multiply(t, 0.7071), 0.7071)`), ale mylące — **matematyczny współczynnik przy `t` to `x² = 0.5`**. Test nie powinien asertować „obecności stałej `0.7071`" jako dowodu poprawności, bo to mówi o zapisie, nie o wartości.

### 16.3 Continuous i prismatic

| Test | Wejście | Oczekiwanie | Wykrywany błąd |
|---|---|---|---|
| `BuildsContinuousJointLikeRevolute` | dwa identyczne jointy różniące się tylko typem (`Revolute` vs `Continuous`), ta sama nazwa zmiennej | macierze `structurallyEqual` | osobna, rozjeżdżająca się ścieżka dla `Continuous`; brak obsługi → identyczność |
| `BuildsPrismaticJointTranslationAlongAxis` | `Prismatic`, `axis = normalize([1,2,0])` = `[0.4472, 0.8944, 0]` — podana już jako jednostkowa, origin zerowy | `(0,3)` = `Multiply(0.4472…, q)`, `(1,3)` = `Multiply(0.8944…, q)`, `(2,3)` = **`Constant(0)`, `isZero` = true**; blok 3×3 **jednostkowy**, bez `Sin`/`Cos` | potraktowanie prismatic jak obrotu. **Oś z dwiema niezerowymi składowymi** — przy `[0,0,1]` test przeszedłby nawet dla implementacji wpisującej `q` w jedną komórkę na sztywno |
| `PrismaticDoesNotRotate` | j.w. | żadna komórka bloku 3×3 nie jest `Sin` ani `Cos` | j.w., asercja od drugiej strony |

**Uwaga do `(2,3)` = `Constant(0)`:** ta asercja przechodzi **tylko** dzięki jawnej regule `scaledAxisComponent` z §7.1. Bez niej implementacja zbudowałaby `Multiply(Constant(0), Symbol(q))`, a `isZero` zwróciłoby `false` — zweryfikowane uruchomieniem. V1 tego proposalu zawierała tę asercję **bez** odpowiadającej jej reguły konstrukcyjnej, czyli plan testów przeczył planowi implementacji.

### 16.4 Zmienna, inwarianty, błędy

| Test | Wejście | Oczekiwanie | Wykrywany błąd |
|---|---|---|---|
| `UsesJointVariableName` | `Revolute`, `variable->name = "theta_custom"` | węzeł `Symbol` w macierzy ma nazwę `"theta_custom"` | wymyślanie nazwy z `index` zamiast czytania z `variable` |
| `PreservesHomogeneousLastRow` | **cztery jointy**: fixed, revolute (oś dowolna), continuous, prismatic — wszystkie z niezerowym origin | dla każdego: `(3,0)`,`(3,1)`,`(3,2)` to `isZero`, `(3,3)` to `isOne` | rozbicie struktury jednorodnej. **Sprawdzane dla wszystkich typów**, nie tylko fixed — prompt wprost tego wymaga |
| `IgnoresAxisForFixedJoint` | `Fixed`, `axis = [0,0,0]`, origin niezerowy | **sukces** — zwraca `T_origin` | walidowanie osi dla jointu, który jej nie używa; odrzucałoby poprawne dane (fixed jointy z realnych URDF-ów, §2.4 pkt 4) |

**Testy `RejectsActuatedJointWithoutVariable` i `RejectsZeroRotationAxis` — usunięte z tego pliku.** Po przyjęciu Pakietu A (§11) oba warunki są `assert`-owanymi preconditions, więc test wymagałby `EXPECT_DEATH` — mechanizmu świadomie odrzuconego w warstwie symbolicznej i nieużywanego nigdzie w repo. Kontrakt zostaje udokumentowany w nagłówku.

**Odpowiadające im testy przenoszą się do loadera** (proposal z §15), gdzie te same sytuacje są błędami domenowymi z `LoadError` i testują się zwykłym sprawdzeniem `expected`:

```
tests/test_urdf_model_loader.cpp:
    RejectsZeroAxisForActuatedJoint
    RejectsNonFiniteAxis
    RejectsNonFiniteOrigin
    RejectsMalformedVectorText          ("1 abc 3", "1 2", "1 2 3 4")
    AcceptsZeroAxisForFixedJoint
    NormalizesNonUnitAxis
    DefaultsMissingAxisToXPerUrdfSpec
```

To jest praktyczna konsekwencja przeniesienia walidacji na granicę: przypadki błędne testują się tam, gdzie dane wchodzą, a nie trzy etapy dalej.

### 16.5 Test integracyjny

| Test | Wejście | Oczekiwanie |
|---|---|---|
| `BuildsAllKr640ChainJoints` | wszystkie 7 jointów z `kr640.urdf` przez `UrdfModelLoader` + `KinematicChainBuilder` | każdy buduje się bez błędu; 6 zawiera symbol o oczekiwanej nazwie (`q1`..`q6`), fixed nie zawiera żadnego symbolu; wszystkie mają poprawny ostatni wiersz |

Jeden test na realnych danych — żeby wyłapać rozjazd między ręcznie budowanymi `KinematicJoint` w pozostałych testach a tym, co faktycznie produkuje pipeline.

### 16.6 Potrzebny helper testowy

Oczekiwanie „macierz zawiera symbol `q1`" nie ma dziś czym zostać sprawdzone — warstwa symboliczna nie udostępnia takiej funkcji, a rekurencyjne przejście drzewa trzeba napisać ręcznie:

```cpp
// Tylko w pliku testowym — NIE w API produkcyjnym.
bool containsSymbol(const Expression& expression, std::string_view name);
```

Implementacja: `std::visit` po `expression.node().value`, zejście w dzieci dla węzłów binarnych i unarnych, porównanie `SymbolNode::name` w liściach.

**Świadomie zostaje w teście, nie w `Expression.hpp`.** Jedynym dzisiejszym konsumentem jest ten jeden test; wyniesienie do publicznego API byłoby projektowaniem pod hipotetycznego użytkownika. Gdy pojawi się drugi (prawdopodobnie `ForwardKinematicsBuilder` albo przyszły simplifier — oba będą chciały wiedzieć, od których zmiennych zależy wyrażenie), wtedy warto to przenieść wraz z przemyślanym API, np. zwracającym zbiór wszystkich symboli zamiast odpowiadającym na pytanie o jeden.

Ten sam helper przyda się w `UsesJointVariableName` i `AssignsSymbolsOnlyToActuatedJoints`-podobnych asercjach — czyli powstaje raz, na górze pliku testowego, obok `using`-deklaracji.

## 17. Ryzyka i trade-offy

| Ryzyko | Ocena | Mitygacja |
|---|---|---|
| Fast path i ogólny Rodrigues rozjadą się semantycznie | Realne — dwie ścieżki dla tej samej matematyki. **Dziś nietestowalne**: `structurallyEqual` musi zwrócić `false`, bo drzewa celowo mają inny kształt | Częściowa: obie ścieżki testowane osobno wobec postaci kanonicznej. Pełna weryfikacja wymaga `ExpressionEvaluator` (§18) — ryzyko przyjęte świadomie |
| Odwrotna kolejność mnożenia przechodzi testy | Realne, jeśli dane testowe przypadkiem komutują | Testy kolejności używają **jawnie dobranych** danych rozróżniających (§5.3, §16.1); w opisach zaznaczone, które przypadki NIE nadają się |
| Duże drzewa dla osi nieosiowych (43 węzły/joint) | Akceptowalne — realne roboty używają osi osiowych; symplifikacja to zadanie przyszłego `EquationSimplifier` | — |
| Brak ewaluatora ogranicza siłę testów | Realne — nie da się sprawdzić „`R(q=π/2)` równa się oczekiwanej macierzy numerycznej" | Testy strukturalne pokrywają semantykę; ewaluator jako otwarta kwestia (§18) |
| `assert` nie chroni w Release (Pakiet A) | Realne do czasu walidacji w loaderze | Główny powód, dla którego §11 wymaga Twojej decyzji |
| Tolerancja `1e-12` jako liczba wzięta z sufitu | Niska waga — oddziela przypadek zdegenerowany, nie porównuje bliskości | Udokumentowana wraz z uzasadnieniem (§9.3) |

## 18. Otwarte pytania

Wszystkie cztery pytania z v1 zostały rozstrzygnięte w review. Zostaje jedno, świadomie odłożone:

**`ExpressionEvaluator` — kolejność prac.** Brak ewaluatora jest realnym ograniczeniem siły testów, i to w konkretnym, nazwanym miejscu: **równoważności fast path z ogólnym wzorem Rodriguesa nie da się dziś przetestować**. `structurallyEqual` **musi** zwrócić `false`, bo drzewa celowo mają inny kształt (`cos(q)` kontra `((1-cos(q))·0)·0 + cos(q)`). Jedyny uczciwy test to podstawienie kilku wartości `q` i porównanie liczb — czyli ewaluator.

Do tego czasu obie ścieżki są testowane osobno, każda wobec oczekiwanej postaci kanonicznej. To wykryje błąd w każdej z nich z osobna, ale **nie wykryje rozjechania się ich semantyki**. Odnotowuję jako świadomie przyjęte ryzyko (§17), nie jako przeoczenie.

### Rozstrzygnięte w review (były otwarte w v1)

| Pytanie z v1 | Rozstrzygnięcie |
|---|---|
| Gdzie walidować geometrię | **W loaderze** — walidacja i normalizacja na wejściu danych (§11.2) |
| Fast path dla osi ujemnych | **Tak** — `Rotation(−Z, q) = Rotation(Z, −q)`, tanie i objęte testem `BuildsRevoluteJointAroundNegativeZAxis` (§8.1, §16.2) |
| Czy używać `SymbolicRotation` jako typu pośredniego | **Tak** — wymusza to konstrukcja strukturalna z §14.2; v1 rekomendowała odwrotnie, przed odkryciem problemu z ostatnim wierszem |

## 19. Rekomendacja końcowa

```
approve
```

**pod warunkiem wdrożenia w kolejności:** najpierw proposal walidacji loadera (§15, „Prerequisite"), potem implementacja tego komponentu.

### Rozstrzygnięte na podstawie kodu, pomiaru i specyfikacji URDF

Kolejność `T_origin · T_motion`; konwencja `R_rpy = Rz(yaw)·Ry(pitch)·Rx(roll)`; `Translation · Rotation` dla origin; wzór Rodriguesa dla dowolnej osi; fast path dla osi osiowych **wraz z ujemnymi** (uzasadniony pomiarem 33→7 węzłów i nieczytelnością komórek); `Continuous` traktowany jak `Revolute`; model prismatic z regułą `scaledAxisComponent`; normalizacja w loaderze; inwariant zmiennej; `assert` jako model błędów; fabryka jako pole; podział helperów; **konstrukcja strukturalna zamiast mnożenia 4×4**; strategia asercji w testach.

### Co zmieniła rewizja v2

| Punkt review | Charakter | Efekt |
|---|---|---|
| oś `[1,1,0]` w teście arbitrary-axis | **błąd matematyczny w v1** | → `[1,2,3]`; wyjaśnione, dlaczego `[1,1,0]` dawałaby fałszywe poczucie bezpieczeństwa |
| `M(2,3) = Constant(0)` dla prismatic | **sprzeczność testu z implementacją w v1** | → jawna reguła `scaledAxisComponent` (§7.1) z uzasadnieniem, czym różni się od odrzuconego anihilatora |
| `FastPathMatchesGeneralFormula` | **test nie testował tego, co deklarował** | → przemianowany; niemożliwość porównania odnotowana jako otwarta kwestia (§18) |
| walidacja loadera szersza niż `isfinite` | uzupełnienie | → zmierzone 7 patologii `parse_xyz`, w tym zerowanie całego wektora przy przepełnieniu (§2.5) |
| domyślna oś `[0,0,1]` vs URDF `[1,0,0]` | **niezgodność ze specyfikacją** | → nowa §2.6, naprawa w proposalu loadera |
| rozdzielenie walidacji i normalizacji | słaby argument w v1 | → normalizacja przeniesiona do loadera; mój argument o „normie potrzebnej dla fast path" był błędny (§9.2) |
| pełne mnożenie 4×4 | **wykryty problem reprezentacji** | → zmierzone: ostatni wiersz przestaje być kanonicznym `Constant(0)`, wymagany test by failował; konstrukcja blokowa (§14.2) |

### Uwaga do zaakceptowanej decyzji o fabryce

Przyjmuję sprostowanie: przechowywanie `ExpressionFactory` jako pola **nie daje dziś realnej wstrzykiwalności** — to konkretny typ bezstanowy bez interfejsu polimorficznego, więc lokalna fabryka byłaby równie testowalna. Argument z v1 był przesadzony. Zostawiam pole wyłącznie dla spójności przyszłego API, i tak to teraz opisuję — nie jako korzyść, której nie ma.

Po zatwierdzeniu przygotuję proposal walidacji loadera, a po jego wdrożeniu — proposal implementacyjny tego komponentu z pełnym kodem.
