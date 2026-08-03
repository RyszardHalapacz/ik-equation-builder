# Proposal: `ForwardKinematicsBuilder` — architektura (v2)

## Prompt

> Następny komponent to `ForwardKinematicsBuilder`. Roadmapa mówi wprost, że po `JointTransformBuilder` należy zbudować `KinematicChain → SymbolicTransform FK`, czyli `T_base_tool(q) = T_joint_1(q1) · ... · T_joint_n(qn)`. Nie przechodziłbym od razu do kodu, bo istnieje jedna istotna decyzja architektoniczna: jak mnożyć transformacje bez niszczenia kanonicznego ostatniego wiersza.

**Rewizja v2** po werdykcie `REQUEST CHANGES`. Review zaakceptowało kierunek (strukturalne `multiplyTransforms`, pusty łańcuch → identity, zwijanie lewostronne, API z fabryką, brak `std::expected`, wariant B, memoizacja ewaluatora) i zgłosiło sześć zastrzeżeń. Wszystkie sześć przyjmuję — **cztery z nich to realne błędy w v1**, nie kwestie gustu. Co się zmieniło: §19.

## Status weryfikacji

**Ten dokument nie zawiera kodu produkcyjnego do naniesienia.** Proces: architektura → review → proposal implementacyjny z pełnym kodem → review → kod na dysku.

Wszystkie liczby pochodzą z pomiaru, nie z oszacowania. Trzy jednorazowe programy (`fk_probe.cpp`, `fk_probe2.cpp`, `fk_probe3.cpp`, poza repo, w `_claude_scratch`) zlinkowane z `libkinemaforge_ik.a`. Review słusznie zażądało powtórzenia pomiaru po dodaniu brakujących fast pathów — wyniki w §6.5, i **jeden z nich jest niewygodny dla mojej v1** (§6.6).

---

## 1. Cel

```
KinematicChain  →  SymbolicTransform
T_base_tool(q) = T_1(q1) · T_2(q2) · ... · T_n(qn)
```

Ostatni builder przed uzyskaniem pełnej symbolicznej macierzy FK. Po nim projekt po raz pierwszy wyprowadza równanie kinematyki prostej z pliku URDF end-to-end.

---

## 2. Stan obecny repozytorium

### 2.1 Istniejący nagłówek

`src/ik_equations/builders/ForwardKinematicsBuilder.hpp` deklaruje `build(chain, transformBuilder) -> SymbolicTransform`. Brak `.cpp`, brak wpisu w CMake, brak testów. Nagłówek pochodzi z początkowego szkieletu i **nie przeszedł review** — traktuję go jako propozycję, nie zatwierdzony kontrakt.

### 2.2 Co gwarantuje wejście

`KinematicChain` niesie `joints` uporządkowane **od bazy do narzędzia**; jointy `fixed` zostają (niosą offsety); tylko aktywne mają `variable`; `baseLink == toolLink` daje **pusty łańcuch, nie błąd**. Pinowane 17 testami.

### 2.3 Co gwarantuje `JointTransformBuilder`

Zatwierdzony, zaimplementowany, 22 testy. W wyniku gwarantuje:

- ostatni wiersz **dokładnie** `[0 0 0 1]` — komórki rozpoznawane przez `isZero`/`isOne`
- blok obrotu wolny od śmieci `0 · sin(q)`

To jest inwariant, którego `ForwardKinematicsBuilder` **nie ma prawa zepsuć**. Cała trudność tego komponentu sprowadza się do tego jednego zdania.

### 2.4 Warstwa symboliczna — dostępne operacje i ich pułapka

`SymbolicMatrix.hpp` daje `multiply(lhs, rhs, factory)`, z komentarzem, który jest ostrzeżeniem skierowanym dokładnie do tego komponentu:

> Note that `A * identity()` does NOT give back A's cells for symbolic input: `x * 0` is deliberately kept, so the off-diagonal products survive as `Multiply` nodes.

`ExpressionFactory` zwija stałe i elementy neutralne, ale **nie ma** reguły `x · 0 → 0` (zatwierdzona decyzja: utrata informacji o dziedzinie). Tej decyzji ten proposal nie rusza.

**Zmierzone zachowanie fabryki** (istotne dla §6.4 — czy potrzebny jest czwarty fast path):

| Reguła | Zwija |
|---|---|
| `x + 0 → x` | tak |
| `0 + x → x` | **tak** — zwijanie jest symetryczne |
| `1 · x → x` | tak |
| `x − 0 → x` | tak |

---

## 3. Zakres i non-goals

**W zakresie:** złożenie uporządkowanego łańcucha w jedną `SymbolicTransform`; strukturalne `multiplyTransforms` zachowujące kanoniczny ostatni wiersz; komplet fast pathów dla elementów neutralnych; testy na obu poziomach (warstwa symboliczna i builder).

**Poza zakresem — jawnie:** upraszczanie trygonometrii, rozwijanie/faktoryzacja, globalne `x · 0 → 0`, CSE, „naprawianie" drzew po fakcie, numeryczna walidacja FK (wymaga `ExpressionEvaluator`), transformacja odwrotna, jakobian, IK, fasada.

---

## 4. Semantyka

### 4.1 Kolejność mnożenia

`T_i` przekształca układ rodzica jointu `i` w układ jego dziecka. Ponieważ dziecko jointu `i` jest rodzicem jointu `i+1`, iloczyn teleskopuje i daje przekształcenie bazy w narzędzie. Kolejność odwrotna nie ma sensu geometrycznego.

Akumulacja **lewostronna**, `result` zawsze po lewej:

```
result = Identity
for (joint : chain.joints)
    result = multiplyTransforms(result, transformBuilder.build(joint), factory_)
```

### 4.2 Pusty łańcuch → `Identity`

Trzy zgodne argumenty: pusty iloczyn jest elementem neutralnym; `KinematicChainBuilder` traktuje `baseLink == toolLink` jako **sukces**, więc przeciwny kontrakt tutaj byłby sprzeczny; wynika za darmo z akumulatora zainicjowanego na `Identity`, bez ani jednej gałęzi `if`.

---

## 5. Kluczowa decyzja: `multiplyTransforms`, nie `multiply(4×4)`

### 5.1 Struktura blokowa

```
[ R_a  p_a ]   [ R_b  p_b ]     [ R_a·R_b   p_a + R_a·p_b ]
[  0    1  ] · [  0    1  ]  =  [    0            1       ]
```

Ostatni wiersz **składamy ręcznie**, startując od `SymbolicTransform::identity()` — tak jak zatwierdzony `assembleTransform` w `JointTransformBuilder`.

### 5.2 Dlaczego naiwne mnożenie 4×4 jest wykluczone

To argument o **poprawności reprezentacji**, nie o wydajności. Naiwne `multiply` liczy `(3,0)` jako `0·R00 + 0·R10 + 0·R20 + 1·0` — matematycznie zero, ale bez `x · 0 → 0` nic się nie zwija.

Zmierzone:

| Komórka | Ma być | `kr640` naiwnie | `kr4_r600` |
|---|---|---|---|
| `(3,0)` | `Constant(0)` | `Add`, **31 738 węzłów**, `isZero() == false` | **94 112 węzłów** |
| `(3,3)` | `Constant(1)` | `Add`, **31 738 węzłów**, `isOne() == false` | **94 110 węzłów** |

Trzydzieści tysięcy węzłów, żeby wyrazić stałą 1. Kaskada konsekwencji: `isIdentityTransform` przestaje działać (więc fast pathy z §6 przestają się wyzwalać, co pogłębia problem); następne etapy nie mogą założyć struktury bez ewaluacji numerycznej; generator kodu wyemitowałby te 31 738 węzłów; cały wysiłek `JointTransformBuilder` zostaje zniweczony w pierwszym mnożeniu.

---

## 6. Fast pathy dla elementów neutralnych

**To sekcja najmocniej przepisana w v2.** v1 miała jeden fast path i odrzucała trzy kolejne. Review wykazało, że to błąd — i pomiar to potwierdza.

### 6.1 Zasada nadrzędna, której v1 nie nazwała

Review sformułowało to lepiej niż mój oryginalny dokument:

> To nie jest niepotrzebna optymalizacja. To zachowanie elementu neutralnego na poziomie reprezentacji.

I to jest właściwa rama. W warstwie bez `x · 0 → 0` **żadna** tożsamość `A · I = A`, `I · A = A`, `R · 0 = 0` nie zachodzi strukturalnie. Jeżeli `multiplyTransforms` ma być operacją wielokrotnego użytku w warstwie symbolicznej, to musi te tożsamości **odtwarzać jawnie** — inaczej dostarcza operację, która łamie prawa, jakich wołający ma prawo od mnożenia oczekiwać.

W v1 uzasadniałem pojedynczy fast path zyskiem w liczbie węzłów. To było uzasadnienie właściwej decyzji **niewłaściwym argumentem**, i dlatego trzy pozostałe przypadki wypadły z dokumentu — nie „opłacały się" na dwóch konkretnych robotach. §6.6 pokazuje, jak zły to był miernik.

### 6.2 Komplet fast pathów — przyjęty algorytm

```
multiplyTransforms(lhs, rhs, factory):

    assert hasCanonicalHomogeneousLastRow(lhs)
    assert hasCanonicalHomogeneousLastRow(rhs)

    if isIdentityTransform(lhs):  return rhs          // I · T = T
    if isIdentityTransform(rhs):  return lhs          // T · I = T

    R_lhs, p_lhs = blocks(lhs)
    R_rhs, p_rhs = blocks(rhs)

    if      isIdentityRotation(R_lhs):  R = R_rhs     // I · R = R
    else if isIdentityRotation(R_rhs):  R = R_lhs     // R · I = R
    else:                               R = multiply(R_lhs, R_rhs, factory)

    if      isZeroVector(p_rhs):        rotated = zeros()   // R · 0 = 0
    else if isIdentityRotation(R_lhs):  rotated = p_rhs     // I · p = p
    else:                               rotated = multiply(R_lhs, p_rhs, factory)

    p = p_lhs + rotated                                // x + 0 → x, w fabryce

    return assemble([R p; 0 1])
```

Zgodnie z algorytmem podanym w review. Każda gałąź odpowiada jednej tożsamości algebraicznej, którą warstwa symboliczna gubi.

### 6.3 Trzy przypadki, które v1 pominęła — zmierzone

Wszystkie trzy zweryfikowane syntetycznie, niezależnie od danych robota.

**`T · I` (review §1).** `T` = obrót wokół Z o `q1`, `I` = joint `fixed` o zerowym origin:

| | komórki różne od `T` | węzły |
|---|---:|---:|
| bez fast pathu | **8 / 16** | 76 |
| z fast pathem | 0 / 16 | 21 (= dokładnie `T`) |

**`T · Trans` — `R_b == I`, `p_b ≠ 0` (review §2).** Czysto translacyjny joint `fixed` po prawej:

| | komórki obrotu różne od `R_T` | węzły |
|---|---:|---:|
| bez fast pathu | **6 / 9** | 76 |
| z fast pathem | 0 / 9 | 38 |

**`A · T` — `p_b == 0` (review §3).** Joint czysto obrotowy po prawej, `A` z nietrywialnym `R_a` i `p_a = [0,35; 0; 0]`:

| | komórki kolumny translacji równe `p_a` | węzły kolumny |
|---|---:|---:|
| bez fast pathu | **1 / 3** | 22 |
| z fast pathem | 3 / 3 | 3 |

Review ma rację również co do wagi trzeciego przypadku: jointy czysto obrotowe to **podstawowy** przypadek robota przemysłowego, nie egzotyka. `kr640` ma dwa takie w łańcuchu (`joint_a5`, `joint_a6`, oba `xyz="0 0 0"`).

### 6.4 Czwarty fast path (`p_lhs == 0`) — **niepotrzebny**

Nasuwa się symetryczne pytanie o zerowy lewy wektor translacji. Odpowiedź: nie trzeba, bo `p = p_lhs + rotated` idzie przez fabrykę, a ta zwija **w obie strony** — zmierzone w §2.4: zarówno `x + 0 → x`, jak i `0 + x → x`.

Zapisuję to jawnie, żeby proposal implementacyjny nie dołożył czwartej, martwej gałęzi przez symetrię z pozostałymi.

### 6.5 Pomiar po zmianie — zgodnie z żądaniem review

`v1` = tylko fast path akumulatora. `v2` = komplet z §6.2.

| Robot | wariant | total | unique | depth |
|---|---|---:|---:|---:|
| `kr640` | v1 | 47 997 | 353 | 22 |
| `kr640` | **v2** | **21 882** | **281** | 22 |
| `kr4_r600` | v1 | 153 703 | 516 | 24 |
| `kr4_r600` | **v2** | **153 703** | **516** | 24 |

Trafienia fast pathów na łańcuch 7-jointowy:

| Robot | `lhs==I` | `rhs==I` | `R_lhs==I` | `R_rhs==I` | `p_rhs==0` | pełne 3×3 |
|---|---:|---:|---:|---:|---:|---:|
| `kr640` | 1 | 0 | 0 | 1 | 2 | 5 |
| `kr4_r600` | 1 | 0 | 0 | 0 | 0 | 6 |

Dla `kr640`: **−54% węzłów z krotnościami, −20% DAG-u**, i 12 z 16 komórek wyniku różni się strukturalnie od v1.

Pełny łańcuch od punktu wyjścia: **466 848 → 21 882 węzłów, czyli 21,3×**.

### 6.6 Wynik niewygodny dla v1 — i dlaczego wzmacnia review

Dla `kr4_r600` nowe fast pathy dają **dokładnie zero**. Identyczne liczby, 0 z 16 komórek różnych. Każdy joint tego robota ma jednocześnie niezerowe `rpy` i niezerową translację, więc żaden z trzech nowych warunków się nie wyzwala.

Gdybym utrzymał kryterium z v1 — „nie wyzwala się na badanych łańcuchach, więc pomijam" — odrzuciłbym te fast pathy **po raz drugi**, mając w ręku pomiar pokazujący 54% zysku na drugim robocie i strukturalną poprawność na wszystkich trzech testach syntetycznych.

To jest dokładnie ten błąd, który review nazwało w §1: mierzenie ogólnej reguły dwoma konkretnymi plikami URDF. Zysk w węzłach jest **efektem ubocznym**, nie uzasadnieniem. Uzasadnieniem jest to, że `T · I` ma zwracać `T`.

### 6.7 Prawy operand jednostkowy jest osiągalny na realnych danych

W v1 pisałem, że taki joint istnieje w `kr4_r600.urdf` (`base_link-base`, `fixed`, zerowe `xyz` i `rpy`), ale leży na martwej gałęzi względem `base_link → tool0`.

Sprawdziłem, co się dzieje, gdy o tę gałąź poprosić wprost:

```
KinematicChainBuilder::build(robot, "base_link", "base")
  → sukces, 1 joint: "base_link-base", type = Fixed
  → JointTransformBuilder daje transformację dokładnie jednostkową (16/16 komórek)
```

Czyli przypadek jest osiągalny **przez normalny pipeline, na danych, które już leżą w repo** — nie tylko syntetycznie. Daje to darmowy test integracyjny (§15.4) obok syntetycznego `RightIdentityReturnsLeftStructurally`.

### 6.8 Konsekwencja: łańcuch jednojointowy

Z fast pathu akumulatora wynika, że dla łańcucha jednojointowego wynik FK jest **strukturalnie identyczny** z wyjściem `JointTransformBuilder`. Zweryfikowane: wszystkie 16 komórek przechodzi `structurallyEqual`. Daje to tani test pinujący fast path (`SingleJointResultMatchesJointTransformBuilder`) — bez tej gałęzi test failuje, więc nikt jej nie usunie „jako mikrooptymalizacji".

---

## 7. Kontrakt `multiplyTransforms` — nowa sekcja (review §4)

Review trafnie zauważa lukę: `SymbolicTransform` to **wyłącznie alias** `SymbolicMatrix<4,4>`. Typ nie niesie żadnej gwarancji, że ostatni wiersz to `[0 0 0 1]` — a wzór blokowy z §5.1 jest poprawny **tylko** dla macierzy, które tę własność mają. Dla dowolnej macierzy 4×4 wzór jest po prostu fałszywy.

Alternatywa „funkcja przyjmuje dowolne 4×4" odpada z tego samego powodu, i review formułuje to poprawnie.

**Przyjęte: precondition udokumentowany i asertowany.**

```cpp
// Preconditions: both operands are canonical homogeneous transforms —
// last row exactly [0 0 0 1], recognised by isZero/isOne.
// Asserted, not validated: every producer in this project (JointTransformBuilder,
// SymbolicTransform::identity(), and this function itself) guarantees it.
bool hasCanonicalHomogeneousLastRow(const SymbolicTransform&) noexcept;
```

Trzy powody, dla których `assert` jest tu właściwy, a nie `std::expected`:

1. Wszyscy producenci transformacji w projekcie gwarantują tę własność: `JointTransformBuilder` (pinowane testem), `SymbolicTransform::identity()`, i sama `multiplyTransforms` (składa wynik od `identity()`). Kontrakt jest **domknięty** — wynik jest znów poprawnym wejściem.
2. Jest to ten sam wzorzec granicy, który zatwierdziliśmy dla `JointTransformBuilder`: walidacja na wejściu danych (loader), asercje w środku pipeline'u.
3. Naruszenie oznacza błąd programisty, nie błędne dane wejściowe — jedyna droga to ręcznie zbudowana macierz 4×4.

Weryfikacja: w pomiarze §6.5 asercja była aktywna na wszystkich 14 wywołaniach obu robotów i **nie została naruszona ani razu**.

**`isIdentityTransform` sprawdza wszystkie 16 komórek.** Review ma rację: nazwa obiecuje pełny test macierzy, więc sprawdzanie 12 komórek i milczące poleganie na precondition byłoby kontraktem rozjeżdżającym się z nazwą. Przy kanonicznych wejściach oba warianty są równoważne, więc kosztem jest cztery dodatkowe wywołania predykatu — cena bez znaczenia za nazwę, która nie kłamie.

---

## 8. Wariant B — umiejscowienie (zatwierdzone, doprecyzowane)

Review zatwierdziło wariant B i doprecyzowało formę:

> `SymbolicTransform.hpp` — deklaracje, `SymbolicTransform.cpp` — implementacja, zamiast rozbudowywania nagłówka funkcjami `inline`.

Przyjęte. Do warstwy symbolicznej trafiają:

```cpp
bool isIdentityTransform(const SymbolicTransform&) noexcept;
bool hasCanonicalHomogeneousLastRow(const SymbolicTransform&) noexcept;

SymbolicTransform multiplyTransforms(const SymbolicTransform& lhs,
                                     const SymbolicTransform& rhs,
                                     const ExpressionFactory& factory);
```

`isIdentityRotation` i `isZeroVector` pozostają szczegółem implementacyjnym w `.cpp` (anonimowa przestrzeń nazw), chyba że proposal implementacyjny wykaże potrzebę ich eksportu.

`SymbolicTransform.cpp` będzie **pierwszym** `.cpp` tej warstwy poza `Expression.cpp` i `ExpressionFactory.cpp` — trzeba go dopisać do `kinemaforge_ik` w `CMakeLists.txt`.

`JointTransformBuilder` pozostaje **nietknięty**. Koszt: `isIdentityRotation` istnieje w dwóch miejscach (tam prywatnie, tu w `.cpp` warstwy symbolicznej). Zapisuję jako świadomy dług; ekstrakcja to osobna, mechaniczna zmiana po wdrożeniu, gdy będą dwa realne miejsca użycia i testy po obu stronach.

---

## 9. Projekt API buildera

```cpp
class ForwardKinematicsBuilder
{
public:
    explicit ForwardKinematicsBuilder(ExpressionFactory factory = {});

    SymbolicTransform build(const KinematicChain& chain,
                            const JointTransformBuilder& transformBuilder) const;

private:
    ExpressionFactory factory_;
};
```

Dokładnie w kształcie zatwierdzonym dla `JointTransformBuilder`. Odrzucone: lokalna fabryka w `build()` (odbiera wołającemu kontrolę, niespójne), fabryka jako trzeci parametr metody (asymetria bez zysku), wyciąganie fabryki z `JointTransformBuilder` (wymagałoby gettera, czyli poszerzenia zatwierdzonego API dla wygody innego komponentu).

### 9.1 Dwie fabryki — do zaprotokołowania

W pipelinie będą dwie instancje `ExpressionFactory`. Dziś nieszkodliwe: klasa nie ma ani jednego pola, jej kopie są nierozróżnialne. Przestanie być nieszkodliwe, gdy fabryka zyska stan (cache hash-consingowy, licznik węzłów, konfigurację tolerancji) — wtedy dwie instancje to dwa rozłączne cache'e.

Nie proponuję nic robić teraz. Proponuję **zapisać w `STATUS.md`**, że jeśli fabryka zyska stan, fasada `IkEquationBuilder` musi wstrzyknąć jedną instancję do obu builderów. Konstruktory z argumentem już to umożliwiają — to jest powód, dla którego mają ten argument.

### 9.2 Model błędów — bez zmian

`build()` zwraca `SymbolicTransform` bezpośrednio. Bez `std::expected`, bez wyjątków, bez walidacji runtime.

Nie istnieje `KinematicChain`, dla którego to obliczenie mogłoby zawieść: łańcuch pochodzi z `KinematicChainBuilder`, który już odfiltrował błędy topologiczne przez `std::expected`; geometrię zwalidował loader; inwarianty per-joint asertuje `JointTransformBuilder`; pusty łańcuch jest poprawnym wejściem o poprawnym wyniku. `std::expected` oznaczałoby typ błędu bez ani jednej wartości.

---

## 10. Wewnętrzny podział

```
ForwardKinematicsBuilder::build(chain, transformBuilder)
  └─ result = Identity
     for (joint : chain.joints)
         result = multiplyTransforms(result, transformBuilder.build(joint), factory_)
     return result
```

Po przeniesieniu `multiplyTransforms` do warstwy symbolicznej builder jest **pętlą i niczym więcej**. To jest oczekiwany rezultat wariantu B, nie oznaka, że komponent jest zbędny: jego odpowiedzialnością jest kolejność i kompletność łańcucha, a nie algebra.

---

## 11. Granica wobec simplifiera

`ForwardKinematicsBuilder` i `multiplyTransforms` **nie mogą**: upraszczać trygonometrii, rozwijać/faktoryzować, wprowadzać ogólnego `x · 0 → 0`, robić CSE, modyfikować drzew po zbudowaniu.

Fast pathy z §6 **nie są** wyjątkiem od tej reguły. Simplifier patrzy na **zbudowane wyrażenie** i pyta, czy wolno je przepisać — to wymaga wiedzy o dziedzinach. Fast path patrzy na **stałą znaną przed zbudowaniem czegokolwiek** (`isZero`/`isOne` na komórkach, które już są stałymi) i wybiera, którego wyrażenia w ogóle nie budować. Nic nie jest przepisywane, bo nic nie zostało jeszcze zapisane.

Czwarte zastosowanie tej samej zasady w projekcie (po fast pathach osi głównych, jednostkowego `R_origin` i zerowej składowej osi w `JointTransformBuilder`). Warto traktować ją jako **ustaloną regułę projektu**, nie decyzję podejmowaną od nowa przy każdym komponencie.

---

## 12. Kierunek zwijania — zatwierdzony lewostronnie

Zmierzone oba (mnożenie macierzy jest łączne, więc drzewa różnią się, wynik nie):

| Robot | lewostronne (total / unique / depth) | prawostronne |
|---|---|---|
| `kr640` | 47 997 / 353 / 22 | 29 082 / 327 / 22 |
| `kr4_r600` | 153 703 / 516 / 24 | 105 362 / 509 / **29** |

*(pomiar na wariancie v1; różnica jakościowa nie zmienia się po dodaniu fast pathów)*

Prawostronne daje 39%/31% mniej węzłów z krotnościami, ale tylko **7%/1,4% mniej węzłów unikalnych**, i jest głębsze dla `kr4_r600`. Zatwierdzone lewostronne: różnica w `total` jest w dużej mierze artefaktem liczenia współdzielonych poddrzew (§13), zysk w rzeczywistym DAG-u jest poniżej progu istotności, a przepływ base → tool czyta się zgodnie z kolejnością `chain.joints`.

---

## 13. Konsekwencja dla `ExpressionEvaluator`

Rozjazd `total` vs `unique` jest ogromny: **21 882 vs 281** dla `kr640`, **153 703 vs 516** dla `kr4_r600`. Drzewo FK to silnie współdzielony DAG — `Expression` jest uchwytem na `shared_ptr<const ExpressionNode>`, więc `cos(q1)` użyty w dwunastu miejscach to **jeden** węzeł.

**`ExpressionEvaluator` musi memoizować po tożsamości węzła.** Bez tego ~154 tys. odwiedzin zamiast ~516 dla `kr4_r600` — trzy rzędy wielkości narzutu. Ten sam problem dotknie każdego przyszłego printera, simplifiera i generatora kodu.

Głębokość (22–24) jest bezpieczna dla rekurencji, więc memoizacja wystarczy; przechodzenie iteracyjne po jawnym stosie nie jest konieczne.

Do zapisania w `STATUS.md` — review to zatwierdziło.

---

## 14. Plan zmian w plikach

**Dodane:**

| Plik | Zawartość |
|---|---|
| `src/ik_equations/symbolic/SymbolicTransform.cpp` | `multiplyTransforms`, `isIdentityTransform`, `hasCanonicalHomogeneousLastRow` |
| `src/ik_equations/builders/ForwardKinematicsBuilder.cpp` | pętla akumulująca |
| `tests/test_symbolic_transform.cpp` | 8 testów z §15.1 |
| `tests/test_forward_kinematics_builder.cpp` | 12 testów z §15.2–15.4 |

**Zmienione:**

| Plik | Zmiana |
|---|---|
| `src/ik_equations/symbolic/SymbolicTransform.hpp` | deklaracje trzech funkcji + komentarz kontraktowy |
| `src/ik_equations/builders/ForwardKinematicsBuilder.hpp` | konstruktor z fabryką, pole `factory_`, komentarz kontraktowy |
| `CMakeLists.txt` | dwie linie — `SymbolicTransform.cpp`, `ForwardKinematicsBuilder.cpp` |
| `tests/CMakeLists.txt` | dwie linie |
| `STATUS.md` | stan komponentu; notatki z §9.1 i §13 |

**Bez zmian — jawnie:** `ExpressionFactory`, `Expression`, `SymbolicMatrix`, `JointTransformBuilder`, `KinematicChainBuilder`, `UrdfModelLoader`, `IkEquationBuilder`.

---

## 15. Plan testów

Review §5 ma rację: skoro `multiplyTransforms` trafia do warstwy symbolicznej jako operacja wielokrotnego użytku, nie może być testowana wyłącznie pośrednio przez builder. **Dwa poziomy, dwa pliki.**

### 15.1 `tests/test_symbolic_transform.cpp` — 8 testów (algebra)

Lista z review, przyjęta bez zmian:

| Test | Co pinuje |
|---|---|
| `RecognizesIdentityTransform` | predykat na wszystkich 16 komórkach |
| `RejectsNonIdentityTransform` | każda pojedynczo zaburzona komórka daje `false` |
| `LeftIdentityReturnsRightStructurally` | `I · T = T` — §6.2 |
| `RightIdentityReturnsLeftStructurally` | `T · I = T` — **review §1**; 8/16 komórek bez fast pathu |
| `PreservesLeftRotationWhenRightRotationIsIdentity` | `R_b = I` — **review §2**; 6/9 komórek obrotu bez fast pathu |
| `PreservesLeftTranslationWhenRightTranslationIsZero` | `p_b = 0` — **review §3**; 2/3 komórki kolumny bez fast pathu |
| `MultipliesHomogeneousTransformsStructurally` | ogólny przypadek: oba operandy nietrywialne, `structurallyEqual` wobec ręcznie zbudowanego `R_a·R_b` i `p_a + R_a·p_b` |
| `PreservesCanonicalHomogeneousLastRow` | wynik ma `[0 0 0 1]` przez `isZero`/`isOne` |

Wszystkie syntetyczne, bez danych robota. Trzy środkowe są **regresyjne wobec v1 tego proposalu** — v1 by ich nie przeszła, liczby w §6.3 mówią dokładnie ile komórek by się rozjechało.

### 15.2 Kontrakty buildera

| Test | Co pinuje |
|---|---|
| `EmptyChainReturnsIdentity` | §4.2 — wszystkie 16 komórek, nie tylko wybrane |
| `BuildsSingleJointForwardKinematics` | jeden joint, sensowna transformacja, kanoniczny ostatni wiersz |
| `SingleJointResultMatchesJointTransformBuilder` | §6.8 — zweryfikowane, że przechodzi |
| `PreservesHomogeneousLastRow` | §5.2 na poziomie całego łańcucha — failuje natychmiast po podmianie na naiwne `multiply(4×4)` |

### 15.3 Kolejność — poprawiona po review §6

**v1 miała tu błąd matematyczny.** Proponowałem `PreservesJointOrder` na „dwóch jointach `fixed` o różnych translacjach". Review słusznie zauważa, że czyste translacje są przemienne: `Trans(p1) · Trans(p2) = Trans(p2) · Trans(p1) = Trans(p1 + p2)`.

Zweryfikowałem: dla dwóch czysto translacyjnych jointów `A·B` i `B·A` różnią się w **0 z 16 komórek** — strukturalnie identyczne. Test w kształcie z v1 przechodziłby dla obu kolejności, czyli nie sprawdzałby niczego.

| Test | Poprawiona konstrukcja |
|---|---|
| `PreservesJointOrder` | `T_1 = Rz(π/2)` (fixed, samo `rpy`), `T_2 = TransX(1)` (fixed, sama translacja). `T_1·T_2` daje przesunięcie wzdłuż **Y**, `T_2·T_1` wzdłuż **X**. Wszystko zwija się do stałych, więc porównanie liczbowe — z `ASSERT_TRUE(isConstant(...))` przed `constantValue(...)` |
| `DistinguishesNonCommutingJointOrder` | jointy aktywne: `T_1 = Rz(q1)`, `T_2 = Rx(q2)`; komórka wybrana tak, by kolejności różniły się **strukturalnie**, nie tylko numerycznie |

Test sprawdzający tylko „czy występują `q1` i `q2`" przechodzi dla dowolnej permutacji jointów, czyli nie sprawdza niczego istotnego.

### 15.4 Zawartość, jointy nieaktywne, rzeczywiste roboty

| Test | Co pinuje |
|---|---|
| `IncludesFixedJoints` | joint `fixed` między dwoma aktywnymi wnosi offset — sprawdzane po wartości w kolumnie translacji, nie po liczbie zmiennych |
| `UsesAllJointVariables` | każde `q1..qn` występuje w wyniku; **rekurencyjny** `containsSymbol` |
| `DoesNotIntroduceSymbolsForFixedOnlyChain` | łańcuch samych `fixed` → żadnej zmiennej w 16 komórkach |
| `BuildsIdentityForKr4BaseToBaseChain` *(nowy w v2)* | §6.7 — `base_link → base` w `kr4_r600.urdf` to realny, jednojointowy łańcuch o transformacie dokładnie jednostkowej; dowodzi, że przypadek z review §1 jest osiągalny przez pipeline, nie tylko syntetycznie |
| `BuildsKr4SymbolicForwardKinematics` | `base_link → tool0`: obecność `q1..q6`, kanoniczny ostatni wiersz |
| `BuildsKr640SymbolicForwardKinematics` | j.w. dla `kr640.urdf` |

**Razem: 8 + 12 = 20 nowych testów. Oczekiwany stan: 159/159.**

### 15.5 Ograniczenie, które trzeba zapisać wprost

Bez `ExpressionEvaluator` testy KR4/KR640 sprawdzają **strukturę, nie wartości**. Potwierdzą kształt, obecność zmiennych i kanoniczny ostatni wiersz — ale **nie** potwierdzą, że macierz FK daje poprawną pozycję narzędzia dla zadanego `q`.

To realna luka, nie formalność: testy strukturalne złapią pomyłkę kolejności i zniszczony ostatni wiersz, ale **nie złapią konsekwentnego błędu znaku** w konwencji RPY ani w złożeniu `p_a + R_a·p_b`. Do czasu ewaluatora poprawność FK opiera się na argumencie, nie na asercji, i tak należy to raportować.

### 15.6 Konwencje

Bez zmian: `TEST(SuiteName, ...)`, `KINEMAFORGE_URDF_DATA_DIR` dla ścieżek, `structurallyEqual` do porównań strukturalnych, `ASSERT_TRUE(isConstant(...))` **przed** `constantValue(...)` (inaczej `assert` w `constantValue` ubija proces zamiast zgłosić failure), `SCOPED_TRACE` w pętlach po komórkach, rekurencyjny `containsSymbol`.

---

## 16. Ryzyka i trade-offy

| Ryzyko | Ocena | Reakcja |
|---|---|---|
| Brak numerycznej weryfikacji FK | **wysokie** — największa luka etapu | świadomie zaakceptowane; `ExpressionEvaluator` bezpośrednio po tym komponencie (§15.5) |
| Duplikacja `isIdentityRotation` (wariant B) | niskie | jawny dług; ekstrakcja osobną zmianą |
| Sześć gałęzi w `multiplyTransforms` | niskie | każda ma dedykowany test w §15.1; algorytm jest płaski, bez zagnieżdżeń |
| Rozmiar drzewa dla dłuższych łańcuchów | średnie | 281–516 unikalnych węzłów dla 7 jointów; traversal wymaga memoizacji (§13) |
| Dwie instancje fabryki | niskie dziś, rośnie ze stanem fabryki | udokumentowane w `STATUS.md` (§9.1) |
| Szum numeryczny ze zwijania trygonometrii | znane, nie nowe | istniejący gap warstwy symbolicznej; ten komponent go nie pogarsza |

---

## 17. Stan decyzji

**Zatwierdzone w review v1, bez zmian:** wariant B (operacja w warstwie `SymbolicTransform`, `.hpp` + `.cpp`), zwijanie lewostronne, pusty łańcuch → identity, API z `ExpressionFactory` jako polem, brak `std::expected`, memoizacja przyszłego ewaluatora, `multiplyTransforms` zamiast pełnego `multiply(4×4)`.

**Zmienione w v2 na żądanie review:** komplet fast pathów (§6.2), kontrakt i preconditions `multiplyTransforms` (§7), `isIdentityTransform` na 16 komórkach (§7), osobny plik testów warstwy symbolicznej (§15.1), poprawiony `PreservesJointOrder` (§15.3).

**Rozstrzygnięte, niewymagające decyzji:** kolejność mnożenia (§4.1), struktura blokowa (§5), brak czwartego fast pathu dla `p_lhs == 0` (§6.4, zmierzone), granica wobec simplifiera (§11).

**Nowe, drobne, do akceptacji:** `BuildsIdentityForKr4BaseToBaseChain` (§15.4) — dodatkowy test wykorzystujący fakt z §6.7. Jeżeli review uzna go za zbędny wobec syntetycznego `RightIdentityReturnsLeftStructurally`, usunięcie kosztuje jedną pozycję i daje 158/158.

---

## 18. Rekomendacja końcowa

Zatwierdzić w kształcie: **akumulacja lewostronna od `Identity`, `multiplyTransforms` w warstwie symbolicznej z asertowanym preconditionem kanonicznej transformacji jednorodnej, komplet sześciu fast pathów dla elementów neutralnych, brak modelu błędów, brak jakiegokolwiek upraszczania.**

Komponent jest prosty — pętla i jedno mnożenie blokowe. Cała trudność leży w utrzymaniu inwariantów w warstwie, która nie zna `x · 0 → 0`, i review pokazało, że v1 rozwiązała ten problem tylko w jednej czwartej.

---

## 19. Co zmieniła rewizja v2

| # review | Zarzut | Werdykt | Gdzie |
|---|---|---|---|
| 1 | brak fast pathu `rhs == I` | **przyjęty — błąd v1** | §6.2, §6.3, §6.7 |
| 2 | brak fast pathu `R_b == I` | **przyjęty — błąd v1** | §6.2, §6.3 |
| 3 | brak fast pathu `p_b == 0` | **przyjęty — błąd v1** | §6.2, §6.3 |
| 3 | powtórzyć pomiar po zmianie | wykonane | §6.5, §6.6 |
| 4 | brak kontraktu `multiplyTransforms` | przyjęty | §7 (nowa sekcja) |
| 4 | `isIdentityTransform` na 16 komórkach | przyjęty | §7 |
| 5 | osobne testy warstwy symbolicznej | przyjęty | §15.1 (nowy plik) |
| 6 | `PreservesJointOrder` matematycznie błędny | **przyjęty — błąd v1** | §15.3 |

Cztery z sześciu zarzutów to realne błędy, nie preferencje. Dwa z nich — §1 i §6 — pokazują ten sam wzorzec myślowy po mojej stronie i wart jest nazwania, bo prawdopodobnie wróci przy kolejnych komponentach:

- **§1/§2/§3**: uzasadniałem regułę ogólną pomiarem na dwóch plikach URDF. `kr4_r600` daje z nowych fast pathów **dokładnie zero** zysku (§6.6) — gdybym trzymał się kryterium z v1, odrzuciłbym je po raz drugi, mimo 54% zysku na `kr640` i strukturalnej poprawności w trzech testach syntetycznych. Właściwym uzasadnieniem jest tożsamość algebraiczna, nie licznik węzłów.
- **§6**: zaprojektowałem test pod nazwę („sprawdza kolejność"), nie pod matematykę. Dwie czyste translacje komutują — zweryfikowane, 0/16 komórek różnych. Test przechodziłby dla obu kolejności.

Zysk łączny po v2, licząc od punktu wyjścia: **466 848 → 21 882 węzłów dla `kr640`, 21,3×**, przy kanonicznym ostatnim wierszu i zachowanych tożsamościach elementu neutralnego.

Po zatwierdzeniu: `proposal-forward-kinematics-builder-implementation.md` z pełnym kodem produkcyjnym i testowym. Bez commita, bez kodu na dysku do tego czasu.
