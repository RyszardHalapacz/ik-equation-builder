# Proposal: numeryczna walidacja FK — architektura (v2)

## Prompt

> To jest teraz najważniejszy krok projektu. Nie fasada. Mamy `URDF → KinematicChain → Symbolic FK → ExpressionEvaluator → macierz double`, ale nie porównaliśmy tej macierzy z niezależnie policzonym FK. Nie wolno porównywać symbolic FK z drugim kodem opartym na `JointTransformBuilder` / `multiplyTransforms` / `ExpressionFactory` — to sprawdziłoby tylko samospójność. Rekomenduję testowy numeryczny FK oparty na kwaternionach. Tolerancja powinna wejść po pomiarze, nie „na oko".

Etap 2 z sześciu: poprawka `STATUS.md` (zrobiona) → **architektura** → review → proposal implementacyjny → wdrożenie → fasada.

## Status weryfikacji

**Ten dokument nie zawiera kodu produkcyjnego i niczego nie kompilowałem.**

Geometrię KR640 w §6.1 odczytałem z `data/urdf/kr640.urdf`. Jedna rzecz jest **niemożliwa do zmierzenia na tym etapie** i wymaga rozstrzygnięcia procesowego — §8.

---

## 1. Cel i czego to naprawdę dowodzi

Cel: porównać `T_base_tool(q)` policzone symbolicznie i wyewaluowane numerycznie z macierzą policzoną **inną drogą**.

Ale zanim ustalimy jak, trzeba precyzyjnie powiedzieć **co ten test obejmuje, a czego nie** — bo łatwo tu przecenić wynik.

### 1.1 Co zostaje zwalidowane

`JointTransformBuilder` (konwencja RPY, Rodrigues, fast pathy osi), `multiplyTransforms` (blokowe składanie, pięć fast pathów), `ForwardKinematicsBuilder` (kolejność zwijania), `ExpressionEvaluator` (ewaluacja drzewa).

### 1.2 Czego **nie** zostaje zwalidowane

Referencja czyta **ten sam `KinematicChain`**, pochodzący z tego samego loadera i tego samego chain buildera. Jeżeli loader źle sparsuje oś albo chain builder pomyli kolejność jointów, **obie strony zobaczą ten sam błąd i zgodnie go potwierdzą.**

To nie jest wada projektu testu — to jego granica, i musi być zapisana, żeby nikt nie czytał zielonego wyniku jako „URDF jest poprawnie interpretowany". Test waliduje **potok symboliczny od `KinematicChain` w górę**, nie parsowanie.

### 1.3 Granica poważniejsza: kwaterniony nie chronią przed wspólnym nieporozumieniem

To jest najważniejsze zastrzeżenie w tym dokumencie i chcę je postawić na początku, a nie schować w ryzykach.

Referencja kwaternionowa chroni przed **błędem implementacyjnym** — literówką w komórce macierzy, pomylonym znakiem sinusa, złą składową osi. Te same wzory zapisane w dwóch różnych reprezentacjach nie popsują się identycznie.

Nie chroni natomiast przed **wspólnym nieporozumieniem co do specyfikacji**. Jeżeli oboje wierzymy, że URDF znaczy `Rz·Ry·Rx`, a naprawdę znaczy coś innego, to napiszę referencję z tego samego przekonania i obie strony zgodnie potwierdzą błąd. Trzy testy referencji z promptu (`NumericReferenceUsesCorrectRpyOrder` i pokrewne) **tego nie łapią** — sprawdzają referencję wobec tego samego przekonania.

Wniosek: potrzebny jest przynajmniej jeden **oracle prawdziwie zewnętrzny** — wartość znana niezależnie od obu implementacji. §6 pokazuje, że dla KR640 taki oracle istnieje i jest liczony ręcznie.

---

## 2. Zakres i non-goals

**W zakresie:** niezależna referencja kwaternionowa jako kod testowy; porównanie 16 komórek dla zestawu konfiguracji na obu robotach; dwa testy syntetyczne dla gałęzi nieosiągalnych z danych; testy samej referencji; ustalenie i **uzasadnienie** tolerancji.

**Poza zakresem:** jakakolwiek zmiana w kodzie produkcyjnym; walidacja loadera i chain buildera (§1.2); fasada; wydajność; kinematyka odwrotna.

---

## 3. Referencja — kształt i umiejscowienie

```
tests/support/NumericForwardKinematics.hpp
tests/support/NumericForwardKinematics.cpp
```

Reprezentacja: kwaternion + wektor, konwersja do macierzy 3×3 **dopiero na końcu**.

```
q_origin  = qz(yaw) · qy(pitch) · qx(roll)
q_motion  = [ cos(θ/2), axis · sin(θ/2) ]          // revolute / continuous
prismatic : rotation = identity, translation = axis · d

kompozycja:  q = q_a · q_b ,  p = p_a + rotate(q_a, p_b)
```

### 3.1 Decyzja: referencja **nie** trafia do `kinemaforge_ik`

Kusi, żeby „przy okazji" dać projektowi numeryczne FK jako funkcję biblioteczną. **Nie.** Gdyby referencja znalazła się w bibliotece produkcyjnej, ktoś prędzej czy później użyłby jej w potoku — i niezależność, która jest jedyną wartością tego kodu, wyparowałaby w momencie, w którym przestałaby być druga.

Referencja jest rusztowaniem testowym. Dodajemy ją do `kinemaforge_tests`, nie do `kinemaforge_ik`, i to jest decyzja architektoniczna, nie szczegół CMake.

### 3.2 Dlaczego akurat kwaterniony

Bo różnią się od produkcji **na poziomie algebry**, nie zapisu. Rodrigues buduje macierz z `(1−cos)` i iloczynów składowych osi; kwaternion buduje ją z `cos(θ/2)`, `sin(θ/2)` i iloczynów par składowych. Złożenie obrotów to mnożenie kwaternionów zamiast mnożenia macierzy 3×3. Nie ma tu wspólnego podwyrażenia, które dałoby się pomylić identycznie po obu stronach.

**Poprawka po review.** Napisałem, że kwaternion jednostkowy „nie może" dać macierzy nieortogonalnej. To prawda matematycznie, ale nieprawda implementacyjnie: w `double` kwaternion osi-kąt nie wychodzi idealnie jednostkowy, kolejne mnożenia wprowadzają dryf normy, a błędna konwersja do macierzy i tak da wynik nieortogonalny. Poprawne sformułowanie:

> Reprezentacja kwaternionowa zachowuje strukturę obrotu **matematycznie** i redukuje ryzyko błędu wspólnego z implementacją macierzową. Zachowanie numeryczne oraz poprawność konwersji muszą być przypięte testami inwariantów.

Stąd dwa wymagania:

- **obrót wektora bez konwersji do macierzy** — `rotate(q, v) = v + 2w(u×v) + 2u×(u×v)` dla `q = (w, u)`; macierz 3×3 powstaje wyłącznie przy budowaniu końcowego wyniku
- **test inwariantów właściwego obrotu**, nie tylko ostatniego wiersza: `RᵀR ≈ I`, `det(R) ≈ +1`, `‖q‖ ≈ 1`, ostatni wiersz `[0,0,0,1]`

**Nie normalizujemy kwaternionu bezwarunkowo po każdym mnożeniu.** Taka normalizacja maskowałaby poważniejszy błąd implementacji — sprawdzamy normę i dopuszczamy wyłącznie szum `double`.

Test `NumericReferenceProducesHomogeneousTransform` zostaje przemianowany na **`NumericReferenceProducesProperRigidTransform`**, bo obiecuje teraz więcej niż kanoniczny ostatni wiersz.

---

## 4. Pipeline testu

```
1. loader          → RobotDescription
2. chain builder   → KinematicChain
3. FK builder      → SymbolicTransform
4. SymbolValues z chain.joints[i].variable->name       (nie z zaszytych "q1".."q6")
5. jeden ExpressionEvaluator → 16 wartości double
6. referencja kwaternionowa na tym samym KinematicChain → RigidTransform
7. porównanie 16 komórek
```

Trzy rzeczy, które muszą być zapisane jako wymagania, bo ich naruszenie daje test wyglądający na poprawny:

**Jeden evaluator na całą macierz.** Nie po jednym na komórkę — inaczej tracimy cache między korzeniami, co jest dokładnie tym, dla czego evaluator jest sesją. Przy okazji ten test jest pierwszym realnym użyciem tej właściwości poza mikrotestami.

**Referencja NIE dostaje `SymbolValues`** — poprawka po review, i to jest zmiana projektowa, nie kosmetyczna. Gdyby referencja adresowała jointy po tych samych nazwach co strona symboliczna, to przy pomyłce chain buildera (ta sama nazwa przypisana dwóm jointom) **obie strony wzięłyby tę samą wartość i zgodnie potwierdziły błąd**.

Wspólna jest **konfiguracja liczbowa**, nie mechanizm adresowania:

```cpp
using JointConfiguration = std::vector<double>;   // jedna wartość na joint aktywny,
                                                  // w kolejności łańcucha

RigidTransform numericForwardKinematics(const KinematicChain&, const JointConfiguration&);
SymbolValues   makeSymbolValues(const KinematicChain&, const JointConfiguration&);
```

- **referencja** zużywa wartości **sekwencyjnie** po pozycji jointu aktywnego
- **strona symboliczna** adresuje je **po nazwie** z `joint.variable->name`

Dwie różne drogi od tej samej liczby do tego samego jointu. Oba helpery asertują: liczba wartości równa liczbie jointów aktywnych, każdy joint aktywny ma `variable`, joint `fixed` nie zużywa wartości.

**Błąd ewaluacji to porażka testu, nie wartość do porównania.** `evaluate` zwraca `std::expected`; każda z 16 komórek musi mieć wartość. `ASSERT` na `has_value()` przed jakimkolwiek porównaniem.

---

## 5. Konfiguracje testowe

Zgodnie z propozycją z promptu, cztery grupy na robota:

| Grupa | Zawartość | Co izoluje |
|---|---|---|
| zerowa | `q = 0` | konwencję RPY i jointy `fixed`; dla KR4 nietrywialne, bo ma niezerowe `rpy` |
| pojedyncze jointy | 6 konfiguracji, jeden joint ≠ 0 | błąd konkretnego jointu — bez tego błąd w `joint_4` może się zamaskować |
| mieszana | `[0.35, −0.45, 0.55, −0.65, 0.40, −0.30]` | interakcje między jointami |
| blisko limitów | naprzemiennie `lower + 5%`, `upper − 5%` | duże kąty, gdzie błędy znaku stają się widoczne |

9 konfiguracji × 2 roboty = **18 porównań macierzy, 288 komórek**.

### 5.1 Limity brane z modelu, nie zaszyte

`KinematicJoint` niesie `limits`, więc `lower + 5%` liczymy w teście z załadowanego modelu. Zaszycie liczb oznaczałoby cichą nieaktualność po zmianie URDF-a. To bezpieczne: limity nie wpływają na poprawność FK, więc test nie waliduje się własnymi danymi wejściowymi.

Zgadzam się z promptem, żeby **nie** używać dokładnych limitów — walidujemy FK, nie zachowanie brzegowe parsera. Margines 5% jest arbitralny, ale jego jedyną rolą jest odsunięcie się od wartości granicznej.

### 5.2 Uwaga do konfiguracji mieszanej

Wartości z promptu trzeba sprawdzić wobec limitów **obu** robotów. KR640 ma `joint_a2` w zakresie `[−2.0944, 0.6109]` — wartość `−0.45` mieści się, ale to nie jest przypadek, który wolno założyć bez sprawdzenia dla każdego jointu. Proposal implementacyjny musi to zweryfikować i, jeśli trzeba, dobrać osobne wektory na robota.

---

## 6. Oracle zewnętrzny — najmocniejszy test w tym zestawie

§1.3 mówi, że sama referencja kwaternionowa nie chroni przed wspólnym nieporozumieniem. Dla KR640 istnieje wyjście.

### 6.1 KR640 przy `q = 0` da się policzyć ręcznie

Odczytane z `data/urdf/kr640.urdf` — **wszystkie siedem jointów ma `rpy="0 0 0"`**:

| Joint | `xyz` |
|---|---|
| `joint_a1` | `0, 0, 0.750` |
| `joint_a2` | `0.350, 0, 0` |
| `joint_a3` | `0, 0, 1.150` |
| `joint_a4` | `1.250, 0, 0.145` |
| `joint_a5` | `0, 0, 0` |
| `joint_a6` | `0, 0, 0` |
| `joint_a6_to_tool0` (fixed) | `0, 0, 0.290` |

Przy `q = 0` każdy obrót ruchu jest jednostkowy, a każde `R_origin` też — więc **każda transformacja jest czystą translacją**, a translacje się dodają, niezależnie od kolejności:

```
x = 0.350 + 1.250                     = 1.600
y = 0
z = 0.750 + 1.150 + 0.145 + 0.290     = 2.335

T_base_tool(0) = [ I | (1.600, 0, 2.335) ]
```

To jest **prawdziwy oracle**: wartość wyprowadzona z pliku URDF przez dodawanie, nie przez żadną z dwóch implementacji. Jeżeli obie strony zgodnie dadzą co innego, wspólne nieporozumienie zostanie wykryte.

Nazwa testu: `Kr640ZeroConfigurationMatchesHandComputedPose`.

**Poprawka po review — czego ten oracle NIE testuje.** Twierdziłem, że sprawdza kolejność składania. To nieprawda: przy `q = 0` wszystkie transformacje są czystymi translacjami, a `Trans(p₁)·Trans(p₂) = Trans(p₂)·Trans(p₁)`. Można odwrócić kolejność wszystkich siedmiu jointów i wynik nadal będzie `(1.600, 0, 2.335)`.

Sprawdza faktycznie: uwzględnienie **wszystkich** translacji (w tym jointu `fixed` do `tool0`), jednostki, orientację jednostkową i samą pozycję zerową. Nie sprawdza ani kolejności, ani konwencji RPY (wszystkie `rpy` zerowe).

### 6.1a Drugi oracle ręczny — ten testuje kolejność

`q1 = π/2`, reszta zero. Joint `a1` obraca wokół `+Z` wszystko, co za nim.

Translacje za `a1`: `x = 0.350 + 1.250 = 1.600`, `z = 1.150 + 0.145 + 0.290 = 1.585`.

```
p = (0, 0, 0.750) + Rz(π/2) · (1.600, 0, 1.585)
  = (0, 0, 0.750) + (0, 1.600, 1.585)
  = (0, 1.600, 2.335)

        [ 0  -1   0   0     ]
   T =  [ 1   0   0   1.600 ]
        [ 0   0   1   2.335 ]
        [ 0   0   0   1     ]
```

Nazwa: `Kr640Joint1QuarterTurnMatchesHandComputedPose`.

Ten wykrywa: złą kolejność transformacji, zły znak osi A1, błędną propagację translacji przez wcześniejszy obrót, zamianę `T_origin · T_motion` na odwrotną, oraz błędną orientację końcową. Oba oracle zostają — sprawdzają rozłączne rzeczy.

### 6.2 Prismatic z obróconym origin też jest liczalny ręcznie

`R_origin = Rz(π/2)`, oś `X`, `q = 0.3`. Przesunięcie `axis · q = (0.3, 0, 0)` obrócone przez `Rz(π/2)` daje `(0, 0.3, 0)`.

Wartość oczekiwaną wpisujemy **wprost**, nie z referencji kwaternionowej. To znów oracle zewnętrzny, i akurat pokrywa gałąź, której żaden prawdziwy robot w repo nie ma.

### 6.3 Czego nadal brakuje — do zapisania w `STATUS.md`

Konwencja RPY dla nietrywialnych kątów pozostaje potwierdzona wyłącznie przez **zgodność dwóch implementacji napisanych przez tego samego autora z tego samego przekonania**. Prawdziwe domknięcie wymagałoby porównania z zewnętrznym narzędziem (KDL, `tf2`, Pinocchio) albo z opublikowanymi pozami KR4. Nie proponuję tego teraz — dodałoby zależność do projektu testowego — ale **nie wolno raportować tego etapu jako pełnego dowodu poprawności interpretacji URDF**.

---

## 7. Testy — 14 pozycji

### 7.1 Roboty (8)

`EvaluatesKr4ZeroConfigurationAgainstNumericReference`
`EvaluatesKr4SingleJointConfigurationsAgainstNumericReference`
`EvaluatesKr4MixedConfigurationAgainstNumericReference`
`EvaluatesKr4NearLimitsAgainstNumericReference`
`EvaluatesKr640ZeroConfigurationAgainstNumericReference`
`EvaluatesKr640SingleJointConfigurationsAgainstNumericReference`
`EvaluatesKr640MixedConfigurationAgainstNumericReference`
`EvaluatesKr640NearLimitsAgainstNumericReference`

### 7.2 Oracle zewnętrzny (2)

`Kr640ZeroConfigurationMatchesHandComputedPose` — §6.1, translacje i offset jointu `fixed`.
`Kr640Joint1QuarterTurnMatchesHandComputedPose` — §6.1a, **kolejność** i propagacja translacji przez wcześniejszy obrót.

Jedyne testy w zestawie, które nie porównują dwóch implementacji ze sobą.

### 7.3 Syntetyczne (3)

`EvaluatesNegativePrincipalAxisAgainstHandComputedPose` — **dodany po review.** KR4 i KR640 mają wyłącznie **dodatnie** osie główne, więc gałąź `negated == true` w fast pathach `JointTransformBuilder` nie zostaje uruchomiona przez żadne realne dane. Oś `(0, 0, −1)`, `q = π/2`, origin jednostkowy; oczekiwana `Rz(−π/2)` wpisana wprost:

```
[  0   1   0 ]
[ -1   0   0 ]
[  0   0   1 ]
```

Bez tego testu deklarację trzeba by zawęzić do „walidujemy dodatnie fast pathy osi występujące w KR4/KR640 oraz ogólny Rodrigues".

`EvaluatesArbitraryAxisRevoluteAgainstQuaternionReference` — oś `normalize([1,2,3])`, kąt `0.7`; pokrywa ogólną gałąź Rodriguesa, której osie z KR4/KR640 nigdy nie uruchamiają (wszystkie są osiowe, więc idą fast pathem).

`EvaluatesRotatedPrismaticAgainstQuaternionReference` — §6.2; wartość oczekiwana **hardkodowana z ręcznego wyliczenia**, nie z referencji.

### 7.4 Testy samej referencji (3)

`NumericReferenceUsesCorrectRpyOrder`
`NumericReferenceUsesCorrectCompositionOrder`
`NumericReferenceProducesProperRigidTransform` — `RᵀR ≈ I`, `det(R) ≈ +1`, `‖q‖ ≈ 1`, ostatni wiersz `[0,0,0,1]`

Zgadzam się z ich sensem — „niezależny oracle" nie może być niezweryfikowanym drugim źródłem błędów. Ale zapisuję wprost, **co one łapią**: literówki i pomyłki w samej referencji. **Nie łapią** wspólnego nieporozumienia co do specyfikacji URDF (§1.3), bo sprawdzają referencję wobec tego samego przekonania, z którego powstała produkcja.

**Razem: 16 nowych. Oczekiwany stan: 187 + 16 = 203.**

*(8 robotów + 2 oracle ręczne + 3 syntetyczne + 3 testy referencji)*

---

## 8. Tolerancja — i problem procesowy, który trzeba rozstrzygnąć

Zgadzam się w całości, że tolerancja nie może być wybrana „na oko" i że porównanie dokładne odpada (`sin(π) ≈ 1.22e-16`). Zgadzam się też z formą warunku:

```
|actual − expected| ≤ absoluteTolerance + relativeTolerance · |expected|
```

kandydat: `1e-12` / `1e-12`, oraz z tym, że w dokumencie powinien znaleźć się **zmierzony najgorszy błąd**, osobno dla bloku obrotu i dla translacji.

### 8.1 Dlaczego nie mogę tego zmierzyć teraz

Żeby zmierzyć najgorszy błąd, trzeba mieć **referencję kwaternionową** i **kod porównujący** — czyli dokładnie to, co ma zawierać proposal implementacyjny. Pomiar teraz oznacza napisanie implementacji przed dokumentem implementacyjnym, czyli odwrócenie procesu, który w tym projekcie działa i który już dwa razy wychwycił błędy przed wejściem kodu na dysk.

### 8.2 Propozycja rozstrzygnięcia — **zatwierdzona w review**

Tolerancja `1e-12 / 1e-12` wchodzi do proposalu implementacyjnego jako **kandydat**, z następującą regułą raportowania:

> Raport z wdrożenia **musi** podać zmierzony najgorszy błąd bezwzględny, osobno dla bloku obrotu i dla kolumny translacji, dla wszystkich 18 konfiguracji. Jeżeli którakolwiek wartość przekroczy kandydata, **nie jest to powód do podniesienia tolerancji** — jest to ustalenie wymagające osobnego review, bo oznacza albo realny błąd, albo że nie rozumiemy źródła szumu.

Alternatywa — napisanie referencji teraz, zmierzenie i wpisanie liczby do architektury — jest do przyjęcia, jeśli tak zdecydujesz, ale wtedy powiedzmy wprost, że ten etap łączy architekturę z implementacją. **Do rozstrzygnięcia w review.**

### 8.3 Czego się spodziewam

Szum wchodzi trzema drogami: zwijanie trygonometrii w fabryce (`1e-16`), akumulacja przy siedmiu złożeniach, i różnica reprezentacji (kwaternion → macierz robi inne zaokrąglenia niż Rodrigues). Rzędy wielkości sugerują błąd `1e-15`–`1e-14`, czyli zapas dwóch–trzech rzędów do `1e-12`. Ale **to jest oczekiwanie, nie pomiar**, i tak je oznaczam.

Osobne raportowanie obrotu i translacji ma konkretny powód: komórki obrotu są `≤ 1`, a translacje KR640 sięgają `2.335`, więc jedna wspólna liczba maskowałaby, po której stronie faktycznie jest problem.

### 8.4 Geometryczny błąd orientacji — diagnostyka dodana po review

Obok błędu per-komórka raport podaje kąt między orientacjami:

```
R_error     = R_referenceᵀ · R_actual
angle_error = acos( clamp( (trace(R_error) − 1) / 2, −1, +1 ) )
```

**Nie jest to warunek zaliczenia testu** — warunkiem pozostaje porównanie per-komórka z §8. Jest to diagnostyka, która odpowiada na pytanie, którego różnica pojedynczej komórki nie rozstrzyga: czy rozjazd oznacza **realny błąd orientacji**, czy szum w jednej komórce. `clamp` jest konieczny, bo przy błędzie rzędu `1e-16` argument `acos` potrafi wyjść minimalnie poza `[−1, 1]`.

Raport z wdrożenia podaje, dla najgorszego przypadku: robota, konfigurację, komórkę, `actual`, `expected`, oraz ten kąt.

---

## 9. Plan zmian w plikach

**Dodane:** `tests/support/NumericForwardKinematics.hpp`, `.cpp`, `tests/test_numeric_fk_validation.cpp`.

**Zmienione:** `tests/CMakeLists.txt` (pliki referencji + test, oraz `target_include_directories` na `tests/`, żeby `support/...` się rozwiązywało), `STATUS.md`.

**Bez zmian — jawnie:** cały `src/`. Ten etap nie dotyka kodu produkcyjnego. Jeżeli walidacja wykryje błąd, poprawka pójdzie **osobnym** proposalem — mieszanie „dodajemy test" z „naprawiamy to, co test znalazł" w jednej zmianie zaciera, co właściwie było zepsute.

---

## 10. Ryzyka

| Ryzyko | Ocena | Reakcja |
|---|---|---|
| test wykryje realny błąd w FK | **średnie i pożądane** — po to powstaje | poprawka osobnym proposalem (§9); raport podaje, która konfiguracja i która komórka |
| wspólne nieporozumienie co do URDF przechodzi niezauważone | **średnie**, nierozwiązane | oracle ręczny dla KR640 (§6.1) łapie część; reszta zapisana jako otwarta luka (§6.3) |
| tolerancja okaże się za ciasna | niskie | reguła raportowania z §8.2 — podniesienie wymaga review, nie decyzji implementatora |
| konfiguracja mieszana poza limitami któregoś robota | niskie | sprawdzenie w proposalu implementacyjnym (§5.2) |
| referencja sama ma błąd | niskie | trzy testy z §7.4 plus oracle ręczny; kwaternion jednostkowy nie może dać macierzy nieortogonalnej |

---

## 11. Stan decyzji

**Zatwierdzone w review:** referencja kwaternionowa wyłącznie w testach, brak zmian w `src/`, jeden evaluator na całą macierz, cztery grupy konfiguracji, tolerancja `1e-12/1e-12` jako kandydat z zasadą „przekroczenie ≠ podniesienie tolerancji, tylko raport i osobne review", oba oracle ręczne, granica dowodu opisana w §1.2 i §1.3.

**Poprawione po review (v2):**

| # | Korekta | Gdzie |
|---|---|---|
| 1 | oracle `q = 0` **nie** testuje kolejności — twierdzenie usunięte | §6.1 |
| 2 | dodany drugi oracle: `q1 = π/2` — ten testuje kolejność | §6.1a |
| 3 | `JointConfiguration` po pozycji zamiast wspólnych `SymbolValues` | §4 |
| 4 | kwaternion nie gwarantuje ortogonalności w `double`; obrót wektora bez macierzy; testy inwariantów | §3.2 |
| 5 | dodany `EvaluatesNegativePrincipalAxisAgainstHandComputedPose` | §7.3 |
| 6 | geometryczny błąd orientacji jako diagnostyka | §8.4 |
| 7 | liczba testów **16 nowych, 203 łącznie** | §7 |

**Otwarte, świadomie:** §6.3 — zewnętrzne narzędzie referencyjne (KDL / `tf2` / Pinocchio) nie wchodzi na tym etapie; luka zostaje zapisana w `STATUS.md`.

---

## 12. Rekomendacja końcowa

Zatwierdzić w kształcie: **referencja kwaternionowa jako kod testowy poza biblioteką, 18 porównań macierzy na dwóch robotach, dwa testy syntetyczne dla gałęzi nieosiągalnych z danych, trzy testy samej referencji, jeden oracle liczony ręcznie, tolerancja `1e-12/1e-12` jako kandydat z obowiązkiem raportowania zmierzonego najgorszego błędu.**

Po tym etapie będzie można powiedzieć: *„KinemaForge generuje z URDF symboliczną macierz FK, a jej poprawność została potwierdzona niezależnym obliczeniem numerycznym"* — z jednym przypisem, którego nie należy pomijać: niezależność dotyczy **implementacji**, nie **interpretacji specyfikacji URDF**, i potok od pliku do `KinematicChain` pozostaje poza zakresem tego dowodu.

Po zatwierdzeniu: `proposal-numeric-fk-validation-implementation.md` z pełnym kodem referencji i testów.
