# Proposal: topologia loadera — architektura (R2)

## Prompt

> Review kodu, uwagi 1, 2 i 3:
> 1. `trace_chain()` jest poprawne wyłącznie dla liniowego manipulatora; KR4 już
>    takim nie jest, więc semantyka loadera zależy od kolejności elementów XML.
> 2. `trace_chain()` może wejść w nieskończoną pętlę — brak `visited`.
> 3. Loader akceptuje odwołania do nieistniejących linków; `KinematicChainBuilder`
>    sam „dopowiada" linki z jointów.
>
> Rekomendacja review: nie naprawiać `trace_chain()` półśrodkami — loader ma
> walidować drzewo, a nie wybierać arbitralny linear chain.

Etap `F2.3.R`, dokument **R2**. Najwyższa waga w całym etapie.

## Status weryfikacji

Wszystkie objawy odtworzone na `main` (`9d2ceb2`) uruchomieniem, nie z lektury.

```text
[1] ten sam KR4, dwa uporządkowania XML
    kr4_r600.urdf                     → accepted, chain = 6 actuated joints
    kr4_r600.urdf (base_link-base
                   i joint_1 zamienione) → REJECTED, incomplete_kinematic_chain

[2] URDF z cyklem root→A, A→B, B→A
    load_urdf(...)                    → nie kończy się w 8 s (timeout, exit 124)

[3] links = {"base"}, joint base→ghost
    KinematicChainBuilder::build(robot, "base", "ghost") → ACCEPTED

[4] znalezisko dodatkowe: URDF rozspójniony (dwa korzenie)
    2 jointy w pliku                  → ACCEPTED, chain = 1 joint
                                        drugie poddrzewo znika bez śladu
```

## 1. Cel i zakres

**W zakresie:** ustalenie, co loader ma gwarantować o topologii, i usunięcie
`trace_chain()` jako mechanizmu wyboru ścieżki.

**Poza zakresem:** `<limit>` (R3), granica `RobotModel`/`within_limits` (R4),
`continuous` jako funkcjonalność, jakakolwiek zmiana w `KinematicChainBuilder`
poza usunięciem unii `knownLinks`.

## 2. Stan obecny — zweryfikowany w plikach

### 2.1 Trzy objawy, jedna przyczyna

`src/kinematics/robot_model_loader.cpp:112–153`. Cała funkcja opiera się na
założeniu, że **jeden parent ma dokładnie jedno dziecko**:

```cpp
std::unordered_map<std::string, std::size_t> parent_to_idx;
for (std::size_t i = 0; i < joints.size(); ++i) {
    parent_to_idx[joints[i].parent_link] = i;   // nadpisuje
    is_child[joints[i].child_link] = true;
}
```

URDF opisuje **drzewo**, nie łańcuch. Trzy niezależne skutki:

**(a) Nadpisywanie → zależność od kolejności XML.** `kr4_r600.urdf` ma
branching w dwóch miejscach:

```text
base_link ├─ base_link-base (fixed, idx 0) → base
          └─ joint_1        (revolute, idx 1) → link_1

link_6    ├─ link6-flange   (fixed, idx 7) → flange
          └─ link6-tool0    (fixed, idx 8) → tool0
```

`parent_to_idx["base_link"]` jest zapisywane dwa razy; wygrywa późniejszy joint.
Dziś to szczęśliwie `joint_1`, więc trace daje 6 jointów. Po zamianie tych dwóch
bloków `<joint>` w pliku — bez zmiany geometrii — wygrywa `base_link-base`, trace
kończy się na `base`, `chain` jest pusta i loader zwraca
`incomplete_kinematic_chain`.

**(b) Brak `visited` → pętla nieskończona.** Pętla `while (parent_to_idx.contains(current))`
nie ma ani zbioru odwiedzonych, ani limitu kroków. Dla cyklu `root→A, A→B, B→A`
korzeń zostaje znaleziony poprawnie (`root` nie jest niczyim dzieckiem), a potem
pętla krąży `A→B→A→B…` w nieskończoność, dopisując indeksy do `chain` — czyli
jest to zawieszenie **z nieograniczonym wzrostem pamięci**.

Sytuacja jest szczególnie niefortunna, bo `KinematicChainBuilder` **ma** wykrywanie
cyklu (`PathSearchResult::CycleDetected`, `KinematicChainBuilder.cpp:79–80`) —
tylko nigdy się do niego nie dochodzi, bo loader wiesza się wcześniej.

**(c) Wybór korzenia zależy od kolejności iteracji hash mapy** — to znalezisko
dodatkowe, nie z review, i objawowo najgroźniejsze:

```cpp
for (auto const& [parent, _] : parent_to_idx) {
    if (!is_child.contains(parent)) { current = parent; break; }
}
```

Przy dwóch rozłącznych poddrzewach oba korzenie spełniają warunek, a `break`
bierze pierwszy **w kolejności iteracji `unordered_map`** — czyli nie w
kolejności XML, tylko w kolejności zależnej od implementacji biblioteki
standardowej. Test: plik z dwoma jointami w dwóch poddrzewach przechodzi jako
poprawny i zwraca `chain` o długości 1.

Różnica wagi wobec (a): (a) daje **fałszywe odrzucenie** poprawnego robota — głośne
i łatwe do zauważenia. (c) daje **cichą złą odpowiedź** — model przechodzi
walidację, a połowa robota po prostu nie istnieje.

### 2.2 Wynik `trace_chain()` nie jest nigdzie używany

Sprawdzone: `RobotModel::chain()` nie ma w `src/` żadnego konsumenta poza
`within_limits()` w tym samym pliku. `UrdfModelLoader::load` przepisuje
`links()` i `joints()` do `RobotDescription` i **`chain()` ignoruje** —
`UrdfModelLoader.cpp:93–97`.

Właściwy wybór ścieżki robi `KinematicChainBuilder`, który rozumie branching:

```cpp
std::unordered_map<std::string_view, std::vector<std::size_t>> childrenOf;
```

Czyli mamy dwie warstwy wyboru ścieżki: jedną wadliwą i nieużywaną, drugą
poprawną i używaną. Wadliwa może odrzucić model, z którym poprawna poradziłaby
sobie bez problemu.

### 2.3 Brak walidacji istnienia linków

Loader czyta `<link>` do `links_` i osobno `<parent link=...>`/`<child link=...>`
do jointów, **nie sprawdzając, czy odwołanie do linku ma pokrycie w deklaracji**.
Wynikowy `RobotDescription` może więc zawierać joint wskazujący na link, którego
w URDF nie ma.

`KinematicChainBuilder` to obchodzi, robiąc unię (`KinematicChainBuilder.cpp:119–129`):

```cpp
// robot_model_loader.cpp doesn't cross-check <link> declarations against
// joint parent/child references, so RobotDescription.links alone isn't
// a reliable "does this link exist" source — union with joint references.
```

Motywacja jest zrozumiała, ale skutek — niebezpieczny. Konsekwencja, której
komentarz nie odnotowuje: **`BaseLinkNotFound` i `ToolLinkNotFound` nie mogą
zadziałać dla żadnej nazwy wspomnianej w jakimkolwiek joincie.** Literówka w
`<parent link="bas"/>` tworzy legalny frame `bas`, względem którego można
zbudować transformację. Kody błędów są słabsze, niż wyglądają.

## 3. Decyzja podstawowa: `trace_chain()` znika

Trzy warianty:

| Wariant | Ocena |
|---|---|
| **A.** Naprawić `trace_chain()`: `vector` dzieci, `visited`, wybór gałęzi | Odrzucony. Powstałby drugi, konkurencyjny mechanizm wyboru ścieżki obok `KinematicChainBuilder` — z pytaniem „którą gałąź wybrać", na które loader **nie ma prawa odpowiadać**, bo nie zna `baseLink`/`toolLink`. |
| **B.** Usunąć `trace_chain()`, w jego miejsce walidacja drzewa | **Wybrany.** Loader odpowiada za „czy ten plik opisuje poprawne drzewo", `KinematicChainBuilder` za „którą ścieżkę w tym drzewie chcę". |
| **C.** Usunąć `trace_chain()` bez zamiennika | Odrzucony. Cykl i rozspójnienie przeszłyby do `RobotDescription`; `KinematicChainBuilder` wykryje cykl na *swojej ścieżce*, ale nie odrzuci pliku z cyklem w gałęzi bocznej. |

Wariant B jest tym, co rekomenduje review, i zgadzam się bez zastrzeżeń.
Podział odpowiedzialności:

```text
UrdfModelLoader / load_urdf
    parsowanie + walidacja struktury drzewa
    → RobotDescription z gwarancją: to jest poprawne drzewo

KinematicChainBuilder
    wybór konkretnej ścieżki base → tool w tym drzewie
```

## 4. Co znaczy „poprawne drzewo" — pięć niezmienników

Loader ma gwarantować wszystkie pięć. Kolejność sprawdzania jest częścią
kontraktu (patrz 5.2).

| # | Niezmiennik | Dziś |
|---|---|---|
| T1 | każdy `joint.parent_link` istnieje w `<link>` | **nie sprawdzane** |
| T2 | każdy `joint.child_link` istnieje w `<link>` | **nie sprawdzane** |
| T3 | żaden link nie jest dzieckiem dwóch jointów | nie sprawdzane w loaderze; `KinematicChainBuilder` ma `InvalidRobotDescription`, ale tylko dla ścieżki, którą idzie |
| T4 | graf nie zawiera cyklu | **wiesza proces** |
| T5 | dokładnie jeden korzeń — graf jest spójny | **przechodzi i gubi poddrzewo** |

Uwagi do poszczególnych:

**T3 jest tym, co czyni graf drzewem.** W drzewie każdy węzeł ma co najwyżej
jednego rodzica. Bez T3 „drzewo" jest w istocie DAG-iem i pojęcie
`T_parent_child` traci jednoznaczność.

**T4 wynika częściowo z T3 + T5, ale nie w całości.** Przy T3 i jednym korzeniu
cykl musiałby być rozłączny z korzeniem — a to wyłapuje T5. Mimo to T4 zostaje
jako **jawne sprawdzenie**, nie jako wniosek: zależność jest subtelna, a koszt
jednego przejścia po grafie jest pomijalny. Wnioskowanie „to już wynika z
tamtego" jest dokładnie tym rodzajem rozumowania, po którym zostaje luka.

**T5 wymaga rozstrzygnięcia dla przypadku zerowego.** URDF z zerem jointów ma
zero korzeni. Czy to błąd?

> **Do rozstrzygnięcia w review.** Rekomendacja: **nie** jest błędem na poziomie
> loadera, jeśli plik ma co najmniej jeden `<link>`. Powód: `KinematicChain`
> dopuszcza `baseLink == toolLink` z pustym łańcuchem jako sukces, więc pojęcie
> „robot bez jointów" jest już w projekcie legalne. Plik z zerem linków **i**
> zerem jointów to natomiast `parse_failure` — nie ma czego opisywać.

### 4.1 Czego loader dalej **nie** gwarantuje

Świadomie:

- że istnieje ścieżka między dowolnymi dwoma linkami — to pytanie
  `KinematicChainBuilder`, zależne od argumentów;
- że robot ma ≥ 6 stopni swobody. Dzisiejsze ostrzeżenie „`chain has N actuated
  joints (expected >=6)`" (`robot_model_loader.cpp:146–151`) **znika razem z
  `trace_chain()`** i nie wraca. Powód: liczba stopni swobody na *ścieżce* jest
  własnością wybranej ścieżki, nie pliku. Robot 3-osiowy nie jest błędnym URDF-em,
  a ostrzeżenie liczone z arbitralnie wybranej gałęzi jest bezwartościowe.

> **Do rozstrzygnięcia w review:** czy ostrzeżenie o `< 6` DOF ma się odrodzić w
> `KinematicChainBuilder`, gdzie miałoby sens (tam ścieżka jest znana). Skłaniam
> się do **nie** — `KinematicChainBuilder` nie ma dziś `DiagnosticBag`, a
> dokładanie mu go dla jednego ostrzeżenia jest nieproporcjonalne. Do decyzji.

## 5. Model błędów

### 5.1 Nowe kody

`LoadErrorCode` dostaje cztery pozycje:

```text
undeclared_parent_link     T1
undeclared_child_link      T2
duplicate_child_link       T3
cyclic_topology            T4
disconnected_topology      T5  (obejmuje też „brak korzenia")
```

`incomplete_kinematic_chain` **zostaje usunięty** — był kodem wyjścia
`trace_chain()` i po jego usunięciu nie ma producenta. To zmiana obserwowalna:
istniejący test go sprawdza (do przejrzenia w proposalu implementacyjnym).

Każdy nowy kod potrzebuje wpisu w `describeCode` w `UrdfModelLoader.cpp:48`.
Po R1 brak wpisu wywali build (`-Wswitch`), co jest dokładnie tym, po co R1 jest
pierwsze.

### 5.2 Kolejność sprawdzania jest deterministyczna

Repo ma na to precedens: `IkEquationSystem::create` wymusza dziewięć
niezmienników w ustalonej kolejności, żeby każdy kod miał jednoznaczne
znaczenie, gdy naruszonych jest kilka. To samo tutaj:

```text
T1/T2 (istnienie linków)  →  T3 (duplikat dziecka)  →  T5 (korzeń)  →  T4 (cykl)
```

Dwie kolejności są **wymuszone, nie arbitralne**:

- **T1/T2 przed wszystkim.** Bez nich zbiór węzłów grafu nie jest ustalony, a
  „cykl" i „spójność" nie mają dobrze określonego znaczenia. Zgłoszenie
  literówki w nazwie linku jako `disconnected_topology` byłoby prawdą bezużyteczną.
- **T3 przed T5.** Link z dwoma rodzicami zaburza liczenie korzeni. Odwrotna
  kolejność raportowałaby jednoznaczny błąd struktury jako problem ze spójnością.

T5 przed T4 to wybór, nie konieczność: przy grafie rozspójnionym komunikat o
braku jednoznacznego korzenia jest bardziej użyteczny niż o cyklu w oderwanej
gałęzi. Do zaakceptowania lub odwrócenia w review.

### 5.3 Kontekst w błędzie

`LoadError` ma już `jointName`, `attribute`, `rawValue` (wszystkie ucinane do 256
znaków). Nowe błędy topologiczne wypełniają:

- T1/T2: `jointName` = joint z odwołaniem, `attribute` = `"parent/link"` lub
  `"child/link"`, `rawValue` = nieznaleziona nazwa;
- T3: `jointName` = **drugi** joint roszczący sobie to dziecko, `rawValue` = nazwa
  linku. Który joint jest „drugi", zależy od kolejności XML — i to jest w
  porządku, bo dotyczy tylko *komunikatu*, nie tego, czy plik przechodzi.
- T4: `rawValue` = nazwa linku, na którym zamknął się cykl;
- T5: `rawValue` = liczba znalezionych korzeni, lub nazwa drugiego korzenia.

> **Do rozstrzygnięcia w review:** czy dla T4 warto zwracać całą ścieżkę cyklu.
> Rekomendacja: nie — `rawValue` jest jednym stringiem uciętym do 256 znaków, a
> pełna ścieżka wymagałaby nowego pola w `LoadError`. Jedna nazwa wystarcza do
> zlokalizowania problemu w pliku.

## 6. Algorytm

Po `T1/T2` graf jest dobrze określony. Reszta to jedno przejście:

```text
1. zbuduj declaredLinks ze <link>                       O(L)
2. dla każdego jointu sprawdź parent i child            O(J)     → T1, T2
3. zbuduj childToParentJoint; kolizja → duplicate       O(J)     → T3
4. korzenie = linki nie występujące jako child          O(L)
   dokładnie jeden → ok, zero → cykl/brak, >1 → rozspójnienie   → T5
5. BFS/DFS z korzenia po childrenOf, licz odwiedzone
   odwiedzone != L → cykl albo sierota                  O(L+J)   → T4
```

Dwie decyzje implementacyjne, obie z precedensem w repo:

- **Iteracyjnie, nie rekurencyjnie.** `KinematicChainBuilder` używa jawnego
  stosu i `STATUS.md` odnotowuje „Recursion depth scaling unmeasured" jako lukę.
  Nie dokładamy nowej rekurencji po nieufnych danych z pliku.
- **Limit kroków jako pas bezpieczeństwa.** `visited` wystarcza do poprawności,
  ale pętla po danych z pliku dostaje dodatkowo twardy limit `L + J + 1` iteracji.
  Uwaga 2 istnieje właśnie dlatego, że ktoś założył, że pętla się skończy.

## 7. Konsekwencja: unia `knownLinks` w `KinematicChainBuilder` **znika**

Po T1/T2 `RobotDescription.links` jest wiarygodnym źródłem „czy ten link
istnieje", więc `KinematicChainBuilder.cpp:119–129` redukuje się do:

```cpp
std::unordered_set<std::string_view> knownLinks;
for (auto const& link : robot.links)
    knownLinks.insert(link.name);
```

To **nie jest kosmetyka** — to przywrócenie znaczenia `BaseLinkNotFound` i
`ToolLinkNotFound`, które dziś nie mogą zadziałać dla nazwy wspomnianej w
jakimkolwiek joincie.

Ważne zawężenie zakresu: gwarancja T1/T2 jest własnością **loadera**, nie typu.
`RobotDescription` to zwykły agregat i ręcznie zbudowany nie gwarantuje niczego —
tak samo jak dziś z geometrią (`RobotDescription.hpp:16–17`). Testy
`KinematicChainBuilder` budujące `RobotDescription` z ręki będą więc musiały
deklarować linki, których dziś nie deklarują. To przewidywalny koszt tej zmiany i
proposal implementacyjny musi go wycenić w liczbie zmienionych testów.

## 8. Plan testów

### 8.1 Nowe, topologia loadera (10)

```text
AcceptsBranchingTree                     KR4-podobny kształt, dwa rozgałęzienia
RejectsUndeclaredParentLink              T1
RejectsUndeclaredChildLink               T2
RejectsLinkClaimedByTwoJoints            T3
RejectsCyclicTopology                    T4  — musi ZAKOŃCZYĆ SIĘ
RejectsDisconnectedForest                T5
RejectsMultipleRoots                     T5, wariant jawny
AcceptsRobotWithLinksButNoJoints         przypadek zerowy z 4.
ReportsUndeclaredLinkWithJointContext    LoadError niesie jointName + rawValue
DiagnosticOrderIsDeterministic           plik łamiący T1 i T5 → zawsze T1
```

`RejectsCyclicTopology` ma jedną nietypową wymaganość: **test musi wykrywać
zawieszenie, nie tylko zły kod błędu.** Dziś ten test by nie „nie przeszedł" —
on by zawiesił cały `ctest`. Proposal implementacyjny musi rozstrzygnąć, czy
polegamy na timeoutcie CTest (`set_tests_properties(... TIMEOUT ...)`), czy na
limicie kroków z 6, który zamienia zawieszenie w normalny błąd. Rekomendacja:
**oba** — limit kroków jako poprawność, timeout CTest jako siatka
bezpieczeństwa, gdyby limit kiedyś przestał działać.

### 8.2 Niezależność od kolejności XML (2) — istota etapu

```text
LoadsIdenticallyRegardlessOfJointOrderInXml
BuildsKr4RegardlessOfJointOrder
```

Drugi jest testem regresji dokładnie na odtworzony przypadek: `kr4_r600.urdf` z
zamienionymi `base_link-base` i `joint_1` musi dać **identyczny**
`RobotDescription** co plik oryginalny (modulo kolejność jointów w wektorze).

> **Do rozstrzygnięcia w review:** jak porównywać „identyczny". Porównanie
> `RobotDescription` pole po polu wymaga stabilnej kolejności jointów, której po
> zamianie w XML nie ma. Rekomendacja: porównywać **zbiór** jointów po nazwie, a
> nie wektor po indeksie — i osobno sprawdzić, że oba pliki dają ten sam
> `KinematicChain` dla `base_link → tool0`. Drugi warunek jest mocniejszy i
> bliższy temu, na czym nam zależy.

### 8.3 `KinematicChainBuilder` po usunięciu unii (2)

```text
RejectsBaseLinkNotDeclaredAsLink
RejectsToolLinkNotDeclaredAsLink
```

Oba dziś **nie przechodzą** (frame jest akceptowany) — to test na uwagę 3.

### 8.4 Do przejrzenia, nie do dopisania

`incomplete_kinematic_chain` znika, więc każdy istniejący test oczekujący tego
kodu wymaga decyzji: czy dany plik ma teraz przechodzić, czy padać na innym
kodzie. Proposal implementacyjny musi wymienić te testy z nazwy — jest to jedyne
miejsce, gdzie ten etap może cicho zmienić obserwowalne zachowanie.

## 9. Plan zmian w plikach

```text
src/kinematics/robot_model_loader.hpp    LoadErrorCode: +4, -1; kontrakt w komentarzu
src/kinematics/robot_model_loader.cpp    -trace_chain (42 linie), +validate_topology
src/ik_equations/UrdfModelLoader.cpp     describeCode: +4 wpisy, -1
src/ik_equations/builders/KinematicChainBuilder.cpp   -unia knownLinks (8 linii)
tests/test_urdf_model_loader.cpp         +12
tests/test_kinematic_chain_builder.cpp   +2, przegląd RobotDescription w istniejących
data/urdf/                               ewentualne nowe fixture'y
```

Styk z R4: usunięcie `trace_chain()` czyni `RobotModel::set_chain()` funkcją bez
producenta, a `chain()` i `within_limits()` — bez sensownej treści. **Ten
proposal tego nie rozstrzyga**; decyzja należy do R4, żeby nie zapadła dwa razy
w sprzeczny sposób. R2 zostawia `set_chain()` nietknięte i po prostu przestaje
je wołać.

## 10. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| T1/T2 odrzuci URDF-y, które dziś przechodzą | **realne** | To jest cel, nie skutek uboczny. Ale: oba pliki w `data/urdf` trzeba sprawdzić **przed** implementacją — jeśli któryś ma niezadeklarowany link, zmiana wywali własne fixture'y |
| Testy `KinematicChainBuilder` z ręcznym `RobotDescription` przestaną przechodzić | średnie | Przewidziane w 7; proposal implementacyjny wycenia liczbę |
| Usunięcie `incomplete_kinematic_chain` to zmiana API | niskie | `LoadErrorCode` nie jest publiczne poza `UrdfModelLoader`, który konsumuje je wewnętrznie |
| Nowa walidacja sama ma pętlę po nieufnych danych | **średnie** | `visited` + twardy limit kroków (6); dokładnie ten błąd naprawiamy, więc nie wolno go powtórzyć |
| Utrata ostrzeżenia „< 6 DOF" | niskie | Świadome (4.1); do potwierdzenia w review |

## 11. Do rozstrzygnięcia w review

1. Czy URDF z linkami, ale zerem jointów, przechodzi (4, T5).
2. Czy ostrzeżenie „< 6 DOF" ma się odrodzić w `KinematicChainBuilder` (4.1).
3. Kolejność T5 vs T4 (5.2).
4. Czy T4 raportuje pełną ścieżkę cyklu, czy jedną nazwę (5.3).
5. Jak porównywać „identyczny wynik" w teście niezależności od kolejności (8.2).
6. Czy `RejectsCyclicTopology` polega na limicie kroków, timeoutcie CTest, czy
   obu (8.1).
7. Czy `parse_joint_type`/`<limit>` zostają nietknięte w tym proposalu — tak,
   `<limit>` to R3; potwierdzić, że nie ma między nimi kolizji w tych samych
   liniach.

## 12. Rekomendacja końcowa

Wykonać jako drugi dokument etapu, po R1. Uzasadnienie kolejności: nowa
walidacja dodaje cztery kody do dwóch `switch`-y i bez `-Wswitch` z R1 pominięty
`case` przechodzi po cichu — czyli dokładnie ten sam rodzaj błędu, jaki ten
proposal naprawia.

Gate:

```text
ten sam robot w dwóch uporządkowaniach XML → identyczny wynik
URDF z cyklem → odrzucony, proces się KOŃCZY
URDF rozspójniony → odrzucony, nic nie ginie po cichu
odwołanie do niezadeklarowanego <link> → odrzucone w loaderze
BaseLinkNotFound / ToolLinkNotFound znów mogą zadziałać
oba pliki w data/urdf ładują się bez zmian w geometrii
```
