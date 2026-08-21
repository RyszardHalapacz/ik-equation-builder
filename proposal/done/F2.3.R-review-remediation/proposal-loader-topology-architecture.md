# Proposal: topologia loadera — architektura (R2, v3 — APPROVED)

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

## Historia wersji

**v1 → v2** po review (verdykt: *approve with changes*). Przyjęte wszystkie
siedem wymaganych poprawek:

| # | Poprawka z review | Gdzie |
|---|---|---|
| 1 | brakował inwariant unikalności i niepustości nazw `<link>` | 4, T0a/T0b |
| 2 | przypadek 0 jointów był rozstrzygnięty **źle** i przeczył T5 | 4.2 |
| 3 | `roots == 0` to **pewny cykl**, nie rozspójnienie | 5.1, 6 |
| 4 | algorytm T4 nie zgadzał się z obiecywaną diagnostyką `rawValue` | 5.3, 6.2 |
| 5 | „cztery pozycje" — jest ich **sześć** | 5.1 |
| 6 | R2 samo zmienia obserwowalne zachowanie `RobotModel` → zależność merge R2↔R4 | 7 |
| 7 | arbitralny limit `L + J + 1` do usunięcia | 6.3 |

Plus dwie korekty redakcyjne z review: precyzyjne sformułowanie o zasięgu zmiany
`LoadErrorCode` (8.1) i przeformułowany gate niezależności od kolejności XML (9.2).

**v2 → v3: APPROVED.** Dwa kontrakty implementacyjne dopisane na wniosek review,
bez zmian w architekturze i bez nowych kodów błędów:

| # | Zapis | Gdzie |
|---|---|---|
| 8 | raportowanego diagnostycznie linku nie wolno wybierać przez iterację po kontenerze haszującym | **5.4** (nowa sekcja) |
| 9 | kolejność `T0→…→T5` jest kontraktem w obrębie walidacji topologii, nie globalnym priorytetem błędów loadera | 5.2 |

Punkt 8 jest antyregresyjny: bez niego nowa walidacja odtworzyłaby w ścieżce
komunikatów dokładnie tę zależność od kolejności hashowania, którą R2 usuwa z
`trace_chain`. Punkt 9 wynika ze zweryfikowanej kolejności etapów w `load_urdf`.

**Status: zatwierdzone. Następny dokument — `proposal-loader-topology-implementation.md`.**

Zamknięte decyzją review: ostrzeżenie „< 6 DOF" **usuwamy i nie odtwarzamy**;
pełna ścieżka cyklu **nie jest potrzebna**.

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

**Zweryfikowane w v2:** oba fixture'y spełniają proponowane niezmienniki, więc
nowa walidacja nie odrzuci własnych danych projektu:

```text
kr4_r600.urdf   10 <link>, 10 unikalnych, 0 pustych nazw, wszystkie
                odwołania parent/child mają deklarację
kr640.urdf       8 <link>,  8 unikalnych, 0 pustych nazw, jak wyżej
```

W v1 było to otwartym ryzykiem („do sprawdzenia przed implementacją"). Jest
sprawdzone i zamknięte.

## 1. Cel i zakres

**W zakresie:** ustalenie, co loader ma gwarantować o topologii, i usunięcie
`trace_chain()` jako mechanizmu wyboru ścieżki.

**Poza zakresem:** `<limit>` (R3), projekt `RobotModel`/`within_limits` (R4),
`continuous` jako funkcjonalność, unikalność `joint.name` (nie jest konieczna do
poprawności grafu `parent→child` — świadomie odłożona), jakakolwiek zmiana w
`KinematicChainBuilder` poza usunięciem unii `knownLinks`.

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
nie ma zbioru odwiedzonych. Dla cyklu `root→A, A→B, B→A` korzeń zostaje
znaleziony poprawnie, a potem pętla krąży `A→B→A→B…` w nieskończoność, dopisując
indeksy do `chain` — czyli jest to zawieszenie **z nieograniczonym wzrostem
pamięci**.

`KinematicChainBuilder` **ma** wykrywanie cyklu (`PathSearchResult::CycleDetected`,
`KinematicChainBuilder.cpp:79–80`) — tylko nigdy się do niego nie dochodzi, bo
loader wiesza się wcześniej.

**(c) Wybór korzenia zależy od kolejności iteracji hash mapy** — znalezisko
dodatkowe, objawowo najgroźniejsze:

```cpp
for (auto const& [parent, _] : parent_to_idx) {
    if (!is_child.contains(parent)) { current = parent; break; }
}
```

Przy dwóch rozłącznych poddrzewach oba korzenie spełniają warunek, a `break`
bierze pierwszy **w kolejności iteracji `unordered_map`** — nie w kolejności XML,
tylko w kolejności zależnej od implementacji biblioteki standardowej.

Różnica wagi wobec (a): (a) daje **fałszywe odrzucenie** poprawnego robota —
głośne i łatwe do zauważenia. (c) daje **cichą złą odpowiedź** — model przechodzi
walidację, a połowa robota nie istnieje.

### 2.2 Wynik `trace_chain()` nie jest nigdzie używany

`RobotModel::chain()` nie ma w `src/` konsumenta poza `within_limits()` w tym
samym pliku. `UrdfModelLoader::load` przepisuje `links()` i `joints()` do
`RobotDescription` i **`chain()` ignoruje** (`UrdfModelLoader.cpp:93–97`).

Czyli mamy dwie warstwy wyboru ścieżki: jedną wadliwą i nieużywaną, drugą
poprawną i używaną. Wadliwa może odrzucić model, z którym poprawna poradziłaby
sobie bez problemu.

### 2.3 Brak walidacji istnienia linków

Loader czyta `<link>` do `links_` i osobno `<parent link=...>`/`<child link=...>`
do jointów, **nie sprawdzając, czy odwołanie ma pokrycie w deklaracji**.

`KinematicChainBuilder` to obchodzi unią (`KinematicChainBuilder.cpp:119–129`).
Konsekwencja, której komentarz nie odnotowuje: **`BaseLinkNotFound` i
`ToolLinkNotFound` nie mogą zadziałać dla żadnej nazwy wspomnianej w jakimkolwiek
joincie.** Literówka w `<parent link="bas"/>` tworzy legalny frame `bas`.

### 2.4 Brak walidacji samych deklaracji `<link>` — dodane w v2

```cpp
for (auto node : robot.children("link"))
    out.model.add_link(Link{.name = node.attribute("name").as_string()});
```

Ani niepustość, ani unikalność. Dla:

```xml
<link name="A"/>
<link name="A"/>
```

powstają **dwa** elementy `links_`, ale każdy zbiór nazw zapadnie je w jeden.
To nie jest kosmetyczna walidacja URDF-a: **`link.name` jest w tym kodzie
tożsamością węzła grafu.** Bez unikalnej tożsamości wszystkie dalsze niezmienniki
pracują na niejednoznacznym modelu, więc twierdzenie „graf został zwalidowany"
byłoby puste.

## 3. Decyzja podstawowa: `trace_chain()` znika

| Wariant | Ocena |
|---|---|
| **A.** Naprawić `trace_chain()`: `vector` dzieci, `visited`, wybór gałęzi | Odrzucony. Powstałby drugi, konkurencyjny mechanizm wyboru ścieżki obok `KinematicChainBuilder` — z pytaniem „którą gałąź wybrać", na które loader **nie ma prawa odpowiadać**, bo nie zna `baseLink`/`toolLink`. |
| **B.** Usunąć `trace_chain()`, w jego miejsce walidacja drzewa | **Wybrany.** |
| **C.** Usunąć bez zamiennika | Odrzucony. Cykl i rozspójnienie przeszłyby do `RobotDescription`; `KinematicChainBuilder` wykryje cykl na *swojej ścieżce*, ale nie odrzuci pliku z cyklem w gałęzi bocznej. |

```text
UrdfModelLoader / load_urdf
    parsowanie + walidacja struktury drzewa
    → RobotDescription z gwarancją: to jest poprawne drzewo

KinematicChainBuilder
    wybór konkretnej ścieżki base → tool w tym drzewie
```

## 4. Co znaczy „poprawne drzewo" — siedem niezmienników

| # | Niezmiennik | Dziś |
|---|---|---|
| **T0a** | każdy `<link>` ma niepustą nazwę | **nie sprawdzane** (dodane w v2) |
| **T0b** | nazwy `<link>` są unikalne | **nie sprawdzane** (dodane w v2) |
| T1 | każdy `joint.parent_link` istnieje w `<link>` | nie sprawdzane |
| T2 | każdy `joint.child_link` istnieje w `<link>` | nie sprawdzane |
| T3 | żaden link nie jest dzieckiem dwóch jointów | nie w loaderze; `KinematicChainBuilder` ma `InvalidRobotDescription`, ale tylko dla swojej ścieżki |
| T4 | graf nie zawiera cyklu | **wiesza proces** |
| T5 | dokładnie jeden korzeń — graf jest spójny | **przechodzi i gubi poddrzewo** |

**T0 poprzedza wszystko** i to jest wymuszone, nie stylistyczne: T0b ustala
tożsamość węzłów. Bez niej zbiór wierzchołków grafu nie jest dobrze określony,
a T1–T5 orzekają o obiekcie, którego nie ma.

**T3 jest tym, co czyni graf drzewem.** W drzewie każdy węzeł ma co najwyżej
jednego rodzica. Bez T3 „drzewo" jest DAG-iem i `T_parent_child` traci
jednoznaczność.

### 4.1 Unikalność `joint.name` — świadomie poza zakresem

Nie jest konieczna do poprawności grafu `parent→child`: tożsamością węzła jest
nazwa linku, nie jointu. Dwa jointy o tej samej nazwie i różnych końcach dają
poprawne drzewo. Zostawione poza R2 zgodnie z review; wróci, jeśli okaże się
potrzebne przy diagnostyce.

### 4.2 Przypadek zerowy — v1 rozstrzygało to **źle**

> v1 rekomendowało: „URDF z zerem jointów nie jest błędem, jeśli ma co najmniej
> jeden `<link>`". Review słusznie wykazało, że to **przeczy T5**: dwa linki i
> zero jointów to dwa izolowane węzły, czyli las, nie jedno spójne drzewo.

Semantyka przyjęta w v2:

```text
0 links, 0 joints    → parse_failure          (nie ma czego opisywać)
1 link,  0 joints     → SUCCESS                (legalne drzewo jednoelementowe)
>1 link, 0 joints     → disconnected_topology  (izolowane węzły)
```

Jest to w pełni zgodne z tym, co projekt już dopuszcza: `baseLink == toolLink`
zwraca pusty `KinematicChain` jako sukces. Do obsłużenia pustego łańcucha **nie
trzeba** akceptować wielu rozłącznych linków.

### 4.3 Czego loader dalej **nie** gwarantuje

- że istnieje ścieżka między dowolnymi dwoma linkami — to pytanie
  `KinematicChainBuilder`, zależne od argumentów;
- że robot ma ≥ 6 stopni swobody. Dzisiejsze ostrzeżenie „`chain has N actuated
  joints (expected >=6)`" (`robot_model_loader.cpp:146–151`) **znika razem z
  `trace_chain()` i nie wraca — ani tutaj, ani w `KinematicChainBuilder`.**
  Rozstrzygnięte przez review: 3R może być całkowicie poprawnym mechanizmem, a
  prawo powiedzieć „za mało osi" ma dopiero komponent wymagający konkretnej
  rodziny solvera — przez `UnsupportedKinematicStructure`, w Fazie 3.

## 5. Model błędów

### 5.1 Sześć nowych kodów, nie cztery

> v1 mówiło „cztery pozycje" i wymieniało pięć. Po dodaniu T0 jest ich sześć.

```text
invalid_link_declaration   T0a + T0b   (pusta lub zduplikowana nazwa <link>)
undeclared_parent_link     T1
undeclared_child_link      T2
duplicate_child_link       T3
cyclic_topology            T4
disconnected_topology      T5
```

`invalid_link_declaration` jest jednym kodem dla obu naruszeń T0, bo oba znaczą
to samo dla wołającego: deklaracja linków nie definiuje poprawnego zbioru węzłów.
Rozróżnienie niesie `attribute`/`rawValue`.

`incomplete_kinematic_chain` **zostaje usunięty** — był kodem wyjścia
`trace_chain()` i po jego usunięciu nie ma producenta.

Każdy kod potrzebuje wpisu w `describeCode` (`UrdfModelLoader.cpp:48`). Po R1
brak wpisu wywali build (`-Werror=switch`) — potwierdzone mutation checkiem przy
implementacji R1.

### 5.2 Kolejność sprawdzania jest deterministyczna — w obrębie walidacji topologii

Precedens: `IkEquationSystem::create` wymusza dziewięć niezmienników w ustalonej
kolejności, żeby każdy kod miał jednoznaczne znaczenie, gdy naruszonych jest kilka.

```text
T0 → T1/T2 → T3 → (liczba korzeni) → (osiągalność)
```

> **Zakres kontraktu, doprecyzowany w v3.** Powyższa kolejność jest kontraktem
> **w obrębie walidacji topologii**, wykonywanej po pomyślnym sparsowaniu
> elementów potrzebnych do zbudowania modelu. **Nie definiuje globalnego
> priorytetu** względem błędów składni, geometrii ani limitów.
>
> Zweryfikowane w kodzie: `trace_chain` — a więc i jego następca — jest wołane w
> `load_urdf` na linii 250, **po** całej pętli po `<joint>`, która wychodzi
> wcześniej na `unsupported_joint_type`, `malformed_vector` (trzy miejsca),
> `degenerate_axis` i `invalid_limits`. Plik mający jednocześnie `axis="abc"` i
> odwołanie do niezadeklarowanego linku zgłosi więc `malformed_vector`, nie `T2`.
>
> To jest opis stanu faktycznego, nie postulat zmiany — R2 nie przestawia
> kolejności etapów loadera.

Kolejności **wymuszone, nie arbitralne**:

- **T0 przed wszystkim** — ustala tożsamość węzłów (4).
- **T1/T2 przed T3+** — bez nich zbiór krawędzi wskazuje na węzły, których nie ma;
  zgłoszenie literówki jako `disconnected_topology` byłoby prawdą bezużyteczną.
- **T3 przed liczeniem korzeni** — link z dwoma rodzicami zaburza liczenie korzeni.

### 5.3 Kontekst w błędzie — uzgodniony z tym, co algorytm potrafi

> v1 obiecywało dla T4 „nazwę linku, na którym **zamknął się** cykl". Review
> wykazało, że algorytm tego nie wie — `visited.size() != links.size()` mówi
> tylko, że dany link jest **nieosiągalny z korzenia**. v2 osłabia kontrakt do
> tego, co da się dostarczyć.

| Kod | `attribute` | `rawValue` |
|---|---|---|
| `invalid_link_declaration` | `"link/name"` | nazwa (lub pusto dla T0a) |
| `undeclared_parent_link` | `"parent/link"` | nieznaleziona nazwa; `jointName` = joint z odwołaniem |
| `undeclared_child_link` | `"child/link"` | jak wyżej |
| `duplicate_child_link` | — | nazwa linku; `jointName` = **drugi** joint roszczący sobie to dziecko |
| `cyclic_topology` | — | **pierwszy link należący do komponentu zawierającego cykl** — nie punkt domknięcia |
| `disconnected_topology` | — | nazwa drugiego znalezionego korzenia, lub ich liczba |

Który joint jest „drugi" przy `duplicate_child_link`, zależy od kolejności XML —
i to jest w porządku, bo dotyczy tylko *komunikatu*, nie tego, czy plik przechodzi.

Świadomy wybór: **nie** raportujemy pełnej ścieżki cyklu ani punktu domknięcia.
Wymagałoby to prawdziwego DFS z kolorowaniem w komponencie nieosiągalnym, a review
potwierdziło, że jedna nazwa wystarcza — **pod warunkiem, że tekst nie obiecuje
więcej.** Ta tabela jest tym warunkiem.

### 5.4 Wybór raportowanego linku musi być deterministyczny — dodane w v3

Trzy kody z tabeli wyżej — `invalid_link_declaration`, `cyclic_topology`,
`disconnected_topology` — raportują **jeden element wybrany ze zbioru** kandydatów
(drugi korzeń, dowolny link z komponentu cyklicznego). To jest miejsce, w którym
łatwo wprowadzić z powrotem dokładnie tę wadę, którą R2 usuwa.

**Kontrakt:** kontenery haszujące (`unordered_set`/`unordered_map`) mogą służyć
wyłącznie do sprawdzania przynależności. **Elementu raportowanego diagnostycznie
nie wolno wybierać przez iterację po kontenerze haszującym** — wybór następuje
deterministycznie, przez przejście po `model.links()` w kolejności deklaracji
w XML i wzięcie pierwszego pasującego:

```cpp
// dobrze — kolejność XML, ten sam plik zawsze daje ten sam komunikat
for (auto const& link : links)
    if (isRoot(link.name)) { /* pierwszy korzeń w kolejności XML */ }

// źle — komunikat zależy od implementacji hashowania
for (auto const& name : rootSet) { ... }
```

Uzasadnienie jest wprost antyregresyjne. Dzisiejsza wada (c) z sekcji 2.1 polega
na tym, że `trace_chain` wybiera korzeń przez `for (auto const& [parent, _] :
parent_to_idx)` z `break`. Gdyby nowa walidacja wybierała raportowany link tym
samym sposobem, ten sam plik dawałby różne komunikaty na różnych bibliotekach
standardowych — i to w kodzie, którego celem jest usunięcie zależności od
kolejności iteracji.

**Czego ten kontrakt nie obiecuje:** niezależności komunikatu od kolejności XML.
Po przestawieniu dwóch błędnych linków w pliku diagnostyka może wskazać drugi z
nich, i to jest akceptowalne — plik jest inny. Wymagamy tylko, żeby **to samo
wejście zawsze dawało to samo wyjście.**

Zasięg: dotyczy wyłącznie diagnostyki. **To, czy plik przechodzi**, nigdy nie może
zależeć od kolejności czegokolwiek — to gwarantują T0–T5 i test z 9.2.

## 6. Algorytm

> v1 miało tu niespójność, którą review nazwało najważniejszą rzeczą algorytmiczną:
> deklarowało T4 jako „jawne sprawdzenie", a realizowało je jako **wniosek** z
> `visited != L`. v2 opisuje to jako wniosek, bo nim jest, i naprawia błędną
> klasyfikację `roots == 0`.

### 6.1 Tablica decyzyjna

Po T0, T1, T2 i T3 graf jest dobrze określony i każdy węzeł ma **co najwyżej
jednego rodzica**. Dopiero to czyni poniższe rozumowanie poprawnym.

```text
J == 0:
    L == 0        → parse_failure
    L == 1        → OK
    L >  1        → disconnected_topology

J > 0:
    roots == 0    → cyclic_topology          (v1 mówiło disconnected — BŁĄD)
    roots >  1    → disconnected_topology
    roots == 1    → traversal(root)
                      visited == L → OK
                      visited != L → cyclic_topology
```

### 6.2 Dlaczego wnioskowanie jest poprawne

Oba wnioski o cyklu opierają się na T3 (każdy węzeł ma ≤ 1 rodzica) i muszą być
tak opisane, a nie sprzedawane jako bezpośrednia detekcja:

**`roots == 0` przy `L > 0` ⇒ cykl jest pewny.** Skończony niepusty graf, w którym
każdy węzeł ma rodzica, musi zawierać cykl — idąc w górę po rodzicach nie da się
zatrzymać, a węzłów jest skończenie wiele. Nazwanie tego „rozspójnieniem" było
błędem klasyfikacji: mamy osobny, dokładniejszy kod.

**`visited != L` przy `roots == 1` ⇒ cykl jest pewny.** Każdy nieodwiedzony
komponent nie może mieć własnego korzenia, bo korzeni globalnie jest dokładnie
jeden i leży w komponencie odwiedzonym. Zatem w tym komponencie każdy węzeł ma
rodzica, więc — jak wyżej — zawiera cykl.

To jest ta sama struktura argumentu, którą v1 stosowało do T4 („wynika częściowo
z T3 + T5"), tylko doprowadzona do końca i zapisana jako uzasadnienie, a nie jako
obietnica jawnej detekcji.

### 6.3 Gwarancja zakończenia — bez arbitralnego licznika

> v1 proponowało twardy limit `L + J + 1` iteracji „jako pas bezpieczeństwa".
> Review to odrzuciło: iteracyjny BFS/DFS z `visited` po skończonych kontenerach
> **już** ma gwarancję zakończenia, a `L + J + 1` zależy od tego, co nazwiemy
> „krokiem", więc łatwo wyprodukować fałszywy alarm przy legalnej implementacji.
> Zgadzam się — to był licznik pozorny.

Zakończenie zapewnia `visited` nad skończonym zbiorem węzłów; każdy węzeł wchodzi
na stos co najwyżej raz. Jeśli chcemy dodatkowej asercji, wiążemy ją z algorytmem,
nie z abstrakcyjnym „krokiem":

```text
processedVertices <= links.size()
processedEdges    <= joints.size()
```

Timeout CTest zostaje jako **zewnętrzna** siatka bezpieczeństwa, nie jako element
poprawności.

Druga decyzja implementacyjna, z precedensem: **iteracyjnie, nie rekurencyjnie.**
`KinematicChainBuilder` używa jawnego stosu, a `STATUS.md` odnotowuje „Recursion
depth scaling unmeasured" jako lukę. Nie dokładamy rekurencji po nieufnych danych
z pliku.

Złożoność: `O(L + J)` czasu i pamięci, jedno przejście na każdy niezmiennik.

## 7. R2 **nie jest** samodzielnym stanem releasowalnym — zależność R2 ↔ R4

> Dodane w v2. v1 zauważało, że `set_chain()` traci producenta, i odsyłało do R4.
> Review wykazało, że konsekwencja jest większa: to **obserwowalna zmiana
> zachowania publicznych metod** `mt::kinematics::RobotModel`.

Dziś po `load_urdf("kr640.urdf")` model ma wypełnione `chain_` (6 indeksów).
Po R2 `chain_` jest **puste dla każdego robota**, bo nikt go już nie ustawia. A:

```cpp
bool RobotModel::within_limits(std::span<double const> joint_values) const noexcept {
    if (joint_values.size() != chain_.size()) return false;
    for (std::size_t k = 0; k < chain_.size(); ++k) { ... }
    return true;
}
```

daje wtedy `within_limits({}) == true` dla dowolnego robota — pusty wektor
„mieści się w limitach" pustego łańcucha. Funkcja przestaje cokolwiek znaczyć.

Dwa warianty i wybór:

| Wariant | Ocena |
|---|---|
| **A.** R2 i R4 są osobnymi proposalami, ale R2 nie jest releasowalne bez R4 | **Wybrany** (rekomendacja review) |
| **B.** R2 tymczasowo neutralizuje ten kontrakt | Odrzucony — miesza zakresy, a R4 istnieje dokładnie po to |

**Zapis wiążący:** R2 może być zaimplementowane i zrecenzowane osobno, ale
**nie wolno scalać remediacji do `main` z R2 bez R4.** Jednostka releasowalna:

```text
R1  →  (R2 + R4)  →  R3  →  R5  →  F2.4
```

R4 musi wejść w tym samym branchu przed merge. Bez tego zostawiamy w repo API,
które wygląda na działające i zwraca `true` na wszystko.

## 8. Konsekwencje w innych plikach

### 8.1 Zasięg zmiany `LoadErrorCode` — precyzyjniej niż w v1

> v1 pisało „`LoadErrorCode` nie jest publiczne poza `UrdfModelLoader`". To
> nieprawda: typ mieszka w `src/kinematics/robot_model_loader.hpp`, a `load_urdf`
> publicznie zwraca `std::expected<LoadResult, LoadError>`.

Precyzyjnie: zmiana **nie wpływa na publiczną fasadę `IkEquationBuilder`**, która
konsumuje `LoadError` wewnętrznie i zamienia na komunikat. W warstwie
`mt::kinematics` jest to **source-level zmiana enuma**, a repo nie ma dziś
zewnętrznego konsumenta tej warstwy.

### 8.2 Unia `knownLinks` w `KinematicChainBuilder` znika

Po T1/T2 `RobotDescription.links` jest wiarygodnym źródłem „czy ten link istnieje",
więc `KinematicChainBuilder.cpp:119–129` redukuje się do:

```cpp
std::unordered_set<std::string_view> knownLinks;
for (auto const& link : robot.links)
    knownLinks.insert(link.name);
```

To **nie kosmetyka** — to przywrócenie znaczenia `BaseLinkNotFound` i
`ToolLinkNotFound`.

Zawężenie: gwarancje T0–T5 są własnością **loadera**, nie typu. `RobotDescription`
to agregat i ręcznie zbudowany nie gwarantuje niczego — tak samo jak dziś z
geometrią (`RobotDescription.hpp:16–17`). Testy `KinematicChainBuilder` budujące
`RobotDescription` z ręki będą musiały deklarować linki, których dziś nie
deklarują. Proposal implementacyjny wycenia to w liczbie zmienionych testów.

## 9. Plan testów

### 9.1 Topologia loadera (13)

```text
AcceptsBranchingTree                     KR4-podobny kształt, dwa rozgałęzienia
RejectsEmptyLinkName                     T0a
RejectsDuplicateLinkName                 T0b
RejectsUndeclaredParentLink              T1
RejectsUndeclaredChildLink               T2
RejectsLinkClaimedByTwoJoints            T3
RejectsCyclicTopologyWithNoRoot          roots == 0 → cyclic_topology
RejectsCyclicTopologyUnreachableFromRoot roots == 1, visited != L → cyclic_topology
RejectsDisconnectedForest                roots > 1
RejectsMultipleIsolatedLinksWithNoJoints L > 1, J == 0 → disconnected
AcceptsSingleLinkWithNoJoints            L == 1, J == 0 → OK
RejectsEmptyRobot                        L == 0, J == 0 → parse_failure
DiagnosticOrderIsDeterministic           plik łamiący T0 i T5 → zawsze T0
```

Dwa testy cyklu, nie jeden — to dwie różne gałęzie tablicy z 6.1 i dwa różne
uzasadnienia z 6.2.

**Oba testy cyklu muszą wykrywać zawieszenie, nie tylko zły kod błędu.** Dziś taki
test nie „nie przechodzi" — zawiesza cały `ctest`. Poprawność zapewnia `visited`
(6.3); dodatkowo `set_tests_properties(... TIMEOUT ...)` jako siatka zewnętrzna.

### 9.2 Niezależność od kolejności XML (2) — istota etapu

```text
LoadsEquivalentModelRegardlessOfJointOrderInXml
BuildsSameKr4ChainRegardlessOfJointOrder
```

> Gate przeformułowany po review. v1 mówiło „identyczny wynik", co jest nieostre,
> bo wektor jointów zachowuje kolejność źródła.

Kryterium:

```text
RobotDescription topologicznie i geometrycznie RÓWNOWAŻNY
    linki po nazwach (zbiór)
    jointy po nazwach i relacji parent→child (zbiór)
    geometria jointów (origin, axis, limits) porównana po nazwie
+
IDENTYCZNY wybrany KinematicChain dla base_link → tool0
```

Drugi warunek jest mocniejszy i bliższy temu, na czym zależy: dwa uporządkowania
tego samego pliku muszą dać ten sam łańcuch kinematyczny.

### 9.3 `KinematicChainBuilder` po usunięciu unii (2)

```text
RejectsBaseLinkNotDeclaredAsLink
RejectsToolLinkNotDeclaredAsLink
```

Oba dziś **nie przechodzą** (frame jest akceptowany) — to test na uwagę 3.

### 9.4 Do przejrzenia, nie do dopisania

`incomplete_kinematic_chain` znika, więc każdy istniejący test oczekujący tego
kodu wymaga decyzji: czy dany plik ma teraz przechodzić, czy padać na innym
kodzie. Proposal implementacyjny musi wymienić te testy z nazwy — jest to jedyne
miejsce, gdzie ten etap może cicho zmienić obserwowalne zachowanie.

## 10. Plan zmian w plikach

```text
src/kinematics/robot_model.hpp           (bez zmian w R2 — patrz 7, to R4)
src/kinematics/robot_model_loader.hpp    LoadErrorCode: +6, -1; kontrakt w komentarzu
src/kinematics/robot_model_loader.cpp    -trace_chain (42 linie), +validate_topology
src/ik_equations/UrdfModelLoader.cpp     describeCode: +6 wpisów, -1
src/ik_equations/builders/KinematicChainBuilder.cpp   -unia knownLinks (8 linii)
tests/test_urdf_model_loader.cpp         +15
tests/test_kinematic_chain_builder.cpp   +2, przegląd RobotDescription w istniejących
data/urdf/                               nowe fixture'y dla przypadków negatywnych
```

## 11. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| T0–T2 odrzucą URDF-y, które dziś przechodzą | **zamierzone** | To cel. Ryzyko dla **własnych** fixture'ów **zamknięte w v2**: oba pliki sprawdzone, 10/10 i 8/8 unikalnych nazw, zero pustych, wszystkie odwołania z deklaracją |
| **R2 bez R4 psuje `within_limits`** | **wysokie** | Sekcja 7, wariant A — zakaz merge'a bez R4 |
| Testy `KinematicChainBuilder` z ręcznym `RobotDescription` przestaną przechodzić | średnie | 8.2; proposal implementacyjny wycenia liczbę |
| Nowa walidacja sama ma pętlę po nieufnych danych | średnie | `visited` nad skończonym zbiorem (6.3); dokładnie ten błąd naprawiamy, więc nie wolno go powtórzyć |
| Utrata ostrzeżenia „< 6 DOF" | niskie | Rozstrzygnięte przez review: usunąć i nie odtwarzać (4.3) |

## 12. Do rozstrzygnięcia w review (v2)

Zamknięte w v2: T0 (dodane), przypadek zerowy (poprawiony), `roots == 0` → cykl,
uzgodnienie algorytmu z `rawValue`, liczba kodów, zależność R2↔R4, limit kroków,
„< 6 DOF", pełna ścieżka cyklu, sformułowanie o zasięgu `LoadErrorCode`, gate
niezależności od kolejności XML.

Pozostaje otwarte:

1. Czy `invalid_link_declaration` ma być jednym kodem dla T0a i T0b (5.1), czy
   dwoma. Rekomenduję jeden — dla wołającego znaczą to samo.
2. Czy `disconnected_topology` przy `L > 1, J == 0` to właściwy kod, czy zasługuje
   na osobny (np. `no_joints`). Rekomenduję istniejący: przyczyna jest ta sama —
   graf nie jest jednym spójnym drzewem.
3. Czy `duplicate_child_link` ma wskazywać pierwszy czy drugi joint (5.3).
4. Kolejność `T4`/`T5` przestała być pytaniem — tablica z 6.1 rozstrzyga oba
   przypadki jednoznacznie. Do potwierdzenia, że tak ma zostać.

## 13. Rekomendacja końcowa

Wykonać jako drugi dokument etapu, po R1 (zaimplementowanym). Nowa walidacja
dodaje sześć kodów do dwóch `switch`-y, a mechanizm z R1 już potwierdzono
działający — pominięty `case` wywali build.

Gate:

```text
ten sam robot w dwóch uporządkowaniach XML → równoważny RobotDescription
                                             + identyczny KinematicChain
URDF z cyklem (oba warianty) → cyclic_topology, proces się KOŃCZY
URDF rozspójniony → disconnected_topology, nic nie ginie po cichu
odwołanie do niezadeklarowanego <link> → odrzucone w loaderze
zduplikowana nazwa <link> → odrzucona
BaseLinkNotFound / ToolLinkNotFound znów mogą zadziałać
oba pliki w data/urdf ładują się bez zmian w geometrii
R4 w tym samym branchu przed merge (sekcja 7)
```
