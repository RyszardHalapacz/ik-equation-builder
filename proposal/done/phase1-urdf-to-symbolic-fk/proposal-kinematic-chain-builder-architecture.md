# Proposal: `KinematicChainBuilder` — architektura (bez implementacji)

## Uwaga o formacie tego dokumentu

Ten proposal **celowo odbiega** od standardowego formatu (prompt / stan obecny / pełny kod zmian) ustalonego wcześniej dla tego repo: na wyraźne życzenie z promptu **nie zawiera kodu C++** i **nie tworzy żadnych plików `.hpp`/`.cpp`**. To jest wyłącznie dokument architektoniczny. Implementacja (z pełnym kodem, zgodnie ze standardowym workflow) będzie osobnym proposalem, dopiero po zatwierdzeniu tego dokumentu — patrz sekcja 15.

## Prompt

> Jesteś architektem C++ oraz robotics software engineer pracującym nad projektem KinemaForge. [...] Kolejnym komponentem jest KinematicChainBuilder. Nie chcemy jeszcze implementacji. Chcemy przygotować pełny proposal architektoniczny. [...] Builder ma otrzymać RobotDescription oraz wskazany base link i tool link. Jego zadaniem jest znalezienie dokładnie jednej ścieżki kinematycznej pomiędzy nimi oraz zwrócenie uporządkowanego modelu KinematicChain. [...] Builder nie wykonuje żadnych obliczeń symbolicznych, nie tworzy macierzy, nie liczy FK — jedynie przygotowuje uporządkowaną reprezentację łańcucha.
>
> *(pełna treść promptu — 12 wymaganych sekcji + 7 zagadnień szczególnych + lista testów — znajduje się w wiadomości użytkownika w tej konwersacji; nie powielam jej tu w całości, żeby nie dublować tekstu).*

## 1. Motywacja i miejsce w pipeline

```
URDF → UrdfModelLoader → RobotDescription → KinematicChainBuilder → KinematicChain
     → JointTransformBuilder → SymbolicTransform → ForwardKinematicsBuilder → FK
```

`UrdfModelLoader` jest gotowy i przetestowany (9 testów, zob. `proposal-loader-test-coverage.md`). Jego kontrakt jest już ustalony i istotny dla tego dokumentu: **`origin.translation`, `origin.rpy` i `axis` są przepisywane 1:1, bez żadnej kompozycji geometrycznej** — `RobotDescription` to czysto strukturalny opis robota, nie licz na to, że coś w nim jest już "obrócone" czy "uproszczone".

`KinematicChainBuilder` jest kolejnym ogniwem: zamienia płaską listę wszystkich jointów robota (`RobotDescription.joints`, w kolejności zapisu w URDF, niekoniecznie w kolejności topologicznej) na **uporządkowaną ścieżkę** pomiędzy dwoma wskazanymi linkami, gotową do skonsumowania przez `JointTransformBuilder` joint po joincie.

## 2. Stan obecny — co już jest zadeklarowane w repo

Repo ma już zarys modelu danych (same deklaracje, bez implementacji `.cpp`), który częściowo *już przesądza* odpowiedzi na pytania 1 i 2 z promptu. Poniżej opis pól (nie kod) tego, co istnieje dziś:

**`RobotDescription`** (`src/ik_equations/model/RobotDescription.hpp`)

| Pole | Typ | Znaczenie |
|---|---|---|
| `name` | string | nazwa robota z `<robot name="...">` |
| `links` | lista `UrdfLink` | wszystkie linki (tylko `name`) |
| `joints` | lista `UrdfJoint` | wszystkie jointy, **w kolejności iteracji po `<joint>` w pliku XML** |

**`UrdfJoint`** (`src/ik_equations/model/UrdfJoint.hpp`) — pola: `name`, `parentLink`, `childLink`, `type` (`Fixed/Revolute/Continuous/Prismatic`), `origin` (`translation: Vector3`, `rpy: Vector3`), `axis: Vector3`, `limits` (`lower/upper/velocity/effort/hasPositionLimits`).

**`KinematicJoint`** (`src/ik_equations/model/KinematicChain.hpp`) — pola: `index`, `name`, `parentLink`, `childLink`, `type`, `origin`, `axis`, `limits`, **plus** `variable: optional<JointVariable>`. To jest **już własna struktura**, nie `UrdfJoint` — duplikuje jego pola zamiast go zawierać.

**`KinematicChain`** — pola: `baseLink`, `toolLink`, `joints: vector<KinematicJoint>`.

**`JointVariable`** (`src/ik_equations/model/JointVariable.hpp`) — pola: `name` (string), `index` (`size_t`).

**`KinematicChainBuilder`** (`src/ik_equations/builders/KinematicChainBuilder.hpp`) — zadeklarowana jedna metoda: `build(robot, baseLink, toolLink) -> KinematicChain`, `const`. Brak `.cpp`.

Ten dokument traktuje ten istniejący szkielet jako punkt wyjścia i albo go **potwierdza** (z uzasadnieniem), albo **proponuje konkretną korektę** — nie projektuje modelu danych od zera.

## 3. Odpowiedzialność klasy

### 3.0 Dlaczego w ogóle potrzebny jest osobny model `KinematicChain`, a nie przekazanie `RobotDescription` wprost?

To jest decyzja poprzedzająca wszystkie inne w tym dokumencie i zasługuje na jawne uzasadnienie, nie tylko domyślne założenie.

`RobotDescription` opisuje **całego robota**: wszystkie linki i jointy z URDF, łącznie z tymi, które nie leżą na żądanej ścieżce base→tool (np. gałąź `link6-flange` w KR4, czujniki, chwytaki, alternatywne narzędzia zamontowane jako boczne gałęzie). `RobotDescription.joints` nie jest nawet gwarantowane jako posortowane topologicznie — to surowa lista w kolejności zapisu w pliku XML (sekcja 7).

Gdyby `JointTransformBuilder`/`ForwardKinematicsBuilder` dostawały `RobotDescription` wprost, każdy z nich musiałby albo (a) samodzielnie rozwiązywać, które jointy i w jakiej kolejności tworzą żądaną ścieżkę — czyli duplikować dokładnie tę logikę, którą ma implementować `KinematicChainBuilder` — albo (b) po cichu zakładać, że `RobotDescription.joints` już jest tą ścieżką, co jest fałszywe dla każdego robota z rozgałęzieniem (znowu: KR4). `KinematicChain` istnieje więc po to, żeby **rozwiązanie topologii zdarzyło się dokładnie raz, w jednym miejscu**, a kolejne etapy dostawały już gotową, jednoznaczną, uporządkowaną odpowiedź na pytanie "które jointy i w jakiej kolejności" — nie znając nic o reszcie robota. To ten sam powód, dla którego istnieje `RobotDescription` zamiast każdego etapu parsującego XML samodzielnie: węższy, celowy model pośredni na granicy dwóch odpowiedzialności.

**W zakresie:**
- znalezienie jedynej ścieżki base→tool na grafie link-joint-link,
- zachowanie kolejności jointów wzdłuż tej ścieżki,
- zachowanie fixed jointów na ścieżce (nie tylko actuated),
- ponumerowanie **wyłącznie** actuated jointów jako `q1..qn`,
- walidacja wejścia (base/tool istnieją, ścieżka istnieje),
- defensywne zabezpieczenie przechodzenia grafu przed nieskończoną rekurencją — **to nie jest pełna walidacja poprawności `RobotDescription`**, tylko zabezpieczenie własnego algorytmu (szczegóły: sekcja 9).

**Poza zakresem** (explicit z promptu, potwierdzone przez istniejące API sąsiednich klas):
- żadnej matematyki symbolicznej — nie tworzy `Expression`/`SymbolExpression`, nawet dla nazwy zmiennej (`JointVariable.name` to zwykły `std::string`, symbol z niego zrobi dopiero `JointTransformBuilder` przez `ExpressionFactory::symbol(name)`),
- żadnych macierzy (`SymbolicMatrix`/`SymbolicTransform`),
- żadnego FK,
- żadnego parsowania URDF (to już zrobił `UrdfModelLoader`).
- Ograniczenia zadaniowe, takie jak wymuszona orientacja TCP lub kierunek osi narzędzia, nie są własnością łańcucha kinematycznego i będą obsługiwane przez późniejszy etap budowania układu równań IK (`ConstraintBuilder` — zob. `analysis-ik-pipeline-constraint-builder.md`).

## 4. Wejście i wyjście

- **Wejście:** `RobotDescription const&`, `baseLink: string`, `toolLink: string`.
- **Wyjście:** `std::expected<KinematicChain, KinematicChainError>` (sukces — `KinematicChain` przez wartość, uzasadnienie w sekcji 5.2; błąd — `KinematicChainError`, sekcja 10/11).
- **Kontrakt:** metoda jest **czysta** (bez efektów ubocznych, bez stanu w klasie) — dokładnie tak, jak `UrdfModelLoader::load` i jak zadeklarowane już `ForwardKinematicsBuilder::build`/`JointTransformBuilder::build`. To jest spójne z filozofią projektu opisaną w README: *"No caching, no hidden state carried between robots."*

## 5. Struktury danych

### 5.1 Pytanie 1 — `UrdfJoint` wewnątrz `KinematicChain`, czy własna struktura?

| | Osadzenie `UrdfJoint` (bezpośrednio lub przez kompozycję) | Własna struktura (`KinematicJoint`, obecny stan) |
|---|---|---|
| **Zalety** | Zero duplikacji pól; jedno źródło prawdy dla geometrii jointu | `KinematicChain` ma stabilny, wąski kontrakt niezależny od tego, co URDF-parsing kiedykolwiek dorzuci (visual, collision, materiały, dynamics — `UrdfJoint` dziś jest oszczędny, ale to warstwa parsowania URDF, naturalnie przyciąga więcej pól w miarę rozwoju loadera) |
| **Wady** | Każde przyszłe pole potrzebne tylko `UrdfJoint`-owi (np. coś z `<visual>`) automatycznie "przecieka" do `KinematicChain`, mimo że kolejne etapy (`JointTransformBuilder`) go nie potrzebują | Duplikacja pól (`name`, `parentLink`, `childLink`, `type`, `origin`, `axis`, `limits`) — dwa miejsca do synchronizacji przy zmianie `UrdfJoint` |
| **Sprzężenie etapów** | `KinematicChainBuilder` i `JointTransformBuilder` stają się sprzężone z kształtem danych warstwy parsowania URDF | Każdy etap pipeline'u zna tylko strukturę zaprojektowaną *dla niego* — zgodne z README: *"Each stage owns exactly one responsibility [...] future stages [...] can be added without changing the earlier pipeline"* |

**Rekomendacja: własna struktura (`KinematicJoint`), zgodnie z tym, co już jest zadeklarowane w repo.** Powód decydujący: to nie jest kompozycja 1:1 — `KinematicJoint` dokłada `index` (pozycja w łańcuchu) i `variable` (obecne tylko dla actuated), których `UrdfJoint` strukturalnie nie ma i mieć nie powinien (to nie jest własność jointu z URDF, tylko własność *tego konkretnego przebiegu* budowania łańcucha). Skoro i tak potrzebna jest nowa struktura na te dwa pola, korzystniejsze jest, żeby ta struktura niosła własne kopie pól geometrii, niż żeby `JointTransformBuilder` (konsument) musiał znać zarówno `KinematicJoint`, jak i zagnieżdżony w nim `UrdfJoint`.

### 5.2 Pytanie 2 — kopie danych, czy wskaźniki/referencje do `RobotDescription`?

| Kryterium | Referencje/wskaźniki do `RobotDescription` | Kopie (obecny stan) |
|---|---|---|
| **Ownership** | `KinematicChain` nie jest właścicielem danych — właścicielem pozostaje wywołujący, który musi utrzymać `RobotDescription` przy życiu | `KinematicChain` jest w pełni samodzielny, właścicielem jest on sam |
| **Lifetime** | Niebezpieczne: `IkEquationBuilder::selectChain()` mogłoby zbudować `KinematicChain` odwołujący się do `robotDescription_`, a każde późniejsze `loadRobotModel()` (przeładowanie) unieważni te referencje bez ostrzeżenia kompilatora | Brak problemu — `KinematicChain` żyje niezależnie od `RobotDescription` |
| **Bezpieczeństwo** | Wymaga ręcznej dyscypliny (dangling reference to klasyczne źródło UB w C++); trudne do przetestowania w izolacji (test musi utrzymać `RobotDescription` dokładnie tak długo, jak `KinematicChain`) | Value semantics — kompilator/`ASan` łapie oczywiste błędy, testy budują `RobotDescription` lokalnie i od razu z niej korzystają bez zarządzania czasem życia |
| **Prostota** | Wymaga API z jawnym czasem życia (np. `std::span`/`std::reference_wrapper`, dokumentacja "musisz utrzymać X przy życiu") | Zero dodatkowej dokumentacji na temat czasem życia |
| **Koszt** | Brak kopiowania stringów/doubli | Kopiowanie ~10-30 małych structów (typowy robot przemysłowy: 6-9 jointów) — rząd wielkości: pojedyncze mikrosekundy. Dla porównania: sam moduł buduje się w kierunku *symbolicznej* matematyki (drzewa wyrażeń, alokacje `shared_ptr`), gdzie ten koszt jest całkowicie pomijalny |
| **Przyszła rozbudowa** | Utrudnia dodanie np. cache'owania łańcuchów albo równoległego przetwarzania wielu łańcuchów z tego samego `RobotDescription` (aliasing) | `KinematicChain` można swobodnie przekazywać, kopiować, przechowywać w kontenerach, zwracać z funkcji — bez adnotacji czasu życia |

**Rekomendacja: kopie (value semantics), zgodnie z obecnym stanem struktur.** Przy tej skali danych (pojedyncze cyfry–dziesiątki jointów) argument wydajnościowy za referencjami jest fikcyjny, a koszt bezpieczeństwa/prostoty realny — zwłaszcza że `IkEquationBuilder` (fasada) i tak trzyma `RobotDescription` oraz `KinematicChain` jako osobne pola o tym samym czasie życia (zob. `IkEquationBuilder.hpp`), więc referencje nie dają tu żadnej korzyści, tylko ryzyko.

### 5.3 Finalny model danych — doprecyzowanie semantyki pól

Model jest w porządku strukturalnie; wymaga jednak **doprecyzowania dwóch rzeczy**, które dziś są niejednoznaczne, bo istnieją dwa różne pola o nazwie `index`:

1. **`KinematicJoint::index`** = pozycja jointu w `KinematicChain::joints` (0-based, zgodna z indeksowaniem wektora). Dotyczy **każdego** jointu na ścieżce, actuated i fixed.
2. **`JointVariable::index`** = numer zmiennej symbolicznej (1-based, żeby wprost odpowiadał sufiksowi `"q" + index`, tj. `q1` ↔ `index == 1`). Dotyczy **tylko** actuated jointów.

To są dwa niezależne liczniki idące różnym tempem (fixed jointy inkrementują pierwszy, nie inkrementują drugiego) — to jest dokładnie ten rodzaj niejawnej niespójności, którą proposal ma wychwycić przed napisaniem kodu, a nie po.

Reszta pól `KinematicJoint` (`name`, `parentLink`, `childLink`, `type`, `origin`, `axis`, `limits`) to **wierne kopie** odpowiednich pól z dopasowanego `UrdfJoint` — żadna transformacja geometryczna się tu nie dzieje (to zadanie `JointTransformBuilder`).

## 6. Fixed joints — dlaczego muszą zostać zachowane (pytanie 3)

`ForwardKinematicsBuilder` liczy FK jako **iloczyn transformacji wszystkich jointów na ścieżce, w kolejności** (opis w README: *"multiplies the whole chain of transforms together into one final symbolic transform"*). `JointTransformBuilder` z kolei — również już zadeklarowany — ma zamieniać *pojedynczy* joint (dowolnego typu) na transformację 4×4, gdzie fixed joint wnosi **translację + stały obrót z `origin`**, a actuated joint dokłada do tego dodatkowo **obrót sparametryzowany zmienną**. Fixed joint nie jest więc "pomijalny" — jest częścią iloczynu transformacji, tak samo jak actuated, tylko bez zmiennej.

Jeśli fixed jointy zostałyby usunięte z łańcucha, straciłby się **fizyczny offset**, jaki reprezentują — a to nie jest offset zerowy. Konkretny przykład z `kr4_r600.urdf`:

```
base_link --[fixed: base_link-base]--> base      (martwa gałąź, base_link ma dwoje dzieci)
base_link --[revolute q1: joint_1]--> link_1
...
link_5    --[revolute q6: joint_6]--> link_6
link_6    --[fixed: link6-flange]--> flange      (martwa gałąź, nie prowadzi do tool0)
link_6    --[fixed: link6-tool0]--> tool0         (xyz="0 0 -0.0285", rpy≈(pi, 0, pi))
```

(`joint_1`'s `parentLink` to `base_link` bezpośrednio — `base` to ślepa gałąź, do niej nic dalej nie prowadzi, tak samo jak `flange`.)

Gdyby builder pominął `link6-tool0` jako "tylko fixed", FK dla `toolLink="tool0"` kończyłoby się na `link_6` — czyli 2.85 cm i pełny obrót o `rpy≈(pi,0,pi)` za wcześnie. To byłby błąd geometryczny, nie kosmetyczny.

Ten sam przykład pokazuje coś jeszcze ważniejszego dla algorytmu (sekcja 8): **zarówno `base_link`, jak i `link_6` mają dwoje dzieci** (`base`/`link_1`, oraz `flange`/`tool0`) — robot nie jest tu prostą listą, tylko drzewem z rozgałęzieniami na obu końcach. Algorytm musi to poprawnie obsłużyć, a rozwiązana ścieżka `base_link → tool0` ma finalnie 7 jointów (`joint_1..joint_6` + `link6-tool0`), nie 9 — oba fixed dead-endy (`base_link-base`, `link6-flange`) zostają poza wynikiem.

## 7. Numeracja zmiennych `q1..qn` (pytanie 4)

Zasada: zmienną (`JointVariable`) dostaje **wyłącznie** joint, którego `type` oznacza stopień swobody — dziś `Revolute` i `Prismatic`. `Fixed` nigdy nie dostaje zmiennej (`variable = nullopt`).

**Otwarta uwaga o `Continuous`:** enum `JointType` ma wartość `Continuous`, ale `UrdfModelLoader`/`robot_model_loader.cpp` jej dziś w ogóle nie obsługuje (URDF z `type="continuous"` kończy się błędem parsowania na wcześniejszym etapie — zob. `proposal-loader-test-coverage.md`, sekcja "Znane luki"). Innymi słowy: `KinematicChainBuilder` **nigdy nie zobaczy** dziś jointu typu `Continuous` w praktyce. Rekomendacja: mimo to od razu potraktować `Continuous` jako actuated w logice numerowania (razem z `Revolute`/`Prismatic`) — koszt to jeden dodatkowy `case`, korzyść to brak przyszłej niespodzianki, gdy loader dogoni obsługę tego typu. To decyzja tania i defensywna, nie zwiększa zakresu tego etapu.

**Kolejność numeracji:** ściśle wzdłuż **rozwiązanej ścieżki base→tool**, od `baseLink` do `toolLink`, nie wg kolejności w `RobotDescription.joints` (ta może, ale nie musi, odpowiadać topologii — `UrdfModelLoader` niczego nie sortuje, przepisuje jointy w kolejności ich wystąpienia w pliku XML). Pierwszy napotkany actuated joint na ścieżce dostaje `q1`, kolejny `q2`, itd. — dokładnie zgodnie z opisem w README: *"actuated joints assigned symbolic variables (q1, q2, ...)"*.

Model danych: `JointVariable{name: "q" + std::to_string(n), index: n}`, gdzie `n` startuje od `1` i rośnie tylko na actuated jointach (patrz sekcja 5.3 o dwóch licznikach).

## 8. Algorytm znajdowania ścieżki (pytanie 5)

### 8.1 Charakterystyka grafu

`RobotDescription.joints` to zbiór krawędzi skierowanych `parentLink -> childLink` (jeden joint = jedna krawędź). Poprawny URDF opisuje **drzewo**: każdy link ma co najwyżej jednego rodzica (jest dzieckiem co najwyżej jednego jointu), ale może mieć wielu dzieci (przykład `link_6` w sekcji 6). Z własności drzewa wynika ważny fakt: **jeśli `toolLink` jest potomkiem `baseLink`, ścieżka między nimi jest unikalna** — nie ma więc potrzeby algorytmu do grafów ogólnych (np. Dijkstra, wszystkie ścieżki) — wystarczy przeszukanie drzewa.

**Świadome ograniczenie zakresu:** przeszukiwanie idzie **tylko w dół**, wzdłuż krawędzi `parent -> child`. To pokrywa jedyny przypadek użycia tego etapu projektu (pojedyncze ramię przemysłowe, `baseLink` jest przodkiem `toolLink`). Łańcuchy "w bok" (np. relatywna kinematyka między dwoma narzędziami różnych ramion) są poza zakresem — jeśli kiedyś będą potrzebne, to osobny, świadomy proposal, nie coś, co powinno się przemycić tutaj.

### 8.2 DFS vs BFS

Słuszna uwaga z review: to jest drzewo, nie graf ogólny, więc wybór między DFS i BFS **nie jest decyzją o dużej wadze** — na drzewie oba znajdą ten sam, jedyny istniejący wynik. Krótkie uzasadnienie zamiast pełnego porównania: **DFS**, bo ścieżka to po prostu bieżący stos wywołań/wektor w trakcie przejścia (bez osobnej fazy odtwarzania z mapy `came_from`, której wymagałby BFS), a backtracking przy rozgałęzieniu (`link_6` → `flange`/`tool0` w KR4) jest jego naturalnym zachowaniem, nie dodatkiem. To wybór "domyślny i wystarczający", nie wynik głębszej analizy kompromisów.

### 8.3 Kroki algorytmu

1. **Budowa grafu:** mapa `parentLink -> lista (indeks jointu w RobotDescription.joints, childLink)`, budowana jednym przejściem po `RobotDescription.joints`. Przy tej samej okazji (bez osobnego przejścia — patrz sekcja 9) opcjonalnie odnotowujemy, czy dany `childLink` już wcześniej wystąpił jako dziecko innego jointu.
2. **Walidacja istnienia `baseLink`/`toolLink`** względem zbioru "znanych linków" = suma nazw z `RobotDescription.links` **oraz** wszystkich `parentLink`/`childLink` z jointów (nie polegamy wyłącznie na `RobotDescription.links` — `UrdfModelLoader`/`robot_model_loader.cpp` nie waliduje dziś spójności między `<link>` a referencjami w `<joint>`, więc grunt prawdy dla topologii to i tak krawędzie).
3. **Przypadek szczególny `baseLink == toolLink`** — patrz "otwarte pytanie" w sekcji 14; rekomendacja robocza: zwróć pusty (ale poprawny) `KinematicChain` (identyczność, brak jointów) zamiast traktować to jako błąd.
4. **DFS od `baseLink`**, wyłącznie po krawędziach w dół, z akumulacją ścieżki i zbiorem odwiedzonych linków **na bieżącej gałęzi** (czysto defensywne zabezpieczenie przed nieskończoną rekurencją, nie "walidacja modelu" — patrz sekcja 9):
   - jeśli bieżący link == `toolLink` → sukces, zwróć zakumulowaną ścieżkę jointów,
   - jeśli bieżący link już jest w zbiorze odwiedzonych na tej gałęzi → napotkano cykl → błąd,
   - jeśli brak dalszych dzieci i cel nie osiągnięty → backtrack do rodzica, spróbuj kolejne dziecko,
   - jeśli wszystkie gałęzie wyczerpane bez trafienia w `toolLink` → **brak ścieżki** → błąd.
5. **Ponumerowanie:** jedno przejście po znalezionej, uporządkowanej liście jointów: ustaw `KinematicJoint::index` = pozycja (0-based), i dla `type` ∈ {`Revolute`, `Prismatic`, `Continuous`} ustaw `variable = JointVariable{"q" + n, n}` z rosnącym `n` (start 1); dla `Fixed` zostaw `variable = nullopt`.
6. **Wynik:** sukces → `std::expected` zawierający `KinematicChain{baseLink, toolLink, joints}`; błąd na dowolnym wcześniejszym kroku → `std::unexpected(KinematicChainError)`.

## 9. Walidacja danych wejściowych

Review słusznie kwestionuje, czy pełna walidacja spójności `RobotDescription` powinna być odpowiedzialnością tego buildera — stanowisko "builder ufa wejściu, zabezpiecza tylko własne przejście grafu" jest architektonicznie uzasadnione i domyślnie je przyjmuję. Jest jednak jeden techniczny niuans, który warto nazwać wprost, zanim to zatwierdzimy do końca:

**Cykl a "child link z wieloma rodzicami" (duplicate parent) to dwa różne zjawiska.** Zbiór `visited` ograniczony do *bieżącej gałęzi* DFS (czysto defensywny guard z sekcji 8.3, krok 4) wykrywa cykl — sytuację, w której link jest własnym przodkiem. **Nie wykrywa** natomiast linku z dwoma rodzicami (dwie różne krawędzie prowadzą do tego samego dziecka — nazywane roboczo "diamentem" tylko w dyskusji, w kodzie/testach: *duplicate parent* / *child link with multiple parents*, żeby uniknąć niejednoznacznej nazwy), bo obie gałęzie do niego prowadzące nigdy nie współistnieją na jednej ścieżce DFS — każda jest odwiedzana osobno, więc lokalny `visited` nic nie zauważa. Efekt bez dodatkowego sprawdzenia: dla `RobotDescription` z takim duplikatem `build()` zwróci "poprawnie wyglądający" łańcuch, ale jego kształt zależy od **kolejności iteracji** po `RobotDescription.joints` przy budowie mapy `parent -> children` — czyli ten sam, niezmieniony `RobotDescription` mógłby (przy innej implementacji kontenera/kolejności wstawiania) dać inny wynik. To nie jest hipotetyczne — to bezpośrednia konsekwencja "ufania wejściu" bez żadnego sprawdzenia.

**Decyzja (zatwierdzona):** jedno tanie sprawdzenie "czy ten `childLink` już ma przypisanego rodzica", wykonywane jako efekt uboczny budowy mapy `parent -> children` w kroku 1 (którą i tak budujemy jednym przejściem po jointach) — zero dodatkowego przejścia po danych, jeden `insert` do hash-seta na joint. To nie jest "pełna walidacja modelu" (nie sprawdzamy np. spójności limitów czy referencji do nieistniejących linków) — to jedno konkretne sprawdzenie chroniące jedyne założenie, na którym opiera się cały algorytm (URDF opisuje drzewo: każdy link ma co najwyżej jednego rodzica). Wykrywane niezależnie od tego, czy duplikat dotyczy linku na żądanej ścieżce, czy nie → `KinematicChainError::InvalidRobotDescription` (sekcja 10).

Poza tym jednym niuansem, walidacja ogranicza się do:
- `baseLink` niepusty i obecny w grafie,
- `toolLink` niepusty i obecny w grafie.

## 10. Możliwe błędy (pytanie 6)

Zamiast rozdrabniać błędy strukturalne na osobne kody (co sugerowałaby "pełna walidacja modelu", odrzucona w sekcji 9), przyjmuję prostszy, czteroelementowy zestaw z review — jeden ogólny kod na "wejście nie spełnia założeń, na których opiera się algorytm":

```
enum class KinematicChainError
{
    BaseLinkNotFound,
    ToolLinkNotFound,
    NoPathFound,
    InvalidRobotDescription,
};
```

| Sytuacja | Gdzie wykrywana | `KinematicChainError` |
|---|---|---|
| `baseLink` nie istnieje w robocie | Walidacja wejścia (krok 2) | `BaseLinkNotFound` |
| `toolLink` nie istnieje w robocie | Walidacja wejścia (krok 2) | `ToolLinkNotFound` |
| Brak ścieżki `baseLink → toolLink` (np. `toolLink` istnieje, ale nie jest potomkiem `baseLink`) | DFS wyczerpuje wszystkie gałęzie (krok 4) | `NoPathFound` |
| Cykl napotkany podczas przejścia (defensywny guard, sekcja 8.3/9) | DFS, powrót do odwiedzonego linka na bieżącej gałęzi | `InvalidRobotDescription` |
| Link z dwoma rodzicami (duplicate parent, sekcja 9 — zatwierdzone) | Budowa grafu (krok 1) | `InvalidRobotDescription` |
| "Wiele ścieżek" | W poprawnym drzewie strukturalnie niemożliwe — zawsze objaw duplicate parent, nie osobna kategoria | `InvalidRobotDescription` |
| Pusty `RobotDescription` (brak linków/jointów) | Naturalnie łapane przez walidację `baseLink`/`toolLink` | `BaseLinkNotFound`/`ToolLinkNotFound`, bez osobnej kategorii |
| `baseLink == toolLink` | Zatwierdzone: nie jest błędem (sekcja 14) | Sukces — pusty `KinematicChain` |

**Rozróżnienie od błędów programistycznych:** powyższe to wyłącznie błędy *domenowe* — nieprawidłowe/niespójne dane wejściowe, których wystąpienia builder musi się spodziewać i obsłużyć jako normalny (choć niepomyślny) wynik. Naruszenie własnego wewnętrznego niezmiennika implementacji (np. DFS "kończy się sukcesem", ale ostatni link na ścieżce to nie `toolLink` — czyli błąd w samym algorytmie, nie w danych) to **bug**, nie `KinematicChainError` — sygnalizowany `assert`/`std::unreachable()`, poza publicznym kontraktem API.

## 11. API klasy (pytanie 10)

Klasa pozostaje domyślnie konstruowalna, bez pól, bezstanowa — dokładnie jak `UrdfModelLoader` i pozostałe zadeklarowane już buildery. Zmienia się natomiast sygnatura zwracanej wartości względem tego, co jest dziś zadeklarowane w `KinematicChainBuilder.hpp`.

**Decyzja: `std::expected<KinematicChain, KinematicChainError>` zamiast wyjątku.** To zmiana względem pierwszej wersji tego proposalu (tam: `std::runtime_error`, w stylu `UrdfModelLoader`) — przyjmuję argument z review. Co więcej, precedens w repo przemawia **za** tą zmianą silniej, niż sam argument "to nowoczesny C++23": dolna warstwa, `mt::kinematics::load_urdf` (`robot_model_loader.hpp`), którą `UrdfModelLoader` opakowuje, **już dziś** zwraca `std::expected<LoadResult, LoadError>`. To `UrdfModelLoader` na granicy fasady konwertuje ten `expected` na wyjątek — czyli to on jest w tym repo wyjątkiem od wzorca warstwy `mt::kinematics`, nie odwrotnie. `KinematicChainBuilder::build` zwracające `std::expected<KinematicChain, KinematicChainError>` jest więc krokiem **bliżej** istniejącego stylu dolnej warstwy, a nie odejściem od niego.

Konkretne korzyści (zgodnie z review):
- brak wyjątków jako mechanizmu sterowania przepływem dla w pełni oczekiwanych, domenowych niepowodzeń,
- testy proste i precyzyjne: `EXPECT_FALSE(result.has_value())` + `EXPECT_EQ(result.error(), KinematicChainError::BaseLinkNotFound)` — bez parsowania treści komunikatu,
- `IkEquationBuilder` (fasada, jeszcze niezaimplementowana) będzie mogła propagować błąd przez `return std::unexpected(...)`, bez `try/catch`.

**Granica z błędami programistycznymi (sekcja 10):** `KinematicChainError` obejmuje wyłącznie błędy domenowe. Naruszenie wewnętrznego niezmiennika implementacji (bug w samym algorytmie DFS, nie w danych) sygnalizowane jest `assert`/`std::unreachable()`, nie przez `expected` — zgodnie z podziałem z review: *oczekiwane niepowodzenia* idą przez `expected`, *naruszenia kontraktu* zostają poza publicznym API błędów.

**Rozważona i odłożona alternatywa: `mt::DiagnosticBag`.** Repo ma już gotowy, używany dziś mechanizm na dokładnie ten rodzaj sytuacji — `mt::kinematics::load_urdf` zwraca `LoadResult{RobotModel, DiagnosticBag}`, gdzie `DiagnosticBag` niesie *nie-fatalne* ostrzeżenia (np. "pominięto fixed joint", "łańcuch ma <6 actuated jointów") równolegle z sukcesem, z gotowym kodem `DiagnosticCode::Kinematics_InvalidRobotModel`. To kusząca opcja dla przyszłego rozróżnienia "diament poza żądaną ścieżką to ostrzeżenie, diament na ścieżce to twardy błąd" — ale to dodatkowa złożoność, której ten proposal świadomie nie wprowadza teraz (sekcja 9 traktuje `InvalidRobotDescription` jako twardy błąd niezależnie od tego, czy leży na ścieżce). Odnotowuję to jako możliwy kierunek na później, nie decyzję do podjęcia w tym dokumencie.

**Konsekwencja dla nazw testów (sekcja 13):** nazwy `ThrowsWhen...`/`ThrowsOn...` z oryginalnej listy w prompt-cie odzwierciedlają wyjątki. Skoro API nie rzuca, proponuję zmienić je na `ReturnsErrorWhen...`/`Fails...`, z zachowaniem identycznego pokrycia scenariuszy — czysto nazewnicza konsekwencja tej decyzji, nie nowy zakres.

## 12. Granice odpowiedzialności względem pozostałych komponentów

- **`UrdfModelLoader`** → **`KinematicChainBuilder`**: granica to `RobotDescription`. Loader nic nie wie o "łańcuchu" ani o base/tool linku; builder nic nie wie o XML/pugixml.
- **`KinematicChainBuilder`** → **`JointTransformBuilder`**: granica to `KinematicChain`. Builder decyduje **które** jointy są istotne, w **jakiej kolejności** i **czy** dostają zmienną oraz **jaką nazwę** ta zmienna ma — ale nie tworzy `Expression`/`SymbolExpression` z tej nazwy (to zrobi `JointTransformBuilder` przez `ExpressionFactory::symbol(...)`, per joint). Builder nie dotyka `origin`/`axis` poza kopiowaniem — nie liczy żadnego obrotu.
- **`JointTransformBuilder`** (kolejny etap, niezaimplementowany): dostaje pojedynczy `KinematicJoint` i zamienia go na `SymbolicTransform` — tu dopiero dzieje się matematyka (`rpy`, potem obrót o `axis` sparametryzowany `variable`, jeśli obecna).
- **`ForwardKinematicsBuilder`**: mnoży `SymbolicTransform`-y w kolejności `KinematicChain::joints` — całkowicie zależny od tego, że `KinematicChainBuilder` dostarczył poprawną kolejność i **żadnego** brakującego (w tym fixed) jointu.
- **`IkEquationBuilder`** (fasada): woła `KinematicChainBuilder::build` w `selectChain(...)`, przechowuje wynik w `kinematicChain_`, udostępnia przez `kinematicChain()`.

**Otwarta konsekwencja (nierozstrzygana w tym dokumencie):** skoro `KinematicChainBuilder::build` zwraca `std::expected`, a `UrdfModelLoader::load` rzuca wyjątkiem, `IkEquationBuilder::selectChain(...)` będzie musiało jakoś ujednolicić te dwa style na granicy fasady — albo samo rzuci na podstawie `expected.error()` (symetrycznie do tego, jak dziś `UrdfModelLoader` konwertuje `mt::kinematics`'owy `expected` na wyjątek), albo cała warstwa `ik` docelowo przejdzie na `expected` i to `UrdfModelLoader` zostanie retrofitowany. `IkEquationBuilder` nie ma dziś żadnej implementacji poza konstruktorem, więc nic tu jeszcze nie jest ustalone — to świadomie zostawiam jako temat osobnego proposalu, nie coś do rozstrzygnięcia przy okazji tego dokumentu.

## 13. Proponowana lista testów jednostkowych (pytanie 7 z listy + rozszerzenie)

Nazwy poniżej zaktualizowane pod `std::expected` (sekcja 11) — patrz też uwaga o zachowaniu identycznego pokrycia scenariuszy z oryginalnej listy w prompt-cie.

**Wymagane z promptu (asercje przez `result.has_value()`/`result->...`/`result.error()`, nie `EXPECT_THROW`):**
- `BuildsKr640BaseToTool0Chain` — pełny happy path na realnym URDF (liniowy, bez rozgałęzień).
- `PreservesJointOrder` — kolejność jointów w wyniku odpowiada topologii, nie kolejności w `RobotDescription.joints`.
- `KeepsFixedJoints` — fixed jointy obecne w wyniku (np. `joint_a6_to_tool0` dla KR640, `link6-tool0` dla KR4 — `base_link-base` do niego nie należy, to ślepa gałąź, zob. sekcja 6).
- `AssignsSymbolsOnlyToActuatedJoints` — `variable.has_value()` tylko dla `Revolute`/`Prismatic`(/`Continuous`), `nullopt` dla `Fixed`.
- `NumbersSymbolsFromQ1` — pierwsza zmienna na ścieżce to `q1`, rosnąco.
- `BuildsKr4BaseToTool0Chain` — happy path na URDF z rozgałęzieniem przy `link_6`.
- `ReturnsBaseLinkNotFoundWhenBaseLinkDoesNotExist` (było: `ThrowsWhenBaseLinkDoesNotExist`).
- `ReturnsToolLinkNotFoundWhenToolLinkDoesNotExist` (było: `ThrowsWhenToolLinkDoesNotExist`).
- `ReturnsNoPathFoundWhenNoPathExists` (było: `ThrowsWhenNoPathExists`).

**Proponowane dodatkowe:**
- `PicksCorrectBranchAtLinkSix` (KR4) — wprost sprawdza, że `flange` (martwa gałąź) **nie** trafia do łańcucha, gdy `toolLink="tool0"`; to jest test na poprawność DFS z backtrackingiem, nie tylko na "czy w ogóle coś zwrócił".
- `ChainRecordsRequestedBaseAndToolLinkNames` — `KinematicChain.baseLink`/`toolLink` faktycznie ustawione na przekazane wartości wejściowe.
- `JointIndexMatchesPositionInChain` — `KinematicJoint::index` zgodny z pozycją w wektorze, także dla fixed jointów (regresja na pomieszanie dwóch liczników z sekcji 5.3).
- `PreservesOriginAndAxisForFixedJoints` — geometria fixed jointu (np. `link6-tool0`) skopiowana wiernie, nie "zerowana" tylko dlatego, że nie jest actuated.
- `ReturnsInvalidRobotDescriptionOnCyclicInput` — syntetyczny, ręcznie zbudowany w teście `RobotDescription` z cyklem (bez potrzeby pliku URDF), sprawdza defensywny guard z sekcji 8.3/9.
- `ReturnsInvalidRobotDescriptionOnDuplicateChildLink` — analogicznie, syntetyczny `RobotDescription` z linkiem mającym dwóch rodziców (zatwierdzone w sekcji 14).
- `ReturnsEmptyChainWhenBaseEqualsTool` — zatwierdzone w sekcji 14: `baseLink == toolLink` zwraca sukces z pustym `KinematicChain::joints` (transformacja tożsamościowa).

**Uwaga o fixture'ach:** testy błędów strukturalnych (`ThrowsOnDuplicateChildLink`, `ThrowsOnCyclicRobotDescription`) **nie wymagają plików URDF** — `RobotDescription` można zbudować ręcznie wprost w teście, bo `KinematicChainBuilder` przyjmuje ją jako zwykłą wartość wejściową. To jest bezpośrednia zaleta granicy ustalonej w sekcji 5.2 (kopie, nie referencje do żywego loadera) — testy tego etapu są w pełni odizolowane od `UrdfModelLoader`/pugixml. Happy-path testy (`BuildsKr640...`, `BuildsKr4...`) nadal korzystają z istniejących `data/urdf/kr640.urdf`/`kr4_r600.urdf` przez pełną ścieżkę `UrdfModelLoader → KinematicChainBuilder`, żeby mieć też pokrycie integracyjne na realnych danych.

## 14. Decyzje (zatwierdzone)

1. **`baseLink == toolLink`** → sukces, `KinematicChain{baseLink, toolLink, joints={}}` — poprawna semantyka matematyczna: `T(base → base) = I`. Brak jointów nie oznacza błędu.
2. **Duplicate parent / child link z wieloma rodzicami** (sekcja 9) → wykrywane jako tani efekt uboczny budowy mapy `parent -> children`, zero dodatkowego przejścia po danych; nie jest to pełna walidacja `RobotDescription`, tylko ochrona jedynego założenia, na którym opiera się algorytm. Wynik: `KinematicChainError::InvalidRobotDescription`.
3. **`Continuous` traktowany jako actuated już teraz** (razem z `Revolute`/`Prismatic`), mimo że loader go jeszcze nie dostarcza — tanie i logicznie poprawne, bez potrzeby wracania do tej logiki później.

Wszystkie trzy punkty zatwierdzone bez zmian względem rekomendacji. Proposal architektoniczny uznany za zamknięty — kolejny krok: proposal implementacyjny (pełny kod), zob. `proposal-kinematic-chain-builder-implementation.md`.

## 15. Plan implementacji (po zatwierdzeniu tego dokumentu)

1. Osobny proposal, tym razem w standardowym formacie tego repo (prompt / stan obecny / **pełny kod zmian**), obejmujący: `KinematicChainBuilder.cpp`, `KinematicChainError` (enum), aktualizację sygnatury w `KinematicChainBuilder.hpp` na `std::expected<KinematicChain, KinematicChainError>`, oraz plik testów `tests/test_kinematic_chain_builder.cpp` z listą z sekcji 13.
2. Kolejność wewnątrz implementacji: budowa grafu (+ opcjonalny check diamentu, zależnie od 14.2) → DFS z defensywnym guardem cykli → numeracja `q1..qn` → mapowanie na `KinematicChainError` → testy (happy path na KR640/KR4 najpierw, potem błędy).
3. Rejestracja nowego pliku źródłowego i testowego w `CMakeLists.txt`/`tests/CMakeLists.txt` (dziś `KinematicChainBuilder.cpp` nie istnieje i biblioteka się linkuje tylko dlatego, że nic go nie wywołuje).
