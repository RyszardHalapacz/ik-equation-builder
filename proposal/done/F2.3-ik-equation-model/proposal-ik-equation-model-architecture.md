# Proposal: model równań i targetów IK — architektura (v3)

## Prompt

> Zaprojektować model domenowy wejścia i wyjścia przyszłego `ConstraintBuilder`: `SymbolicTransform T_base_tcp(q) + target → IkEquationSystem`. Ten etap **nie buduje jeszcze constraintów** — ma zaprojektować stabilne typy. Wyłącznie dokument architektoniczny.

Etap F2.3 roadmapy Fazy 2.

## Status weryfikacji

**Ten dokument nie zawiera kodu produkcyjnego i niczego nie kompilowałem.** Stan repozytorium sprawdzony w plikach, nie w dokumentach.

---

## 1. Cel i zakres

Zaprojektować typy, które będą **wejściem i wyjściem** `ConstraintBuilder`, zanim powstanie sam builder. Powód jest procesowy: budując builder najpierw, wymyślilibyśmy ten model implicite, po jednym wywołaniu naraz.

**W zakresie:** `Equation`, `IkEquationSystem`, metadane równania, hierarchia targetów, `PositionTarget`, `PoseTarget`, reprezentacja i walidacja orientacji, model błędów walidacji, własność wyrażeń, mutowalność, deterministyczna kolejność, szkic API buildera, relacja z fasadą.

**Poza zakresem** — §17.

---

## 2. Rzeczywisty stan repozytorium

`HEAD = 224a9b7` („Implement IkEquationBuilder facade: Phase 1 complete"). **Etap TCP jest zaimplementowany, ale niezacommitowany** — w drzewie roboczym, `ctest` daje **242/242**, `TcpTransformTest.*` 21/21.

Rozbieżność do odnotowania: `git status` pokazuje dorobek TCP jako niezacommitowany (12 plików zmodyfikowanych, 10 nieśledzonych, w tym trzy dokumenty roadmapowe/proposalowe). `STATUS.md` w drzewie **odpowiada** kodowi (242/242), ale ostatni commit go nie zawiera. Ten proposal opiera się na **kodzie z drzewa roboczego**, bo to on jest prawdziwy.

### 2.1 Co realnie istnieje

| Warstwa | Zawartość |
|---|---|
| `model/` | `Vector3.hpp`, `FixedRigidTransform.hpp`, `JointVariable.hpp`, `KinematicChain.hpp`, `KinematicChainError.hpp`, `RobotDescription.hpp`, `UrdfJoint.hpp` — **same nagłówki, żadnego `.cpp`, wyłącznie agregaty** |
| `symbolic/` | `Expression`, `ExpressionFactory`, `SymbolicMatrix`, `SymbolicTransform`, `ExpressionEvaluator` |
| `builders/` | `KinematicChainBuilder`, `JointTransformBuilder`, `ForwardKinematicsBuilder`, `RigidTransformConstruction`, `detail/PrincipalRotation` |
| fasada | `IkEquationBuilder` z siedmioma kodami błędu, pięcioma polami `optional`, dziewięcioma metodami |

**Nie istnieje katalog `validation/`.** To ma znaczenie dla §16.

### 2.2 Właściwości `Expression`, na których opiera się §11

Zweryfikowane w `Expression.hpp`: uchwyt nad `shared_ptr<const ExpressionNode>`, węzły **niemutowalne**, kopiowanie kopiuje jeden wskaźnik i nigdy drzewa, `sameNode` publiczne i O(1). To nie jest założenie — to zapisany kontrakt tej warstwy.

---

## 3. Model równania

### 3.1 Warianty

| Wariant | Zapis | Ocena |
|---|---|---|
| **A** `{Expression lhs; Expression rhs;}` | `lhs = rhs` | **przyjęty** |
| B `{Expression residual;}` | `residual = 0` | odrzucony |
| C osobny `EqualityEquation` | j.w. co A | odrzucony jako przedwczesne rozwarstwienie |

### 3.2 Dlaczego nie postać rezydualna

Prompt podaje argument o rozroście drzewa. Jest prawdziwy, ale **słaby** — `subtract(lhs, rhs)` to jeden węzeł na równanie, przy drzewach rzędu 500 węzłów to szum.

Argument rozstrzygający jest inny: **postać rezydualna niszczy informację, której nie da się odzyskać.** W `lhs = rhs` widać, która strona pochodzi z robota, a która z zadania. Po odjęciu ta granica przestaje istnieć, a jej odtworzenie wymagałoby dekompozycji algebraicznej — czyli dokładnie tego, czego projekt nie ma i czego ten etap nie buduje.

Solver będzie tej granicy potrzebował: „przenieś stałą na prawo, izoluj `q`" zakłada, że wiadomo, co jest stałą zadania. Zaczynanie od `residual = 0` znaczyłoby wyrzucić tę wiedzę i kazać ją odgadywać z kształtu drzewa.

**Normalizacja `lhs − rhs = 0` należy do simplifiera** (F3.3 przewiduje „przenoszenie stron równania"), nie do modelu i nie do buildera. Model niczego nie odejmuje niejawnie.

### 3.3 Prawa strona zawsze `Expression`

Odrzucam wariant z `double rhs`. Dwa kształty tej samej rzeczy zmusiłyby każdego konsumenta do rozgałęzienia na „która to wersja", a zysk — jeden węzeł `Constant` mniej — jest żaden. `Expression` zostawia też otwarte drzwi na targety symboliczne (np. sparametryzowane położeniem na ścieżce), których roadmapa nie wyklucza.

Konsekwencja: budowa równania wymaga `ExpressionFactory`, żeby zamienić `double` z targetu na `Constant`. To jest w porządku — builder i tak ją ma.

---

## 4. Tożsamość i znaczenie równania

`expression = expression` nie mówi, czego równanie dotyczy. Bez metadanych solver musiałby zgadywać z kształtu drzewa.

### 4.1 Dwa pojęcia, nie jedno — blocker z review

v1 używała **jednego** `ConstraintKind{Position, Orientation}` w dwóch rolach: jako rodzaj zadania w systemie i jako rodzaj pojedynczego równania. To jest niespójne i widać to natychmiast na `PoseTarget`: **system nie ma wtedy poprawnej wartości enuma**, bo zadanie jest „poza", a nie „pozycja albo orientacja".

To dwa różne zbiory pojęć i dostają dwa typy:

```cpp
enum class IkTaskKind   { Position, Pose };          // o co poprosił wołający
enum class EquationKind { Position, Orientation };   // co ogranicza jedno równanie
```

`IkTaskKind` rośnie razem z zestawem targetów (`ToolDirection`, `PositionAndDirection`…), `EquationKind` razem z rodzajami równań. Te dwa zbiory nie rosną tak samo — kolejny powód, żeby ich nie sklejać.

### 4.2 Źródło równania jako wariant, nie `{kind, optional<cell>}` — blocker z review

v1 proponowała `{ConstraintKind kind; std::optional<MatrixCell> cell;}`. Review słusznie zauważa, że taka struktura dopuszcza stany **semantycznie fałszywe**, a jednocześnie dokument deklaruje, że metadane są wiążące dla solvera. Nie można mieć obu naraz.

Możliwe do zbudowania i bezsensowne: równanie pozycji wskazujące komórkę `(2,2)`, równanie orientacji wskazujące `(0,3)`, komórka `(999, 1000)`.

Przyjęty kształt:

```cpp
enum class CartesianComponent { X, Y, Z };

struct PositionEquationSource
{
    CartesianComponent component;
};

// Domknięty, bo ma inwariant: row i column w 0..2.
class OrientationEquationSource
{
public:
    [[nodiscard]] static std::optional<OrientationEquationSource>
    create(std::size_t row, std::size_t column) noexcept;

    [[nodiscard]] std::size_t row() const noexcept;
    [[nodiscard]] std::size_t column() const noexcept;

private:
    OrientationEquationSource(std::uint8_t row, std::uint8_t column) noexcept;

    std::uint8_t row_;
    std::uint8_t column_;
};

using EquationSource = std::variant<PositionEquationSource, OrientationEquationSource>;
```

Trzy zyski, wszystkie realne:

- `PositionEquationSource` **nie może** wskazać komórki rotacji — nie ma czym;
- znika `optional`, którego znaczenie zależało od enuma obok;
- dodanie constraintu kierunkowego (F2.6) wymusi obsługę nowej alternatywy **w miejscu kompilacji**, o ile konsumenci używają jawnych zestawów przeciążeń (§6.2), a nie lambd generycznych.

`EquationKind` z §4.1 staje się przez to **wyprowadzalny** z wariantu i **nie jest polem** `Equation`.

Zostaje jednak jako pojęcie — używa go diagnostyka i walidacja `taskKind` z §5.3.2 — więc `Equation.hpp` dostarcza jawnie:

```cpp
[[nodiscard]] EquationKind kindOf(const EquationSource& source) noexcept;
```

Alternatywa (rezygnacja z `EquationKind` i poleganie na `std::holds_alternative`) wystarczyłaby dziś, ale kod walidujący `taskKind` i tak potrzebowałby tego pojęcia, tylko wyrażonego przez sprawdzanie alternatyw w każdym miejscu użycia. Jeden helper jest tańszy niż powtarzane `holds_alternative` — i rośnie razem z wariantem w **jednym** miejscu.

### 4.3 Zakres `0..2` — typ musi go **wymusić**, nie tylko oferować fabrykę

v2 proponowała agregat z publicznymi polami **plus** statyczną fabrykę. Review słusznie zauważa, że to nic nie wymusza: `OrientationEquationSource invalid{7, 9};` pozostaje legalne, a fabryka staje się sugestią.

To jest **ten sam błąd, który v2 naprawiała w trzech innych miejscach** — kontrakt zadeklarowany mocniej, niż gwarantuje typ. Popełniłem go ponownie w tej samej rewizji, w której go opisywałem.

Stąd kształt z §4.2: prywatny konstruktor, `create` zwracające `std::optional`, dostęp przez `row()`/`column()`. **Jedyną drogą konstrukcji jest walidowana fabryka.**

Rozważana alternatywa — `enum class RotationElement { R00 … R22 }` — jest równie poprawna i jeszcze ciaśniejsza, ale `ConstraintBuilder` będzie indeksował macierz, więc `row()`/`column()` oszczędza konwersji przy każdym użyciu. Wybieram klasę.

### 4.4 Gdzie mieszkają metadane i czy są semantyczne

**Per równanie**, jako pole `Equation` — system zawiera mieszankę (przy pełnej pozie są równania pozycji i orientacji), więc metadane na poziomie systemu by nie wystarczyły.

Jednocześnie system niesie `IkTaskKind`. Po rozdzieleniu z §4.1 nie jest to już redundancja ani kolizja: system mówi **o co poproszono** (`Pose`), równanie mówi **co ogranicza** (`Position`, składowa `X`).

**Metadane są semantyczne, nie informacyjne.** Solver ma prawo opierać na nich logikę — dekompozycja pozycja/orientacja to pierwszy krok każdego analitycznego solvera 6R. To jest cel istnienia tego pola, i dlatego §4.2 wyklucza stany, w których byłyby fałszywe.

---

## 5. `IkEquationSystem`

### 5.1 Musi być samowystarczalnym wejściem solvera — blocker z review

v1 nie przechowywała listy niewiadomych i zakładała, że „solver weźmie ją z `KinematicChain`, który fasada posiada". **To był błąd projektowy**, i review wskazuje dokładnie dlaczego: wiąże solver z fasadą albo zmusza do przekazywania dwóch niezależnych obiektów, podczas gdy docelowy potok brzmi

```
IkEquationSystem → EquationSimplifier → EquationSolver
```

Wyprowadzanie niewiadomych z drzew nie ratuje sytuacji, i to z czterech niezależnych powodów:

- **nie istnieje publiczny collector symboli** — każdy plik testowy pisze własny `containsSymbol`;
- kolejność przejścia po DAG-u **nie jest** kolejnością jointów;
- sortowanie po nazwach da `q1, q10, q2`;
- po pojawieniu się symbolicznych parametrów targetu nie będzie jak odróżnić niewiadomej od parametru.

### 5.2 Przyjęty kształt

```cpp
class IkEquationSystem
{
public:
    [[nodiscard]] static std::expected<IkEquationSystem, IkEquationSystemError>
    create(IkTaskKind taskKind,
           std::vector<JointVariable> unknowns,
           std::vector<Equation> equations);

    [[nodiscard]] IkTaskKind taskKind() const noexcept;
    [[nodiscard]] std::span<const JointVariable> unknowns() const noexcept;
    [[nodiscard]] std::span<const Equation> equations() const noexcept;

private:
    IkEquationSystem(IkTaskKind, std::vector<JointVariable>, std::vector<Equation>);

    IkTaskKind taskKind_;
    std::vector<JointVariable> unknowns_;
    std::vector<Equation> equations_;
};
```

`JointVariable` istnieje już w `model/JointVariable.hpp` jako `{std::string name; std::size_t index;}` — niesie i nazwę symbolu, i pozycję w łańcuchu, czyli dokładnie to, czego solver potrzebuje. Nic nowego nie trzeba definiować.

**Kolejność `unknowns`: rosnący `JointVariable::index`.** To jest kontrakt, nie szczegół.

### 5.3 Inwarianty egzekwowane, nie deklarowane — blocker z review

v1 twierdziła, że „pusty system jest niepoprawny", i jednocześnie proponowała agregat, w którym `IkEquationSystem system{};` jest legalny. **Nie można deklarować silnego inwariantu i zostawiać go całkowicie niewymuszonym w typie, który ma być stabilną granicą solvera.**

Mój argument, że domknięcie wymaga `ConstraintBuilder` jako `friend`, **był po prostu nieprawdziwy** — statyczna fabryka zwracająca `std::expected` nie potrzebuje żadnego frienda i działa dziś, bez buildera. Przy okazji zwiększa to testowalną powierzchnię tego etapu, zamiast ją odkładać.

`create` egzekwuje **osiem** inwariantów. Pięć było w v2, trzy dokłada review:

| Inwariant | Powód |
|---|---|
| `equations` niepuste | system bez równań nic nie ogranicza |
| `unknowns` niepuste | zadanie IK bez niewiadomych nie jest zadaniem IK |
| indeksy w `unknowns` unikalne | dwie niewiadome o tym samym indeksie to zepsuty łańcuch |
| nazwy w `unknowns` niepuste | `ExpressionFactory` i tak zabrania pustych nazw symboli |
| `unknowns` posortowane rosnąco po `index` | §5.2 — kontrakt kolejności |
| **nazwy w `unknowns` unikalne** | **nowe** — §5.3.1 |
| **`taskKind` zgodny z rodzajami źródeł równań** | **nowe** — §5.3.2 |
| **kolejność równań zgodna z §12** | **nowe** — §5.3.3 |

#### 5.3.1 Unikalne nazwy, nie tylko indeksy

v2 wymuszała wyłącznie unikalność `index`. Legalne pozostawało:

```cpp
{ JointVariable{"q1", 1}, JointVariable{"q1", 2} }
```

Dla warstwy symbolicznej `"q1"` to **jeden i ten sam symbol** — obie „niewiadome" wskazywałyby na ten sam węzeł. Solver dostałby dwie rzekomo różne zmienne dla jednego stopnia swobody, i to bez żadnego sygnału.

To dokładnie ten sam scenariusz, który przy walidacji numerycznej FK wymusił `throw` na zduplikowanej nazwie w `makeSymbolValues`. Tam był wykrywany; tutaj v2 by go przepuściła.

#### 5.3.2 Zgodność `taskKind` ze źródłami równań

v2 dopuszczała `IkTaskKind::Position` z `OrientationEquationSource` w środku — mimo deklaracji, że metadane są semantyczne.

v2 była przy tym **wewnętrznie sprzeczna**: §5.3.2 wymagała dla `Pose` „co najmniej jednego równania pozycyjnego", a §5.3.3 mówiła o kolejności `X, Y, Z`. Przy pierwszym sformułowaniu legalny byłby system pełnej pozy zawierający **wyłącznie X** i równania orientacji — co nie jest pełną pozą.

Kontrakt jednoznaczny:

```
Position : dokładnie trzy równania pozycyjne — X, Y, Z
Pose     : dokładnie trzy równania pozycyjne — X, Y, Z
           oraz co najmniej jedno równanie orientacyjne
```

Pozycja jest **kompletna w obu zadaniach**: nie ma sensownego zadania IK ograniczającego dwie współrzędne z trzech, a gdyby kiedyś powstało, będzie miało własny `IkTaskKind`, a nie okrojony `Pose`.

Liczba równań **orientacji** pozostaje otwarta — to decyzja F2.5, a roadmapa wprost zabrania zakładania dziewięciu z góry. Sprawdzamy obecność, nie liczność.

Rozdział wad, zgodnie z §5.3.4:

| Sytuacja | Kod |
|---|---|
| brakująca składowa pozycji, duplikat składowej, równanie orientacji w zadaniu `Position` | `TaskEquationMismatch` |
| komplet obecny, ale w złej kolejności | `UnorderedEquations` |

To rozdziela **wadliwą zawartość** od **wadliwej kolejności** — dwie różne wady, dwie różne poprawki po stronie wołającego.

#### 5.3.3 Kolejność równań egzekwowana, nie deklarowana

v2 nazywała kolejność „częścią kontraktu", a `create` przyjmowało dowolny wektor — więc system pozycyjny w kolejności `Z, X, Y` był legalny. Znowu: kontrakt mocniejszy niż typ.

Skoro `EquationSource` niesie już wszystkie potrzebne metadane, `create` może to sprawdzić:

```
Position : dokładnie X, Y, Z, w tej kolejności
Pose     : najpierw równania pozycyjne w kolejności X, Y, Z,
           potem orientacyjne w porządku wierszowym
```

**Selekcja** elementów orientacji pozostaje dowolna (F2.5), ale **wybrane** muszą być uporządkowane rosnąco po `(row, column)`.

Dzięki temu `RejectsUnorderedEquations` i `RejectsTaskEquationKindMismatch` stają się testami **tego** etapu, zamiast czekać na builder.

### 5.3.4 `IkEquationSystemError`

v2 użyła tego typu w sygnaturze `create` i **nigdzie go nie zdefiniowała** — zostawiając model błędów najważniejszego typu tego etapu do wymyślenia przez proposal implementacyjny. Uzupełniam:

```cpp
enum class IkEquationSystemErrorCode
{
    NoEquations,
    NoUnknowns,
    EmptyUnknownName,
    DuplicateUnknownName,
    DuplicateUnknownIndex,
    UnorderedUnknowns,
    TaskEquationMismatch,
    UnorderedEquations
};

struct IkEquationSystemError
{
    IkEquationSystemErrorCode code{};
    std::string message;   // nazywa konkretną niewiadomą lub pozycję równania
};
```

**Osiem kodów, po jednym na inwariant z §5.3** — świadomie bez sklejania. Kusi połączenie `DuplicateUnknownIndex` z `UnorderedUnknowns` („obie dotyczą indeksów"), ale to różne wady: pierwsza oznacza zepsuty łańcuch, druga wyłącznie złą kolejność podania. Wołający naprawia je inaczej.

Kształt `{code, message}` jest identyczny z `TargetValidationError` i `IkEquationBuilderError` — trzeci taki typ w projekcie, ta sama konwencja.

#### Deterministyczna kolejność sprawdzeń

Osiem kodów nie ma znaczenia, dopóki nie wiadomo, który wygrywa przy wielu naruszeniach naraz. `unknowns = {{"q1",1}, {"q1",1}}` narusza **trzy** inwarianty jednocześnie: nazwę, indeks i porządek.

```
1. NoEquations
2. NoUnknowns
3. EmptyUnknownName
4. DuplicateUnknownName
5. DuplicateUnknownIndex
6. UnorderedUnknowns
7. TaskEquationMismatch
8. UnorderedEquations
```

Jedna kolejność jest **wymuszona logicznie**, nie arbitralna: `DuplicateUnknownIndex` musi iść **przed** `UnorderedUnknowns`, bo dwa identyczne indeksy same z siebie łamią ścisły porządek rosnący. Bez tego duplikat zgłaszałby się jako zła kolejność — diagnoza prawdziwa, ale bezużyteczna.

Reszta idzie od wad struktury (brak danych) przez wady zbioru niewiadomych po wady zbioru równań — czyli od tego, co wołający naprawia najwcześniej.

### 5.4 Asymetria wobec `Equation` jest zamierzona

`Equation` zostaje agregatem, `IkEquationSystem` jest domknięty. To nie jest niekonsekwencja: `Equation` **nie ma inwariantu ponad to, co gwarantują jej pola** — `Expression` nigdy nie jest puste, a `EquationSource` po §4.2 nie ma stanów fałszywych. `IkEquationSystem` ma pięć inwariantów, których agregat nie utrzyma.

### 5.5 Pozostałe rozstrzygnięcia

| Pytanie | Rozstrzygnięcie |
|---|---|
| kolejność równań częścią kontraktu? | **tak** — §12 |
| równania mogą się powtarzać? | typ nie zabrania; duplikat nie jest błędem strukturalnym, tylko marnotrawstwem |
| system przechowuje target? | **nie** — §5.6 |
| system przechowuje nazwy chainu/frame'ów? | **nie** — builder ich nie widzi (§14) |
| niemutowalny po utworzeniu? | **tak** — brak setterów, akcesory zwracają `span<const>` |
| tani w kopiowaniu? | **tak** — kopia to kopie uchwytów `Expression`; §11 |

### 5.6 Dlaczego system nie przechowuje targetu

Wartości targetu **już są** w systemie — jako stałe po prawej stronie równań. Druga kopia to drugie źródło prawdy, które może się z pierwszym rozjechać.

Zwracam uwagę na różnicę wobec §5.1: **niewiadome nie są odzyskiwalne** z równań w sposób deterministyczny, a wartości targetu są. To ta różnica decyduje, co trafia do systemu, a co nie — nie symetria ani wygoda.

---

## 6. Hierarchia targetów

| Wariant | Ocena |
|---|---|
| **`std::variant` z zamkniętym zbiorem** | **przyjęty** |
| jedna struktura z `optional` | odrzucony |
| hierarchia polimorficzna | odrzucony |

### 6.1 Dlaczego wariant

Struktura z trzema `optional` dopuszcza `{nullopt, nullopt, nullopt}` — **target, który niczego nie ogranicza** — oraz kombinacje, których nikt nie zdefiniował. To ta sama zasada, dla której fasada trzyma stan w `optional` zamiast w zwykłych polach: stany nielegalne mają być niewyrażalne.

Polimorfizm dokłada alokację i wskaźnik za zbiór, który jest **zamknięty z definicji** — targety wymyślamy my, nie użytkownik biblioteki.

### 6.2 Zakres: tylko dwa targety teraz

```cpp
using IkTarget = std::variant<PositionTarget, PoseTarget>;
```

Zgodnie z rekomendacją zakresową: model rozszerzalny, ale bez projektowania całego języka constraintów kierunkowych przed czasem.

**Uwaga implementacyjna do przeniesienia dalej:** konsumenci mają używać **jawnych zestawów przeciążeń**, nie lambd generycznych. `std::visit` z lambdą generyczną po cichu przyjmie nową alternatywę i skompiluje się, robiąc coś przypadkowego; zestaw przeciążeń **nie skompiluje się**, dopóki nowy target nie zostanie obsłużony. To jest właściwe zachowanie przy rozszerzaniu wariantu.

---

## 7. `PositionTarget`

```cpp
struct PositionTarget
{
    Vector3 position;   // metry, w układzie bazowym transformacji podanej builderowi
};
```

Nazwa `position`, nie `translation` (to składowa transformacji, nie punkt) i nie `point` (za ogólne).

### 7.1 Target nie zna TCP ani końca łańcucha

`ConstraintBuilder` dostaje **konkretny `SymbolicTransform`**, więc target odnosi się do układu, który ta transformacja opisuje. Model targetu nie przechowuje nazwy frame'u i nie wie, czy transformacja kończy się w TCP, czy w końcu łańcucha.

Zysk: target jest niezależny od fasady i od tego, czy TCP w ogóle jest ustawione.

**Koszt, który trzeba nazwać:** nic nie broni sparowania targetu pomyślanego dla TCP z transformacją do końca łańcucha. Typ tego nie wykryje, kompilator też nie. To ta sama klasa luki co kierunek `FixedRigidTransform` (`T_parent_child` niewymuszalny) — §18.

---

## 8. Reprezentacja orientacji targetu

| Wariant | Ocena |
|---|---|
| **`RotationMatrix3`** (3×3, przechowywana) | **przyjęty** |
| kwaternion | przyjmowany na wejściu, nieprzechowywany |
| RPY | przyjmowane na wejściu, nieprzechowywane |

### 8.1 Dlaczego macierz — i dlaczego inaczej niż przy TCP

Przy `FixedRigidTransform` wybraliśmy RPY, **bo jest reprezentacją totalną**: każda szóstka skończonych liczb jest poprawna, więc walidacja nie potrzebuje progu. Tutaj wybieram inaczej i chcę powiedzieć wprost dlaczego, bo to wygląda na niekonsekwencję.

Trzy powody:

1. **Równania i tak będą o komórkach macierzy.** `R_tcp(q)` to symboliczny blok 3×3; porównanie z targetem jest porównaniem komórek. Trzymanie RPY znaczyłoby konwersję do macierzy przy każdym budowaniu równań — czyli przechowywanie w reprezentacji, której konsument nie używa.
2. **Źródło danych.** Target orientacji przychodzi z wizji, CAD-u albo z innego układu — czyli jako macierz lub kwaternion. Offset narzędzia przychodzi z karty katalogowej — czyli jako kąty.
3. **Jednoznaczność.** RPY nie jest unikalne (trójki równoważne przez gimbal). Dla stałego offsetu narzędzia to bez znaczenia; dla targetu, który można porównywać czy deduplikować, ma znaczenie.

Cena jest realna i przyjęta świadomie: macierz **wymaga** walidacji ortogonalności z tolerancją, czyli dokładnie tego, czego przy TCP uniknęliśmy. Tutaj jest to jednak **konieczne**, nie przypadkowe: nieortogonalny target dałby układ równań bez rozwiązania, i to po cichu.

### 8.2 Konwersje wypadają z zakresu tego etapu — blocker z review

v1 proponowała `RotationMatrix3 rotationFromQuaternion(...)` i `rotationFromRollPitchYaw(...)`. **Obie sygnatury są złe**, i to z dwóch niezależnych powodów.

**Zły typ wyniku.** Zwykłe `RotationMatrix3` nie ma jak zgłosić odrzucenia, a dokument w tym samym miejscu deklaruje „odrzucamy błędne wejście, nie normalizujemy". Nie wiadomo, co miałoby się stać dla kwaternionu zerowego, niejednostkowego, z `NaN`, ani dla nieskończonego RPY.

**Druga produkcyjna implementacja konwencji RPY.** To poważniejsze. Dwa dokumenty temu, przy TCP, argumentowałem za usunięciem duplikacji `buildRpyRotation` — właśnie dlatego, że `STATUS.md` odnotowuje, iż **konwencji RPY nic zewnętrznego nie weryfikuje**, więc dwie implementacje to dwa miejsca na jej rozjazd. `rotationFromRollPitchYaw` wprowadzałaby tę duplikację z powrotem, tyle że w arytmetyce numerycznej zamiast symbolicznej. Przeoczyłem to, projektując model targetów w oderwaniu od tamtej decyzji.

**Decyzja: żadnych konwersji w tym etapie.** Publiczne wejście orientacji to wyłącznie:

```cpp
RotationMatrix3                                        // przekazywane wprost
std::expected<void, TargetValidationError> validate(const RotationMatrix3&);
```

Adaptery kwaternionowy i RPY powstaną **jako osobny, świadomy etap wygody API** — i wtedy trzeba będzie rozstrzygnąć tolerancję normy kwaternionu, odrzucanie kontra normalizację, kolejność składowych `w,x,y,z` oraz, dla RPY, czy powstaje wspólna warstwa geometrii numerycznej, czy adapter w ogóle nie wchodzi.

Wewnątrz systemu orientacja i tak istnieje **wyłącznie** jako macierz — dwie reprezentacje to dwa źródła prawdy.

### 8.3 Dokładny kształt `RotationMatrix3`

v2 wybrała macierz, nie pokazując jej API ani layoutu. Skoro jest to **publiczny typ wejściowy**, zostawienie tego implementacji znaczyłoby, że o kolejności indeksów rozstrzyga przypadek.

```cpp
struct RotationMatrix3
{
    std::array<std::array<double, 3>, 3> values{};
};
```

Kontrakt: **`values[row][column]`, row-major, oba indeksy `0..2`.** Walidator (§9) i przyszły `ConstraintBuilder` używają dokładnie tego indeksowania, spójnie z `OrientationEquationSource::row()`/`column()`.

**Bez metod i bez `operator()`** — agregat zostaje agregatem. `SymbolicMatrix` ma `operator()`, bo ma bounds-checking i semantykę macierzową; tutaj nie ma czego sprawdzać poza zakresem tablicy, którym zajmuje się `std::array`.

---

## 9. Walidacja orientacji

```
wszystkie dziewięć wartości skończonych
|(RᵀR − I)ᵢⱼ| ≤ tol     dla każdego elementu
|det(R) − 1|   ≤ tol
```

### 9.1 Tolerancja — najpierw kontrakt źródła danych, potem liczba

Prompt ostrzega, żeby nie dziedziczyć `1e-12` z walidacji FK, i to jest słuszne: `1e-12` było **zmierzonym** ograniczeniem błędu **naszego własnego obliczenia**, a tutaj walidujemy **dane wejściowe**. Inny kontrakt.

**Ale v1 popełniła ten sam błąd jeden poziom niżej** — podała tabelkę rzędów wielkości i wybrała `1e-9` „na oko", nie mierząc. Review to zmierzyło: dla losowych poprawnych rotacji zaokrąglonych do dziewięciu miejsc po przecinku

```
max |RᵀR − I|    ≈ 1.66e-9
max |det(R) − 1| ≈ 1.81e-9
```

czyli **`1e-9` odrzuciłoby poprawną rotację zapisaną z dziewięcioma cyframi po przecinku** — dokładnie ten przypadek, dla którego próg miał być dobrany. Liczba była zła, a metoda jej doboru gorsza.

#### Kontrakt źródła — rozstrzygnięcie

| Wariant | Zakres | Konsekwencja |
|---|---|---|
| **A** | `double` oraz macierze z tekstu o rozsądnej precyzji | próg rzędu `1e-8` |
| B | dodatkowo dane, które przeszły przez `float` | próg rzędu `1e-6` albo wejście kwaternionowe |

**Przyjęty wariant A.** Powód: próg `1e-6` przyjmowałby macierz odchyloną od ortogonalności o `1e-6`, a taka macierz wchodzi wprost do równań i czyni układ **subtelnie niespełnialnym** — bez żadnego sygnału. Deklarowanie wariantu B znaczyłoby milczącą zgodę na niespójny target.

Dane pochodzące z `float` nie są przez to porzucone: ich właściwą drogą jest **jawne** doprowadzenie do ortogonalności po stronie wołającego (przyszłe `orthonormalize`), a nie rozluźnienie progu tak, żeby przeszły niezauważone.

#### Próg: `1e-8`, wiążąco

Review powtórzyło pomiar na **500 000** losowych macierzy rotacji zaokrąglonych do dziewięciu miejsc po przecinku:

```
max |RᵀR − I|    ≈ 1.68e-9
max |det(R) − 1| ≈ 1.94e-9
```

`1e-8` daje ponad pięciokrotny zapas nad zmierzonym maksimum i pozostaje ~100× ciaśniejsze niż poziom błędu z `float`, więc dane z wariantu B nadal odpadają **głośno**.

**Próg jest ustalony, nie kandydacki.** Pomiar ma jednak zostać **powtórzony i zaraportowany przy wdrożeniu** — tą samą dyscypliną, którą przyjęliśmy dla tolerancji FK. Rozjazd z liczbami powyżej byłby ustaleniem do review, nie powodem do cichej korekty progu.

**Tolerancja nie jest współdzielona** z `kAbsoluteTolerance` z `TransformComparison`: sklejenie ich znaczyłoby, że zmiana progu porównania wyników po cichu zmienia próg przyjmowania danych.

### 9.3 Deterministyczny priorytet błędów — uwaga z review

Macierz `diag(1, 1, 0.5)` jest jednocześnie nieortogonalna **i** ma zły wyznacznik. v1 przewidywała dwa osobne testy, nie ustalając, który błąd wygrywa.

**Ustalona kolejność sprawdzeń:**

```
1. wszystkie dziewięć wartości skończonych   → NonFiniteOrientation
2. ortogonalność |(RᵀR − I)ᵢⱼ| ≤ tol         → NonOrthogonalOrientation
3. wyznacznik |det(R) − 1| ≤ tol             → ImproperRotation
```

Konsekwencja, którą v1 przeoczyła: **po przejściu kroku 2 macierz jest ortogonalna, więc `det ∈ {+1, −1}`.** Wyznacznik różny od `+1` oznacza wtedy w praktyce **odbicie**, a nie skalowanie — skalowanie odpadło już na ortogonalności.

Dlatego:

- kod nazywa się `ImproperRotation`, nie „invalid determinant" — po kroku 2 to jest dokładnie to, co opisuje;
- test wyznacznika musi używać **odbicia** `diag(1, 1, −1)` (ortogonalne, `det = −1`), a nie skalowania `diag(1, 1, 0.5)`, które trafi w `NonOrthogonalOrientation`. Test z v1 był źle dobrany do własnego kodu błędu.

### 9.2 Odrzucamy, nie normalizujemy

Automatyczna reortonormalizacja **zmieniłaby target, o który poprosił użytkownik**, i ukryła jego błąd. To ta sama decyzja, co brak normalizacji kwaternionu w referencji numerycznej: sprawdzamy normę, nie naprawiamy jej.

Konsekwencja dla API: użytkownik z lekko nieortogonalną macierzą musi ją naprawić u siebie. Jeśli okaże się to uciążliwe, właściwą odpowiedzią jest **jawna** funkcja `orthonormalize`, a nie milcząca poprawka w walidatorze.

---

## 10. `PoseTarget`

```cpp
struct PoseTarget
{
    Vector3 position;             // metry
    RotationMatrix3 orientation;
};
```

**Semantyka: `T_base_target`** — położenie i orientacja układu docelowego **wyrażone w układzie bazowym** podanej transformacji. Nie „poza" bez wskazania kierunku.

Nie zagnieżdżam `PositionTarget` w `PoseTarget` — daje to niezręczne `pose.position.position`. Wspólna jest walidacja, nie struktura.

---

## 11. Własność `Expression` i mutowalność

### 11.1 `Equation` trzyma `Expression` przez wartość

```cpp
struct Equation
{
    Expression lhs;
    Expression rhs;
    EquationSource source;
};
```

Wariant z `std::reference_wrapper<const Expression>` **odrzucam stanowczo**, i nie z powodów stylistycznych: `ConstraintBuilder` będzie budował wyrażenia lokalnie, więc system referencyjny wskazywałby na obiekty, które giną wraz z powrotem z `build`. To dokładnie ten błąd, który już raz w tym projekcie wystąpił — cache evaluatora na surowym wskaźniku (§blocker, naprawiony kluczem `Expression`).

Przez wartość jest przy tym **tanie**: `Expression` to uchwyt nad `shared_ptr<const ExpressionNode>`, kopiowanie kopiuje wskaźnik, węzły są niemutowalne i współdzielone. Kopia systemu to kopia `2 × n` wskaźników, nie drzew.

### 11.2 Mutowalność — agregaty tam, gdzie nie ma inwariantu

v2 twierdziła, że **wszystkie** typy tego etapu są agregatami, wymieniając wśród nich `IkEquationSystem` i nieistniejący już `MatrixCell`. To była sprzeczność z §5.2, gdzie ten sam dokument projektuje system jako typ domknięty.

Poprawny podział:

| Typ | Kształt | Powód |
|---|---|---|
| `Equation` | **agregat** | brak inwariantu ponad to, co gwarantują pola |
| `PositionTarget`, `PoseTarget`, `RotationMatrix3` | **agregaty** | walidacja jest osobną operacją (§9), nie warunkiem konstrukcji |
| `PositionEquationSource` | **agregat** | `CartesianComponent` jest enumem, każda wartość poprawna |
| `OrientationEquationSource` | **domknięty** | zakres `0..2` (§4.3) |
| `IkEquationSystem` | **domknięty** | osiem inwariantów (§5.3) |

Zasada, nie wyjątek: **typ jest domknięty wtedy i tylko wtedy, gdy ma inwariant, którego agregat nie utrzyma.** Żadnych getterów dla prostych danych — projekt nie robi tego nigdzie (`KinematicJoint`, `FixedRigidTransform`, `JointOrigin` są agregatami).

Uwaga do targetów: pozostają agregatami **mimo** że mają warunki poprawności. To jest świadome — walidacja odbywa się na granicy operacji, która target konsumuje (§9, §13.2), a nie przy konstrukcji. Zamknięcie ich wymusiłoby fabrykę zwracającą `expected` przy każdym literalnym targecie w teście czy w kodzie wołającego, co jest ceną bez zysku: niezwalidowany target nigdy nie dociera dalej niż do `validate`.

---

## 12. Deterministyczna kolejność równań

**Kolejność jest częścią kontraktu**, nie szczegółem implementacji. Zależą od niej: stabilność testów, diagnostyka, solver, generowany kod i powtarzalność wyników.

```
PositionOnly:  (0,3), (1,3), (2,3)                    → X, Y, Z

FullPose:      najpierw pozycja X, Y, Z
               następnie orientacja w porządku wierszowym:
               (0,0) (0,1) (0,2) (1,0) (1,1) (1,2) (2,0) (2,1) (2,2)
```

**Ten etap ustala porządek, nie selekcję.** To, ile i które elementy rotacji faktycznie stają się równaniami, jest decyzją F2.5 — roadmapa mówi wprost, żeby nie zakładać automatycznie dziewięciu niezależnych równań. Tutaj ustalamy tylko, że **jeśli** element trafia do systemu, to w porządku wierszowym.

Żadnych `map`/`unordered_map` na drodze do `equations` — kolejność ma wynikać z pętli po komórkach.

---

## 13. Model błędów

```cpp
enum class TargetValidationErrorCode
{
    NonFinitePosition,
    NonFiniteOrientation,
    NonOrthogonalOrientation,
    ImproperRotation          // ortogonalna, ale det = -1: odbicie
};

struct TargetValidationError
{
    TargetValidationErrorCode code{};
    std::string message;      // nazywa składową, która zawiodła
};
```

Kolejność sprawdzeń i nazwa `ImproperRotation` — §9.3. Skalowanie **nie** trafia tutaj: odpada wcześniej, na ortogonalności.

### 13.2 Publiczne API walidatora — cztery przeciążenia

v2 pokazywała wyłącznie `validate(const RotationMatrix3&)`, a plan testów wymagał walidacji `PositionTarget` i `PoseTarget`. Uzupełniam pełny zestaw:

```cpp
[[nodiscard]] std::expected<void, TargetValidationError> validate(const PositionTarget&);
[[nodiscard]] std::expected<void, TargetValidationError> validate(const RotationMatrix3&);
[[nodiscard]] std::expected<void, TargetValidationError> validate(const PoseTarget&);
[[nodiscard]] std::expected<void, TargetValidationError> validate(const IkTarget&);
```

`validate(const RotationMatrix3&)` jest wystawione osobno, mimo że `PoseTarget` je woła — bo orientacja jest walidowalna sama z siebie i testy progu z §16 celują właśnie w nią, bez pakowania jej za każdym razem w pozę.

`validate(const IkTarget&)` jest **dispatcherem** przez `std::visit` z **jawnym zestawem przeciążeń**, nie z lambdą generyczną — §6.2. Dzięki temu dodanie trzeciej alternatywy do wariantu nie skompiluje się, dopóki ktoś nie zdecyduje, jak ją walidować.

### 13.1 Podział odpowiedzialności

```
typy domenowe            → nie zależą od fasady
walidator targetu        → własny typ błędu (TargetValidationError)
ConstraintBuilder        → własny błąd, opakowujący TargetValidationError bez spłaszczania
fasada                   → później opakuje błąd buildera, też bez spłaszczania
```

**Nie dodaję kodów do `IkEquationBuilderErrorCode`** — fasada nie ma jeszcze żadnej operacji przyjmującej target, więc byłyby to kody nieosiągalne. Dodanie ich nastąpi w etapie integracji z fasadą, i wtedy zachowają typowany błąd wewnątrz, tak jak `chainError` zachowuje `KinematicChainError`.

---

## 14. Przewidywane API `ConstraintBuilder`

Szkic, **nie do implementacji w tym etapie**:

```cpp
class ConstraintBuilder
{
public:
    explicit ConstraintBuilder(ExpressionFactory factory = {});

    [[nodiscard]] std::expected<IkEquationSystem, ConstraintBuildError>
    build(const SymbolicTransform& transform,
          std::span<const JointVariable> unknowns,
          const IkTarget& target) const;

private:
    ExpressionFactory factory_;
};
```

Trzeci parametr wynika z §5.1: system ma być samowystarczalny, a z samego `SymbolicTransform` nie da się odróżnić zmiennej jointu od innego symbolu ani odtworzyć ich kolejności. `std::span<const JointVariable>` to **minimalny** kontekst, który to umożliwia — builder nadal nie widzi `KinematicChain`, URDF-a, TCP ani fasady. Fasada wyciągnie zmienne z aktualnego łańcucha i poda je dalej.

| Wariant | Ocena |
|---|---|
| **klasa z fabryką jako polem, jedno `build` z wariantem** | **przyjęty** |
| wolne funkcje per typ targetu | odrzucony — rozjeżdża się z resztą builderów |
| osobne buildery per rodzaj constraintu | odrzucony — przedwczesne |

Uzasadnienie kształtu: `JointTransformBuilder` i `ForwardKinematicsBuilder` **oba** są klasami z `ExpressionFactory` jako polem i `explicit` konstruktorem z argumentem domyślnym. Trzeci builder o innym kształcie byłby niespójnością bez powodu.

Jedno `build` z wariantem, nie przeciążenia: `IkTarget` ma konstruktor konwertujący z alternatyw, więc wołający z konkretnym targetem i tak pisze `builder.build(fk, PositionTarget{p})`. Przeciążenia dałyby to samo za cenę rozmnożenia punktów wejścia.

**Builder nie zna `IkEquationBuilder`, URDF-a ani TCP.** Dostaje transformację i target, zwraca system. To fasada wie, że przekazywana transformacja to akurat `T_base_tcp`.

---

## 15. Relacja z fasadą i graf zależności

```
        RobotDescription
               │
         KinematicChain
          │          │
   ForwardKinematics  TCP
          │          │
          └────┬─────┘
               │
      TcpForwardKinematics
               │
               │  + target (argument, nie węzeł)
               ▼
        IkEquationSystem
```

### 15.1 Target jako argument, nie pole fasady

| Wariant | Ocena |
|---|---|
| **`buildConstraints(target)`, target jako argument** | **przyjęty** |
| trwałe pole `target_` jako szósty węzeł stanu | odrzucony |

Rozróżnienie jest rzeczowe, nie techniczne: **TCP jest własnością konfiguracji stanowiska** — ustawia się je raz i używa dla wielu zadań. **Target jest zapytaniem** — zmienia się przy każdym ruchu. Zrobienie z niego węzła stanu sugerowałoby, że należy do konfiguracji, i dokładałoby wiersz do tabeli unieważniania oraz obowiązek pamiętania o `setTarget` przed `buildConstraints`.

Przy targetcie jako argumencie nie ma stanu nieaktualnego: `equationSystem()` jest wynikiem ostatniego `buildConstraints`, a kolejne wywołanie go nadpisuje.

`IkEquationSystem` jest **potomkiem `tcpForwardKinematics`** w grafie, więc unieważnia go wszystko, co unieważnia tamten węzeł: zmiana robota, łańcucha, przebudowa FK, zmiana lub wyczyszczenie TCP. Wynika to z reguły zapisanej przy TCP i **nie wymaga nowej reguły** — o to chodziło w zamianie łańcucha liniowego na graf.

### 15.2 Na której transformacji operuje fasada — do rozstrzygnięcia później

`buildConstraints` musiałaby wybrać między `forwardKinematics()` a `tcpForwardKinematics()`. Rekomendacja do etapu integracji: **wymagać `tcpForwardKinematics`**, czyli błąd, gdy TCP nie ustawiono. Wariant „użyj TCP, jeśli jest, w przeciwnym razie końca łańcucha" to ten sam wzorzec cichej różnej odpowiedzi, który projekt odrzucił przy `buildForwardKinematics`.

Nie rozstrzygam tego wiążąco tutaj — to decyzja etapu fasady, nie modelu.

---

## 16. Plan testów

Trzeba powiedzieć wprost rzecz, której lista z promptu nie oddaje: **na tym etapie nie da się przetestować połowy proponowanych inwariantów**, bo nie ma czego uruchomić. `PreservesDeterministicEquationOrder`, `PreservesEquationRoleMetadata` i `RejectsEmptyEquationSystem` wymagają `ConstraintBuilder`, który powstaje w F2.4.

Domknięcie `IkEquationSystem` (§5.3) i wariantowy `EquationSource` (§4.2) **zwiększają** tę powierzchnię wobec v1: inwarianty, które v1 chciała odłożyć do F2.4, są teraz egzekwowane przez typ i dają się przetestować od razu.

### 16.1 Testowalne teraz — 30 testów

**Własność wyrażeń (3)** — `tests/test_ik_equation_model.cpp`

`StoresEquationSidesByValue` (równanie przeżywa zniszczenie źródłowych uchwytów), `CopiedEquationSharesImmutableExpressionNodes` (`sameNode` po kopii), `CopiesIkEquationSystemWithoutDeepCopyingExpressions`.

**Inwarianty `IkEquationSystem` (9)** — j.w.

`CreatesValidSystem`, `RejectsSystemWithoutEquations`, `RejectsSystemWithoutUnknowns`, `RejectsDuplicateUnknownIndices`, **`RejectsDuplicateUnknownNames`**, `RejectsUnknownWithEmptyName`, `RejectsUnorderedUnknowns`, **`RejectsTaskEquationKindMismatch`**, **`RejectsUnorderedEquations`** — po jednym na każdy z ośmiu inwariantów z §5.3 plus przypadek pozytywny. Każdy sprawdza **kod błędu**, nie samo niepowodzenie: osiem kodów z §5.3.4 istnieje po to, żeby je rozróżniać.

**Źródło równania (2)** — j.w.

`CreatesOrientationSourceForValidCell` (`{0,0}`, `{2,2}`) i `RejectsOutOfRangeOrientationSource` (`{3,0}`, `{0,3}`, `{7,9}`) — przez `create` z §4.3, które jest jedyną drogą konstrukcji.

Odpowiednika „równanie pozycji nie może wskazać komórki rotacji" **nie da się napisać jako testu** — po §4.2 nie ma czym wyrazić tego stanu. To zamierzone i jest to argument za wariantem, a nie luka w pokryciu: inwariant wymuszony przez typ nie potrzebuje testu, bo nie da się go złamać.

**Walidacja pozycji (4)** — `tests/test_target_validation.cpp`

`AcceptsFinitePositionTarget`, `RejectsNonFinitePositionTargetX/Y/Z` — każda składowa osobno, `NaN` i `±Inf`, bo walidator ma gałąź na składową.

**Walidacja orientacji (7)**

| Test | Dane |
|---|---|
| `AcceptsIdentityRotation` | `I` |
| `AcceptsValidRotationMatrix` | `Rz(0.7)` |
| `RejectsNonFiniteRotationMatrix` | `NaN`, `±Inf` w komórce |
| `RejectsNonOrthogonalRotationMatrix` | skalowanie `diag(1,1,0.5)` — **odpada na ortogonalności**, §9.3 |
| `RejectsImproperRotation` | odbicie `diag(1,1,−1)` — ortogonalne, `det = −1` |
| `AcceptsRotationJustWithinTolerance` | odchylenie `≈ 0.9e-8` |
| `RejectsRotationJustBeyondTolerance` | odchylenie `≈ 1.1e-8` |

#### Dlaczego testy progu muszą stać tuż przy granicy

v2 proponowała `2e-9` i `1e-6`. **To nie pinuje `1e-8`**: implementacja mogłaby użyć dowolnej wartości między tymi liczbami — `5e-9`, `1e-7` — i oba testy nadal by przeszły. Próg pozostałby liczbą w komentarzu.

Testy stoją więc po obu stronach granicy, `≈0.9e-8` i `≈1.1e-8`, uzyskane przez **kontrolowane skalowanie jednej osi**. Obie wartości celują w metrykę **ortogonalności**, nie wyznacznika — bo po §9.3 ortogonalność jest sprawdzana pierwsza, więc to ona decyduje o wyniku.

**`PoseTarget` (3)**

`AcceptsValidPoseTarget`, `RejectsPoseTargetWithNonFinitePosition`, `RejectsPoseTargetWithInvalidOrientation`.

**Dispatcher `IkTarget` (2)** — §13.2

`ValidatesPositionTargetThroughIkTarget`, `ValidatesPoseTargetThroughIkTarget` — każda alternatywa wariantu przechodzi przez `validate(const IkTarget&)` i daje ten sam wynik co wywołanie bezpośrednie.

**Razem: 3 + 9 + 2 + 4 + 7 + 3 + 2 = 30. Oczekiwany stan: 242 + 30 = 272.**

### 16.2 Odłożone do F2.4, z podaniem powodu

`PreservesDeterministicEquationOrder` i `PreservesEquationSourceMetadata` — wymagają buildera, który produkuje równania. `StoresPoseAsTBaseTarget` — semantyka kierunku jest sprawdzalna dopiero, gdy coś z niej korzysta.

Wypisuję je jako **zobowiązanie**, nie jako pominięcie: proposal F2.4 ma je zawierać. `RejectsEmptyEquationSystem` **przestaje** być odłożony — po §5.3 da się go napisać teraz.

---

## 17. Plan zmian w plikach

Nazwy z promptu skonfrontowane z rzeczywistą strukturą (§2.1).

**Dodane:**

| Plik | Zawartość |
|---|---|
| `src/ik_equations/model/Equation.hpp` | `EquationKind`, `CartesianComponent`, `PositionEquationSource`, `OrientationEquationSource`, `EquationSource`, `Equation` |
| `src/ik_equations/model/IkEquationSystem.hpp/.cpp` | `IkTaskKind`, `IkEquationSystemError`, `IkEquationSystem` — **`.cpp`, bo `create` egzekwuje inwarianty** |
| `src/ik_equations/model/IkTarget.hpp` | `RotationMatrix3`, `PositionTarget`, `PoseTarget`, `IkTarget` — **bez konwersji**, §8.2 |
| `src/ik_equations/model/TargetValidation.hpp/.cpp` | `TargetValidationErrorCode`, `TargetValidationError`, `validate(...)` |
| `tests/test_ik_equation_model.cpp` | **14 testów** — 3 własność `Expression` + 9 inwariantów systemu + 2 `EquationSource` |
| `tests/test_target_validation.cpp` | **16 testów** — 4 pozycja + 7 orientacja + 3 `PoseTarget` + 2 dispatcher |

**Zmienione:** `CMakeLists.txt` (**dwie** linie — `TargetValidation.cpp`, `IkEquationSystem.cpp`), `tests/CMakeLists.txt` (dwie linie), `STATUS.md`.

**Bez zmian — jawnie:** fasada, wszystkie buildery, cała warstwa symboliczna, `tests/support/`.

### 17.1 Dlaczego bez katalogu `validation/`

Prompt sugeruje `src/ik_equations/validation/TargetValidator.*`. Sprawdziłem: takiego katalogu nie ma, a `model/` jest dziś **wyłącznie nagłówkowy**.

Nowy katalog najwyższego poziomu dla dwóch funkcji jest cięższy niż problem. Walidacja typu modelowego naturalnie sąsiaduje z tym typem, więc idzie do `model/TargetValidation.*`.

Konsekwencja, którą odnotowuję: **`model/` przestaje być wyłącznie nagłówkowy** — to pierwszy `.cpp` w tej warstwie. Alternatywą jest katalog z jednym plikiem albo walidacja w nagłówku. Jeśli review woli utrzymać `model/` bez `.cpp`, drugim najlepszym wyborem jest `builders/TargetValidation.*` — walidacja jest bliżej buildera, który ją wywoła, niż osobnego pionu.

### 17.2 Nazwa `IkTarget.hpp` a grupowanie

`UrdfJoint.hpp` trzyma dziś cztery typy (`JointType`, `JointOrigin`, `JointLimits`, `UrdfJoint`), więc grupowanie kilku typów w jednym nagłówku jest zgodne ze stylem projektu. `RotationMatrix3` mogłaby dostać własny plik, ale istnieje wyłącznie po to, żeby opisać target — trzymam ją obok.

---

## 18. Non-goals

Jawnie **nie** projektujemy: implementacji `ConstraintBuilder`, `EquationSimplifier`, `IkPatternDetector`, `EquationSolver`, `CodeGenerator`, automatycznego izolowania `q`, eliminacji zmiennych, upraszczania trygonometrycznego, solverów numerycznych, YAML, konfiguracji MotionBridge, wyboru najlepszego rozwiązania IK, limitów jointów w solverze, osobliwości, klasyfikacji Shoulder/Elbow/Wrist.

**Nie generujemy żadnych równań z `SymbolicTransform`.**

Precyzyjniej, bo v2 twierdziła, że etap „nie dotyka warstwy symbolicznej ani razu" — a `Equation` przechowuje `Expression lhs` i `Expression rhs`, więc to nieprawda: **etap nie modyfikuje warstwy symbolicznej i nie czyta `SymbolicTransform`, ale model `Equation` zależy od `Expression`.** Ta zależność jest zamierzona i jest powodem, dla którego §11.1 rozstrzyga własność przez wartość.

---

## 19. Ryzyka

| Ryzyko | Ocena | Reakcja |
|---|---|---|
| target sparowany z niewłaściwą transformacją (TCP vs koniec łańcucha) | **średnie** | typ tego nie wykryje (§7.1); to ta sama luka co kierunek `FixedRigidTransform`; do zapisania w known gaps |
| pomiar wdrożeniowy odbiegnie od pomiaru z review | niskie | §9.1 — `1.68e-9`/`1.94e-9` na 500 000 macierzy daje 5× zapasu do `1e-8`; rozjazd byłby ustaleniem do review, nie powodem do cichej korekty |
| macierz rotacji jako target wymaga tolerancji, której RPY by nie wymagało | **przyjęte świadomie** | §8.1 — równania i tak są o komórkach macierzy |
| dane z `float` odpadną na progu | **świadome** | §9.1 wariant A; właściwą drogą jest jawne `orthonormalize` po stronie wołającego, nie rozluźnienie progu |
| domknięcie `IkEquationSystem` utrudni przyszłe rozszerzenia | niskie | `create` jest jedynym punktem wejścia, więc nowe pole to zmiana w jednym miejscu |
| adaptery kwaternion/RPY będą potrzebne wcześniej, niż zakładam | niskie | §8.2 — osobny, świadomy etap; wejście macierzowe działa bez nich |

---

## 20. Otwarte decyzje wymagające review — brak

**Brak.** Trzy pozycje otwarte w v2 zostały zamknięte w review:

| Pozycja | Rozstrzygnięcie |
|---|---|
| §9.1 — kontrakt danych i próg | **wariant A, próg `1e-8` wiążąco.** Review powtórzyło pomiar na **500 000** losowych rotacji zaokrąglonych do dziewięciu miejsc: `max\|RᵀR − I\| ≈ 1.68e-9`, `max\|det − 1\| ≈ 1.94e-9`. Ponad pięciokrotny zapas. Próg przestaje być kandydatem; pomiar i tak ma zostać powtórzony i zaraportowany przy wdrożeniu |
| §4.3 — fabryka czy agregat | **typ domknięty**, nie sama fabryka — §4.3 |
| §17.1 — gdzie walidacja | **`model/TargetValidation.*`**; `.cpp` w `model/` i tak powstaje przez `IkEquationSystem.cpp` |

Rozstrzygnięte i **niewymagające** decyzji: `lhs = rhs` bez niejawnego odejmowania, prawa strona zawsze `Expression`, rozdzielenie `IkTaskKind` od `EquationKind`, `EquationSource` jako wariant bez stanów fałszywych, `IkEquationSystem` jako typ domknięty z `create` egzekwującym **osiem** inwariantów w ustalonej kolejności, uporządkowana lista niewiadomych w systemie, brak kopii targetu w systemie, `std::variant` dla targetów ograniczony do `PositionTarget` i `PoseTarget`, `RotationMatrix3` jako jedyna przechowywana reprezentacja orientacji, **brak konwersji kwaternion/RPY w tym etapie**, odrzucanie zamiast normalizacji, kolejność sprawdzeń i nazwa `ImproperRotation`, `PoseTarget` = `T_base_target`, `Equation` przez wartość, porządek wierszowy równań, osobny `TargetValidationError` bez dotykania `IkEquationBuilderErrorCode`, builder jako klasa z fabryką i `span<const JointVariable>`, target jako argument a nie pole fasady.

---

## 21. Rekomendacja końcowa

Zatwierdzić w kształcie: **`Equation{lhs, rhs, source}` jako agregat, bez niejawnej normalizacji; `EquationSource` jako `variant<PositionEquationSource, OrientationEquationSource>`, z `OrientationEquationSource` domkniętym i tworzonym wyłącznie przez `create`; `IkTaskKind` oddzielone od `EquationKind`; `IkEquationSystem` jako typ domknięty, tworzony wyłącznie przez `create` egzekwujące **osiem** inwariantów — w tym unikalność nazw niewiadomych, zgodność `taskKind` ze źródłami i kolejność równań — z ośmiokodowym `IkEquationSystemError`; `IkTarget` jako `std::variant<PositionTarget, PoseTarget>`; orientacja wyłącznie jako `RotationMatrix3`, bez adapterów kwaternionowego i RPY; walidacja odrzucająca, nie normalizująca, w kolejności skończoność → ortogonalność → `ImproperRotation`, z **wiążącym** progiem `1e-8` i czterema przeciążeniami `validate`; `Expression` trzymane przez wartość; typ domknięty wtedy i tylko wtedy, gdy ma inwariant, którego agregat nie utrzyma; `ConstraintBuilder` przyjmujący `span<const JointVariable>` obok transformacji i targetu.**

Etap jest projektowy, nie obliczeniowy — nie powstaje ani jedno równanie. Jego wartość polega na tym, że `ConstraintBuilder` z F2.4 dostanie **zamknięty zbiór wejść i jeden kształt wyjścia**, zamiast definiować je po drodze.

Trzy decyzje niosą najwięcej: **zachowanie granicy `lhs`/`rhs`**, bo po odjęciu nie da się jej odzyskać; **metadane jako wariant**, bo enum z opcjonalną komórką pozwalał zapisać znaczenie, które jest fałszywe, przy jednoczesnej deklaracji, że solver ma na nim polegać; oraz **samowystarczalność systemu**, bo bez listy niewiadomych `IkEquationSystem → EquationSimplifier → EquationSolver` nie jest potokiem, tylko trzema obiektami, które trzeba nosić razem.

---

## 22. Co zmieniła rewizja v3

| # review | Zarzut | Werdykt | Gdzie |
|---|---|---|---|
| 1 | `OrientationEquationSource` z publicznymi polami — fabrykę da się ominąć | **przyjęty — blocker, błąd v2** | §4.2, §4.3 |
| 2.1 | brak inwariantu unikalności **nazw** niewiadomych | **przyjęty — blocker** | §5.3.1 |
| 2.2 | `taskKind` niezgodny ze źródłami równań przechodzi | **przyjęty — blocker** | §5.3.2 |
| 2.3 | kolejność równań deklarowana, nieegzekwowana | **przyjęty — blocker** | §5.3.3 |
| 3 | `IkEquationSystemError` użyty i niezdefiniowany | **przyjęty — blocker** | §5.3.4 |
| 4 | §11.2 przeczy §5.2 i wymienia usunięty `MatrixCell` | **przyjęty** | §11.2 |
| 5 | `APPROVE-READY` przy trzech otwartych decyzjach | **przyjęty** | §20 |
| 6 | testy progu za daleko od granicy, nie pinują `1e-8` | **przyjęty** | §16.1 |
| 7 | brak przeciążeń `validate` dla targetów | **przyjęty** | §13.2 |

Pięć blockerów, i cztery z nich to **ten sam błąd, który v2 opisywała jako swój wzorzec**: kontrakt zadeklarowany mocniej, niż wymusza typ.

- `OrientationEquationSource` — inwariant `0..2` w agregacie z publicznymi polami;
- unikalność niewiadomych — wymuszona po indeksie, nie po nazwie, choć symbolem jest nazwa;
- `taskKind` — deklarowany jako semantyczny, dopuszczający `Position` z równaniem orientacji;
- kolejność równań — nazwana „częścią kontraktu", przyjmowana jako dowolny wektor.

**Rozpoznałem ten wzorzec w §22 rewizji v2 i popełniłem go cztery razy w tej samej rewizji.** Nazwanie błędu najwyraźniej nie wystarcza, żeby go nie powtórzyć; wystarcza dopiero mechaniczna reguła, więc zapisuję ją w §11.2 jako zasadę projektu: **typ jest domknięty wtedy i tylko wtedy, gdy ma inwariant, którego agregat nie utrzyma** — i każde zdanie „X musi spełniać Y" wymaga wskazania, co Y egzekwuje.

Zarzut 3 jest innego rodzaju: użyłem typu w sygnaturze i nie zdefiniowałem go, zostawiając model błędów najważniejszego typu etapu do wymyślenia w proposalu implementacyjnym — czyli dokładnie tam, gdzie nie powinien powstawać.

Zarzut 6 jest wart odnotowania, bo dotyczy **testu, nie kodu**: `2e-9` i `1e-6` przepuściłyby dowolny próg między nimi. Test, który nie odróżnia poprawnej implementacji od kilku niepoprawnych, nie pinuje niczego — ta sama uwaga, która wcześniej w tym projekcie dotyczyła `structurallyEqual` zamiast `sameNode` i dwóch przemiennych translacji.

---

## 23. Co zmieniła rewizja v2

| # review | Zarzut | Werdykt | Gdzie |
|---|---|---|---|
| 1 | jeden `ConstraintKind` w dwóch rolach | **przyjęty — blocker, błąd v1** | §4.1 |
| 2 | `EquationSource` dopuszcza stany fałszywe | **przyjęty — blocker, błąd v1** | §4.2, §4.3 |
| 3 | system nie jest samowystarczalnym wejściem solvera | **przyjęty — blocker, błąd projektowy v1** | §5.1, §5.2, §14 |
| 4 | inwariant „niepusty" deklarowany, nie egzekwowany; argument o friendzie nieprawdziwy | **przyjęty — blocker** | §5.3 |
| 5 | konwersje mają zły typ wyniku i wprowadzają drugą implementację RPY | **przyjęty — blocker** | §8.2 |
| 6 | tolerancja `1e-9` niezmierzona i odrzucałaby poprawne dane | **przyjęty — zmierzone w review** | §9.1 |
| 7 | brak priorytetu błędów ortogonalność/wyznacznik | **przyjęty** | §9.3, §13 |

Pięć z siedmiu to realne błędy v1. Trzy z nich mają wspólny kształt i wart jest nazwania, bo wraca w tym projekcie:

**Deklarowałem kontrakt mocniejszy, niż wymuszał typ.** Metadane „semantyczne, na których solver może polegać" — w strukturze pozwalającej zapisać, że równanie pozycji dotyczy komórki `(2,2)`. Inwariant „pusty system jest niepoprawny" — w agregacie, w którym `{}` jest legalne. Próg „uzasadniony charakterem danych" — dobrany na oko, i o rząd wielkości za ciasny wobec przypadku, dla którego był dobierany.

Zarzut 5 jest osobnego rodzaju i gorszy: **zaproponowałem drugą produkcyjną implementację konwencji RPY dwa dokumenty po tym, jak argumentowałem za usunięciem pierwszej duplikacji.** Projektowałem model targetów w oderwaniu od decyzji podjętej przy TCP. To nie jest pomyłka w rozumowaniu, tylko brak sprawdzenia, co już zostało ustalone.

Zarzut 6 wart jest odnotowania z drugiej strony: review **zmierzyło**, zamiast oszacować, i wyszło, że `1e-9` odrzuca poprawną rotację zapisaną z dziewięcioma miejscami po przecinku (`1.66e-9`, `1.81e-9`). To jest ta sama dyscyplina, którą sam stosowałem przy tolerancji FK, i której tu nie zastosowałem.

---

```
APPROVE-READY
```
