# Proposal: warstwa symboliczna — architektura

## 1. Prompt

> Przeanalizuj dokładnie aktualny stan repozytorium KinemaForge i przygotuj proposal architektoniczny dla warstwy symbolicznej. Nie implementuj jeszcze kodu źródłowego. [...]
>
> **Cel:** zaprojektować minimalną, niemutowalną warstwę symboliczną potrzebną do późniejszej implementacji `JointTransformBuilder`, `ForwardKinematicsBuilder` i symbolicznego FK. Warstwa nie ma jeszcze rozwiązywać równań IK ani wykonywać zaawansowanego upraszczania.
>
> **Zatwierdzona decyzja:** `Expression` ma być lekkim typem wartościowym przechowującym `std::shared_ptr<const ExpressionNode>`. Drzewo ma być niemutowalne. Kopiowanie `Expression` kopiuje wyłącznie `shared_ptr`, nie całe drzewo. Nie proponuj przechowywania całego rekurencyjnego `std::variant` bezpośrednio przez wartość w `Expression`.
>
> **Zakres typów:** `Constant`, `Symbol`, `Add`, `Subtract`, `Multiply`, `Divide`, `Negate`, `Sin`, `Cos`. Bez `Atan2`/`Acos`/`Asin`/`Sqrt`/`Power`/`Derivative`/solvera/pattern detectora/code generatora/pełnego simplifiera.

*(pełna treść promptu — 10 zagadnień decyzyjnych, wymagania testowe, lista non-goals i 17-punktowy format — w wiadomości użytkownika w tej konwersacji)*

## 2. Zweryfikowany stan obecny

Wszystkie poniższe fakty pochodzą z odczytu plików źródłowych oraz **empirycznej weryfikacji kompilatorem/linkerem**, nie z `STATUS.md` ani README.

### 2.1 `symbolic/Expression.hpp`

```cpp
enum class ExpressionType { Constant, Symbol, Add, Subtract, Multiply, Divide, Sin, Cos, Negate };

class Expression;
using ExpressionPtr = std::shared_ptr<const Expression>;   // <-- wskaźnik na Expression, nie na węzeł

struct ConstantExpression { double value{}; };
struct SymbolExpression   { std::string name; };
struct BinaryExpression   { ExpressionPtr left; ExpressionPtr right; };
struct UnaryExpression    { ExpressionPtr operand; };

struct AddExpression : BinaryExpression {};     // puste typy-tagi przez dziedziczenie
struct SubtractExpression : BinaryExpression {};
struct MultiplyExpression : BinaryExpression {};
struct DivideExpression : BinaryExpression {};
struct SinExpression : UnaryExpression {};
struct CosExpression : UnaryExpression {};
struct NegateExpression : UnaryExpression {};

using ExpressionNode = std::variant<ConstantExpression, SymbolExpression, AddExpression,
    SubtractExpression, MultiplyExpression, DivideExpression, SinExpression, CosExpression, NegateExpression>;

class Expression
{
public:
    // Defaults to the constant 0 — lets SymbolicMatrix default-construct
    // its cells without needing a placeholder "empty" expression state.
    Expression() : node_(ConstantExpression{0.0}) {}   // JEDYNA definicja w całym pliku
    explicit Expression(ExpressionNode node);          // zadeklarowany, NIEZDEFINIOWANY
    ExpressionType type() const;                       // zadeklarowany, NIEZDEFINIOWANY
    const ExpressionNode& node() const;                // zadeklarowany, NIEZDEFINIOWANY
private:
    ExpressionNode node_;                              // wariant PRZEZ WARTOŚĆ
};
```

### 2.2 `symbolic/ExpressionFactory.hpp`

Klasa bezstanowa, metody `const`, wszystkie zwracają `ExpressionPtr` (= `shared_ptr<const Expression>`): `constant`, `symbol`, `add`, `subtract`, `multiply`, `divide`, `sin`, `cos`, `negate`. **Żadna nie ma definicji** — brak pliku `.cpp` w `symbolic/`.

### 2.3 `symbolic/SymbolicMatrix.hpp`

```cpp
template <std::size_t Rows, std::size_t Columns>
class SymbolicMatrix
{
public:
    Expression& at(std::size_t row, std::size_t column);              // NIEZDEFINIOWANY
    const Expression& at(std::size_t row, std::size_t column) const;  // NIEZDEFINIOWANY
private:
    std::array<Expression, Rows * Columns> values_;
};
```

Szablon z metodami zadeklarowanymi w ciele klasy, ale bez definicji **gdziekolwiek** — dla szablonu oznacza to, że nie da się ich zinstancjonować.

### 2.4 `symbolic/SymbolicTransform.hpp`

```cpp
using SymbolicRotation  = SymbolicMatrix<3, 3>;
using SymbolicVector3   = SymbolicMatrix<3, 1>;
using SymbolicTransform = SymbolicMatrix<4, 4>;
```

`SymbolicRotation` i `SymbolicVector3` są dziś **nieużywane** przez żaden plik.

### 2.5 Konsumenci warstwy

| Plik | Zależność | Stan |
|---|---|---|
| `builders/JointTransformBuilder.hpp` | `SymbolicTransform build(const KinematicJoint&) const` | tylko deklaracja, brak `.cpp` |
| `builders/ForwardKinematicsBuilder.hpp` | `SymbolicTransform build(const KinematicChain&, const JointTransformBuilder&) const` | tylko deklaracja, brak `.cpp` |
| `IkEquationBuilder.hpp` | pole składowe `SymbolicTransform forwardKinematics_;` | pole istnieje; `IkEquationBuilder() = default;` |

To pole składowe jest **twardym ograniczeniem** dla decyzji o inwariantach (§9): `SymbolicTransform` musi pozostać domyślnie konstruowalny, inaczej `IkEquationBuilder() = default` przestanie się kompilować.

`ForwardKinematicsBuilder::build` przyjmuje `const JointTransformBuilder&` — to ustalona w repo konwencja przekazywania bezstanowych współpracowników jawnie, przez referencję. Ma to wpływ na §10 (mnożenie macierzy) i §8 (kształt fabryki).

### 2.6 Testy i CMake

- `tests/`: `test_kinematics.cpp`, `test_urdf_model_loader.cpp`, `test_kinematic_chain_builder.cpp`. **Zero testów warstwy symbolicznej.**
- `CMakeLists.txt`, biblioteka `kinemaforge_ik`: `Kinematics.cpp`, `robot_model.cpp`, `robot_model_loader.cpp`, `IkEquationBuilder.cpp`, `UrdfModelLoader.cpp`, `builders/KinematicChainBuilder.cpp`. **Żadnego pliku z `symbolic/`.**

### 2.7 Weryfikacja empiryczna

Skompilowałem sondę wobec obecnych nagłówków (`g++ -std=c++23 -Isrc`):

| Konstrukcja | Wynik |
|---|---|
| `Expression e;` | ✅ kompiluje i linkuje |
| `SymbolicTransform m;` | ✅ kompiluje i linkuje |
| `m.at(0, 0)` | ❌ `undefined reference to SymbolicMatrix<4ull,4ull>::at(...)` |
| `ExpressionFactory{}.constant(1.0)` | ❌ `undefined reference to ExpressionFactory::constant(double) const` |
| `Expression{ConstantExpression{2.0}}`, `e.type()` | ❌ `undefined reference` (obie) |

**Wniosek:** projekt linkuje się dziś wyłącznie dlatego, że jedyny użyty element warstwy to inline'owy konstruktor domyślny. Cała reszta to fasada bez implementacji.

Zmierzone rozmiary (ta sama sonda):

| Typ | Rozmiar |
|---|---|
| `ExpressionNode` (wariant) | 40 B |
| `Expression` **obecnie** | 40 B |
| `SymbolicMatrix<4,4>` **obecnie** | 640 B |
| `shared_ptr<const T>` (proponowany `Expression`) | 16 B |
| `SymbolicMatrix<4,4>` **proponowany** | 256 B |

## 3. Problemy w aktualnych nagłówkach

1. **Odwrócona indyrekcja.** `ExpressionPtr = shared_ptr<const Expression>`, a `Expression` trzyma wariant przez wartość. Drzewo wygląda więc tak: `Expression{wariant} → shared_ptr<const Expression> → Expression{wariant} → ...` — czyli na każdym poziomie jest opakowanie `Expression` **wokół** alokacji, zamiast wskaźnika **do** węzła. To dokładnie ten układ, który prompt odrzuca.

   Warto oddać sprawiedliwość obecnemu kodowi: ten kształt **nie jest przypadkowy** — jest jednym z dwóch sposobów przecięcia rekurencji `Expression ↔ ExpressionNode` (dzieci przez `shared_ptr` zamiast przez wartość). Działa i pozwala trzymać `ExpressionNode` jako alias wariantu. Jest jednak droższy: `Expression` ma 40 B zamiast 16 B, kopia symbolu alokuje, a każdy poziom drzewa niesie zbędne opakowanie. Proponowane rozwiązanie przecina tę samą rekurencję w drugim miejscu — kosztem uczynienia `ExpressionNode` strukturą (§6.1a).
2. **Drogie kopiowanie.** Kopia `Expression` kopiuje 40-bajtowy wariant. Dla `SymbolExpression` oznacza to kopię `std::string` — czyli **alokację** przy każdym kopiowaniu symbolu. Kopia nie jest głęboka (dzieci to `shared_ptr`), ale też nie jest darmowa.
3. **Nic nie ma definicji.** Poza konstruktorem domyślnym — patrz §2.7. Nie jest to „częściowa implementacja", tylko wyłącznie deklaracje.
4. **`SymbolicMatrix::at()` nie da się zinstancjonować.** Metody szablonu klasy muszą mieć definicję widoczną w miejscu instancjacji; tu nie ma jej nigdzie.
5. **Brak inicjalizacji o jasnej semantyce.** `std::array<Expression, N>` polega na konstruktorze domyślnym `Expression`, który daje stałą 0. Działa, ale każda komórka trzyma **własną** kopię wariantu — 16 niezależnych `ConstantExpression{0.0}` w macierzy 4×4.
6. **Brak jakiegokolwiek API do budowania transformacji.** Nie ma macierzy zerowej, jednostkowej ani mnożenia. `JointTransformBuilder` nie ma dziś czym się posłużyć.
7. **Puste typy-tagi przez dziedziczenie** (`struct AddExpression : BinaryExpression {}`) wymuszają podwójne nawiasy przy inicjalizacji agregatowej (`AddExpression{{l, r}}`) i sugerują relację „is-a", która nie ma tu znaczenia — `AddExpression` nie jest podstawialne za `BinaryExpression` w żadnym sensownym kontekście.
8. **Brak porównywania.** Ani strukturalnego, ani predykatów `isZero`/`isOne` — a bez nich nie da się zaimplementować żadnej normalizacji.

## 4. Cele

1. Warstwa **niemutowalna**, wartościowa, tania w kopiowaniu (`Expression` = 16 B, kopia = jeden inkrement licznika).
2. **Brak stanu nieprawidłowego** — każdy `Expression` zawsze wskazuje na poprawny węzeł.
3. Komplet 9 typów wyrażeń z §1, z definicjami, które faktycznie się linkują.
4. `SymbolicMatrix` użyteczny na tyle, żeby dało się z niego złożyć i pomnożyć transformacje jednorodne 4×4.
5. Minimalna normalizacja, wystarczająca, by FK 6-DOF nie eksplodował trywialnymi węzłami (uzasadnienie ilościowe w §11).
6. Porównywanie strukturalne na potrzeby testów, **bez** obietnicy równoważności algebraicznej.
7. Zachowanie kompatybilności z `SymbolicTransform forwardKinematics_;` w `IkEquationBuilder`.

## 5. Non-goals

Poza zakresem tego proposalu (jawnie, za promptem): `Atan2`, `Acos`, `Asin`, `Sqrt`, `Power`, różniczkowanie symboliczne, `EquationSimplifier`, `EquationSolver`, `IkPatternDetector`, `CodeGenerator`, `ConstraintBuilder`, solver IK, wzory DH, generowanie kodu, parser wyrażeń tekstowych, serializacja, render LaTeX, interning / hash-consing, globalny cache, arena allocator, ogólna biblioteka algebry liniowej.

Nie projektuję też `JointTransformBuilder` ani `ForwardKinematicsBuilder` — wskazuję jedynie, jakie **minimalne** wymagania nakładają na tę warstwę (§10.4, §12.5).

## 6. Proponowana reprezentacja typów

### 6.1 Węzły — płaskie struktury, bez dziedziczenia

```
ConstantNode { double value; }
SymbolNode   { std::string name; }
AddNode      { Expression lhs, rhs; }
SubtractNode { Expression lhs, rhs; }
MultiplyNode { Expression lhs, rhs; }
DivideNode   { Expression lhs, rhs; }
NegateNode   { Expression operand; }
SinNode      { Expression operand; }
CosNode      { Expression operand; }

struct ExpressionNode { std::variant<ConstantNode, SymbolNode, AddNode, SubtractNode,
                                     MultiplyNode, DivideNode, NegateNode, SinNode, CosNode> value; }
```

Kluczowa zmiana względem stanu obecnego: **dzieci są przechowywane jako `Expression` przez wartość** (16 B, własny `shared_ptr`), a nie jako `shared_ptr<const Expression>`. Rekurencja jest przerwana po stronie `Expression`, który trzyma `shared_ptr<const ExpressionNode>` — dokładnie kierunek zatwierdzony w promptcie.

### 6.1a Dlaczego `ExpressionNode` musi być strukturą, a nie aliasem wariantu

Pierwsza wersja tego proposalu proponowała `using ExpressionNode = std::variant<...>` (jak dziś). **Zweryfikowałem to kompilatorem i jest to ill-formed** — zależności cykliczne układają się tak:

```
ExpressionNode  →  potrzebuje kompletnego  AddNode
AddNode         →  potrzebuje kompletnego  Expression   (pole przez wartość)
Expression      →  potrzebuje TYLKO deklaracji ExpressionNode   (shared_ptr dopuszcza typ niekompletny)
```

Cykl daje się przeciąć wyłącznie w ostatnim ogniwie — ale żeby to zrobić, `ExpressionNode` musi być **deklarowalny z wyprzedzeniem**, a aliasu typu (`using`) zadeklarować z wyprzedzeniem się nie da. Stąd `ExpressionNode` jako `struct` opakowująca wariant w polu `value`.

Zweryfikowany, poprawny porządek w nagłówku:

```
1.  struct ExpressionNode;                          // forward declaration
2.  class Expression { shared_ptr<const ExpressionNode> node_; };   // typ niekompletny OK
3.  struct ConstantNode / AddNode / ...             // tu Expression jest już kompletny
4.  struct ExpressionNode { std::variant<...> value; };             // definicja
```

Sprawdzone empirycznie: kompiluje się, linkuje i działa; `sizeof(Expression) == 16 B` zgodnie z założeniem. Koszt tej korekty to jeden dodatkowy poziom dostępu — `expression.node().value` zamiast `expression.node()` przy `std::visit`. Uznaję to za akceptowalne; alternatywą byłoby trzymanie dzieci jako `shared_ptr<const ExpressionNode>` zamiast `Expression`, co odebrałoby węzłom wygodę pracy na typie wartościowym.

**Dlaczego 9 osobnych typów, a nie 4 z enumem operatora?**

Rozważana alternatywa:

```
BinaryNode { BinaryOperator op; Expression lhs, rhs; }
UnaryNode  { UnaryOperator op; Expression operand; }
```

| | 9 osobnych typów (rekomendowane) | 4 typy + enum operatora |
|---|---|---|
| Mapowanie na `ExpressionType` | 1:1, bez tłumaczenia | dwupoziomowe (wariant → enum) |
| `std::visit` | wyczerpujący, kompilator wymusza obsługę wszystkich | 4 gałęzie + wewnętrzny `switch`, bez wymuszenia wyczerpania na poziomie operatora |
| Przyszły simplifier / codegen | dopasowanie wzorca wprost na typie | dopasowanie dwuetapowe |
| Liczba linii w nagłówku | ~9 linii struktur | ~4 linie + 2 enumy |
| Rozszerzenie o `Atan2` (2-arg) | nowy typ, wariant rośnie | nowa wartość enuma, bez zmiany wariantu |

Rekomendacja: **9 osobnych typów**. Powód rozstrzygający: każdy przyszły konsument tej warstwy (simplifier, generator kodu, ewaluator) to *wizytor*, a wyczerpujący `std::visit` po odrębnych typach jest jedynym wariantem, w którym kompilator wyłapie pominięty przypadek po dodaniu nowego typu węzła. Wersja z enumem przesuwa ten błąd do runtime'u (`default:` albo brakujący `case`).

**Dlaczego bez dziedziczenia** (usunięcie `BinaryExpression`/`UnaryExpression` jako baz): eliminuje podwójne nawiasy przy inicjalizacji agregatowej i fałszywą relację „is-a". Koszt: dwa powtórzone pola w czterech strukturach. Jest to też spójne z konwencją `model/` w tym repo — płaskie agregaty bez zachowania i bez hierarchii.

**Nazewnictwo:** proponuję `*Node` zamiast obecnego `*Expression` (`AddNode` vs `AddExpression`). Powód: `AddExpression` sugeruje, że to *jest* wyrażenie, podczas gdy wyrażeniem jest `Expression`, a to jest jego węzeł. Ponieważ obecne nazwy i tak nie mają ani jednego użycia poza własnym nagłówkiem, zmiana nikogo nie kosztuje. **Do zatwierdzenia** (§17.1).

### 6.2 `Expression`

```
class Expression
{
public:
    Expression();                                  // stała 0, współdzielony węzeł singletonowy

    ExpressionType type() const;                   // dyskryminant wariantu
    const ExpressionNode& node() const;            // dostęp tylko do odczytu

private:
    explicit Expression(ExpressionNode node);      // TYLKO dla ExpressionFactory (§10.1)
    friend class ExpressionFactory;

    std::shared_ptr<const ExpressionNode> node_;   // NIGDY null
};
```

`ExpressionPtr` (obecny alias `shared_ptr<const Expression>`) **znika** — jest artefaktem odwróconej indyrekcji. Jedynym jego użytkownikiem jest dziś `ExpressionFactory.hpp`, który i tak przeprojektowujemy.

## 7. Ownership i lifetime

```
Expression  (16 B, wartość)
   └── shared_ptr<const ExpressionNode>  ─── współwłasność
          └── ExpressionNode (wariant)
                 ├── Expression lhs   (16 B, własny shared_ptr)
                 └── Expression rhs   (16 B, własny shared_ptr)
```

- **Kto posiada węzły:** każdy `Expression` jest współwłaścicielem swojego węzła. Węzeł jest współwłaścicielem swoich dzieci (przez ich `Expression`). Ostatni `Expression` wskazujący na węzeł go zwalnia.
- **Jak długo żyją podwyrażenia:** dokładnie tak długo, jak istnieje choć jeden `Expression` na nie wskazujący — bezpośrednio albo przez rodzica. Nie ma znaczenia, czy zmienna lokalna użyta do zbudowania poddrzewa już wyszła z zakresu.
- **Dlaczego `shared_ptr<const ExpressionNode>` jest bezpieczny:** `const` uniemożliwia modyfikację węzła przez kogokolwiek; skoro nikt nie może modyfikować, współdzielenie jest zawsze bezpieczne, także między wątkami czytającymi (sam licznik referencji `shared_ptr` jest atomowy).
- **Czy występują cykle własności:** **nie, i to strukturalnie niemożliwe.** Węzeł powstaje wyłącznie w konstruktorze, z dzieci, które **już istnieją**. Nie ma żadnej operacji mutującej, więc nie ma sposobu, by istniejący węzeł zaczął wskazywać na węzeł utworzony później. Graf jest z konstrukcji skierowany i acykliczny (DAG — nie drzewo, bo poddrzewa mogą być współdzielone).
- **Czy potrzebny `weak_ptr`:** nie. `weak_ptr` służy przerywaniu cykli albo obserwacji bez własności — żadne z tych tu nie występuje.
- **Czy użytkownik może zmodyfikować istniejący węzeł:** nie. `node()` zwraca `const&`, wskaźnik jest na `const`, żadna metoda nie jest mutująca, brak setterów.

**Ryzyko do odnotowania (nie do rozwiązania teraz):** zwalnianie bardzo głębokiego łańcucha `shared_ptr` jest rekurencyjne i teoretycznie może przepełnić stos. Głębokość drzewa FK dla ramienia 6-DOF to rząd dziesiątek poziomów — daleko od zagrożenia. Odkładam; patrz §15.

## 8. Publiczne API

### 8.1 `Expression`

| Element | Sygnatura | Dostęp | Uwagi |
|---|---|---|---|
| konstruktor domyślny | `Expression()` | publiczny | stała `0.0`, współdzielony węzeł |
| konstruktor z węzła | `explicit Expression(ExpressionNode)` | **prywatny**, `friend ExpressionFactory` | alokuje; §10.1 |
| typ | `ExpressionType type() const` | publiczny | `noexcept` |
| węzeł | `const ExpressionNode& node() const` | publiczny | `noexcept`, tylko odczyt |

### 8.2 `ExpressionFactory`

**Kształt: klasa bezstanowa z metodami `const`** — bez zmian względem obecnej konwencji.

Rozważane alternatywy i uzasadnienie:

| Wariant | Ocena |
|---|---|
| **klasa bezstanowa, metody `const`** (rekomendacja) | Zgodna z `UrdfModelLoader`, `KinematicChainBuilder`, `JointTransformBuilder`, `ForwardKinematicsBuilder` — wszystkie są dokładnie takie. `ForwardKinematicsBuilder::build` już dziś przyjmuje `const JointTransformBuilder&`, więc przekazywanie bezstanowego współpracownika przez referencję jest w tym repo ustalonym wzorcem. |
| statyczne metody | Zrywa z konwencją repo; zamyka drogę do późniejszego dodania stanu lub konfiguracji do fabryki bez zmiany wszystkich wywołań. |
| wolne funkcje | To samo co wyżej, plus zaśmieca przestrzeń nazw `kinemaforge::ik` dziewięcioma bardzo ogólnymi nazwami (`add`, `multiply`, `sin`, `cos`…), z realnym ryzykiem kolizji z `std::sin`/`std::cos` przy ADL. |

Sygnatury — **przyjmują i zwracają `Expression` przez wartość** (16 B, kopia to inkrement licznika; przekazywanie przez `const&` nie dałoby tu nic poza szumem):

```
Expression constant(double value) const;
Expression symbol(std::string name) const;

Expression add(Expression lhs, Expression rhs) const;
Expression subtract(Expression lhs, Expression rhs) const;
Expression multiply(Expression lhs, Expression rhs) const;
Expression divide(Expression lhs, Expression rhs) const;

Expression negate(Expression operand) const;
Expression sin(Expression operand) const;
Expression cos(Expression operand) const;
```

### 8.3 Predykaty i porównywanie (wolne funkcje)

```
bool isConstant(const Expression&) noexcept;
bool isZero(const Expression&) noexcept;        // Constant o wartości dokładnie 0.0
bool isOne(const Expression&) noexcept;         // Constant o wartości dokładnie 1.0
double constantValue(const Expression&);        // wymaga isConstant, inaczej assert

bool sameNode(const Expression&, const Expression&) noexcept;      // tożsamość wskaźnika
bool structurallyEqual(const Expression&, const Expression&);      // rekurencyjne porównanie kształtu
```

**Świadomie BEZ `operator==`.** Uzasadnienie: `operator==` na typie reprezentującym wyrażenie matematyczne czyta się jako „są sobie równe", co byłoby fałszywe — `x + y` i `y + x` są algebraicznie równe, a strukturalnie różne. Nazwana funkcja `structurallyEqual` zmusza czytelnika do zauważenia, że gwarancja jest słabsza. `sameNode` jest osobno, bo odpowiada na inne pytanie (czy to fizycznie ten sam współdzielony węzeł) i jest O(1) — potrzebne w testach współdzielenia poddrzew.

**`structurallyEqual` zaczyna od `sameNode` (korekta z review).** Skoro poddrzewa są w tej warstwie masowo współdzielone (§14), krótkie spięcie na tożsamości wskaźnika sprowadza porównanie całych wspólnych gałęzi do O(1) zamiast O(rozmiar). Dla drzew FK, gdzie ta sama komórka macierzy trafia w wiele miejsc, to nie mikro-optymalizacja, tylko różnica rzędu.

**Porównywanie stałych `double`:** `structurallyEqual` używa dokładnego `==`, nie epsilona. To predykat **strukturalny — ani numeryczny, ani algebraiczny**. Odpowiada wyłącznie na pytanie „czy te dwa drzewa mają ten sam kształt i te same wartości w liściach". W szczególności:

- `x + y` vs `y + x` → `false` (równe algebraicznie, różne strukturalnie),
- `constant(0.1 + 0.2)` vs `constant(0.3)` → `false` (różne bitowo, bliskie numerycznie).

Testy potrzebujące porównania numerycznego powinny wyciągnąć `constantValue()` i użyć `EXPECT_DOUBLE_EQ`.

**Kwestia `NaN` — rozwiązana u źródła (ustalenie z review).** Pierwsza wersja twierdziła, że „`NaN` nigdy nie jest równy sobie", co byłoby **niespójne** z krótkim spięciem na `sameNode`: ten sam węzeł z `NaN` wychodziłby równy sobie przez pierwszą gałąź, a nierówny przez porównanie wartości. Zamiast wybierać, którą semantykę uznać za obowiązującą, usuwamy problem: **`constant()` odrzuca wartości niefinitywne** (§13), więc `NaN` i `±Inf` nie mogą znaleźć się w poprawnym grafie. Pytanie o ich porównywanie przestaje istnieć.

## 9. Inwarianty

| Pytanie z promptu | Rozstrzygnięcie |
|---|---|
| Czy pusty `Expression` jest dozwolony? | **Nie.** `node_` nigdy nie jest null. |
| Czy konstruktor domyślny istnieje? | **Tak.** |
| Czy pusty stan ma oznaczać zero? | Nie ma pustego stanu. Konstruktor domyślny tworzy **stałą 0** — to wartość, nie stan pusty. |
| Czy każdy `Expression` musi zawsze posiadać prawidłowy węzeł? | **Tak**, bezwarunkowo. Nie ma konstruktora ani operacji dającej `Expression` bez węzła. |
| Bezpieczeństwo w `std::array` | Zachowane — patrz niżej. |
| Dostęp do `node()` / `type()` | Zawsze bezpieczny, bez sprawdzania null, bez zwracania `optional`. |

**Wpływ na `SymbolicMatrix` (wprost wymagany w promptcie):** `std::array<Expression, Rows*Columns>` przy inicjalizacji domyślnej wymaga, by `Expression` był domyślnie konstruowalny. Gdybyśmy usunęli konstruktor domyślny (wariant „brak stanu domyślnego w ogóle"), to:
- `std::array` przestałby się domyślnie inicjalizować, więc `SymbolicMatrix` potrzebowałby własnej inicjalizacji wszystkich `Rows*Columns` komórek przez `index_sequence`;
- **i, co ważniejsze, `SymbolicTransform forwardKinematics_;` w `IkEquationBuilder` przestałby się kompilować** przy `IkEquationBuilder() = default`.

Dlatego konstruktor domyślny zostaje — ale zamiast dawać każdej komórce własną kopię wariantu (dziś: 16 niezależnych `ConstantExpression{0.0}` w macierzy 4×4), wszystkie wskazują na **jeden współdzielony węzeł zera**:

```
Expression::Expression() : node_(sharedZeroNode()) {}
// sharedZeroNode(): inline, function-local static — jedna instancja na cały program
```

Macierz 4×4 domyślnie: 16 × 16 B = 256 B, zero alokacji (16 inkrementów licznika na jednym węźle), zamiast obecnych 640 B z 16 niezależnymi wariantami.

**Ryzyko, które ta decyzja wnosi (i mitygacja):** skoro domyślna komórka to `0`, zapomniana komórka nie krzyczy — macierz 4×4 z niewypełnionym `(3,3)` cicho zachowa się jak zdegenerowana, zamiast zgłosić błąd. Mitygacja: nazwane konstruktory `zeros()` i `identity()` (§12), tak by kod produkcyjny praktycznie nigdy nie polegał na domyślnej inicjalizacji, a transformacje startowały od `identity()`.

## 10. Zasady tworzenia węzłów

1. **Jedyna droga do węzła złożonego prowadzi przez `ExpressionFactory` — i jest to wymuszone przez kompilator, nie tylko zalecane.**

   Pierwsza wersja tego proposalu deklarowała tę zasadę, ale jednocześnie zostawiała `explicit Expression(ExpressionNode)` **publiczny** — czyli dowolny kod mógł ominąć fabrykę i zbudować `Expression{ExpressionNode{DivideNode{constant(1), constant(0)}}}` albo `Expression{ExpressionNode{SymbolNode{""}}}`, całkowicie poza kontrolą kontraktów. Inwarianty fabryki nie były więc inwariantami modelu — były konwencją.

   **Decyzja (z review): konstruktor z węzła jest prywatny, `ExpressionFactory` jest `friend`.** Publiczny pozostaje wyłącznie konstruktor domyślny (stała 0 — leaf, nie węzeł złożony). Dzięki temu każda asercja z §13 (skończoność stałej, niepusta nazwa symbolu, brak dzielenia przez literalne zero) obowiązuje **dla całego grafu**, bez wyjątków.

   Testy nie tracą na tym nic: budują wyrażenia przez fabrykę i sprawdzają strukturę przez publiczne `node()`.

2. **Nowy węzeł powstaje przy każdym wywołaniu fabryki**, chyba że normalizacja zwróci istniejący operand (np. `add(x, 0)` zwraca `x` — bez alokacji) albo zadziała współdzielenie zera (§14).
3. **Operandy są współdzielone, nigdy kopiowane w głąb.** `multiply(x, x)` tworzy jeden węzeł `MultiplyNode`, którego oba dzieci wskazują na **ten sam** węzeł `x`.
4. **Kolejność argumentów jest zachowywana dosłownie.** `subtract(a, b)` daje `SubtractNode{a, b}`; fabryka nigdy nie przestawia operandów (to byłoby przekształcenie algebraiczne — poza zakresem).
5. **Brak ogólnego cache'u i interningu.** Dwa wywołania `constant(1.0)` dają dwa różne węzły o równej wartości — `sameNode` da `false`, `structurallyEqual` da `true`. Jedyny wyjątek to `constant(0.0)` (§14). Świadomie; szersze uzasadnienie w §14.

## 11. Minimalna normalizacja

To najbardziej konsekwencjonalna decyzja w dokumencie, więc najpierw liczby.

### 11.1 Dlaczego bez tego pierwszy konsument się dławi

Transformacja pojedynczego jointu to złożenie obrotu z `origin.rpy` i translacji. Mnożenie dwóch macierzy 4×4 to **64 mnożenia i 48 dodawań** — czyli 112 nowych węzłów na jedno złożenie, niezależnie od tego, czy operandy są zerami i jedynkami.

Konkret z `kr640.urdf`: **wszystkie 7 jointów ma `rpy="0 0 0"`**. Macierz obrotu z takiego rpy to dokładnie macierz jednostkowa — same stałe `0.0` i `1.0`. Bez zwijania, FK dla łańcucha base→tool0 (7 jointów, 6 złożeń) zbuduje ok. **700 węzłów**, z których zdecydowana większość to `x*1`, `x*0`, `x+0` — czyli szum reprezentujący liczby, które znamy w momencie budowania.

Z `kr4_r600.urdf` jest tylko trochę lepiej: `rpy` bywa niezerowe, ale to `π/2` i `π`, więc `sin`/`cos` z **argumentu stałego** — również obliczalne od ręki.

Bez normalizacji wynik `ForwardKinematicsBuilder` byłby technicznie poprawny, ale bezużyteczny do oglądania i niepotrzebnie kosztowny dla każdego kolejnego etapu.

### 11.2 Zatwierdzony zestaw reguł

**Kategoria A — zwijanie stałych (czysta ewaluacja):**

```
add(c₁, c₂)       → constant(v₁ + v₂)
subtract(c₁, c₂)  → constant(v₁ - v₂)
multiply(c₁, c₂)  → constant(v₁ · v₂)      [obejmuje c · 0 → 0, bo oba są znane i skończone]
divide(c₁, c₂)    → constant(v₁ / v₂)      [c₂ ≠ 0; patrz §13]
negate(c)         → constant(-v)
sin(c)            → constant(std::sin v)
cos(c)            → constant(std::cos v)
```

Wynik każdego zwinięcia przechodzi przez `constant()`, więc podlega kontroli skończoności (§13) — to domyka inwariant „w grafie nie ma `NaN` ani `Inf`" także dla wartości powstałych z obliczeń, nie tylko podanych wprost.

**Kategoria B — wyłącznie elementy neutralne:**

```
add(x, 0) → x        add(0, x) → x
subtract(x, 0) → x
multiply(x, 1) → x   multiply(1, x) → x
divide(x, 1) → x
```

Każda reguła to sprawdzenie **wyłącznie bezpośrednich dzieci** — O(1), bez przechodzenia drzewa, bez porządkowania, bez rozpoznawania równoważności.

### 11.3 Anihilator `x · 0 → 0` — odrzucony (decyzja z review)

Pierwsza wersja tego proposalu zawierała `multiply(x, 0) → 0` i uzasadniała ją wyłącznie odpornością na `NaN`/`Inf`. **To była zła analiza — na niewłaściwym poziomie.** Właściwy zarzut dotyczy nie *wartości*, lecz **dziedziny określoności**:

```
x = 1 / q        →    (1 / q) · 0    jest nieokreślone dla q = 0
                      po zwinięciu → 0    twierdzi, że jest określone wszędzie
```

Reguła kasuje więc informację o osobliwości. Ma to bezpośrednie znaczenie dla dalszych etapów projektu: przy przekształcaniu równań IK mianowniki będą zależeć od zmiennych złączowych, a ich miejsca zerowe odpowiadają **osobliwościom albo konfiguracjom wykluczonym** — czyli dokładnie tej informacji, którą warto zachować.

Kluczowa obserwacja: sprawdzanie samego `divide(c, constant(0))` (§13) tego **nie** rozwiązuje, bo mianownik na ogół jest wyrażeniem, nie stałą — zeruje się dopiero dla pewnych wartości zmiennych, czego lokalnie stwierdzić nie można.

**Zwijanie stałych pozostaje w mocy**: `multiply(constant(a), constant(0))` → `constant(0)` jest bezpieczne, bo oba argumenty są znane i skończone, więc żadna dziedzina nie znika. Anihilator dotyczy wyłącznie przypadku, gdy `x` jest wyrażeniem symbolicznym.

**Kiedy można wrócić do tej reguły:** gdy zostanie jawnie ustalona semantyka dziedziny wyrażeń i sposób śledzenia warunków obowiązywania. Zarysowany kierunek na przyszłość (świadomie **poza** zakresem tego proposalu): reguła jest bezpieczna dla `x` bez dzielenia w poddrzewie — `sin`, `cos`, `+`, `−`, `·`, stałe i symbole są funkcjami totalnymi na ℝ, więc dla takiego `x` żadna dziedzina nie jest tracona. „Wolne od dzielenia" dałoby się propagować jako flagę wyliczaną przy konstrukcji węzła (O(1), bottom-up). Nie proponuję tego teraz — to dodatkowe pole w każdym węźle dla korzyści, która przy obecnej skali nie jest krytyczna (§11.4).

### 11.3a Konsekwencja, którą trzeba nazwać wprost: `A · I ≠ A`

Odrzucenie anihilatora ma **bezpośredni, widoczny skutek dla mnożenia macierzy**, który łatwo przeoczyć przy projektowaniu testów. Dla `C = A · I` każda komórka jest sumą `K` iloczynów, z których tylko jeden trafia w jedynkę na przekątnej; pozostałe mnożą przez zero i — zgodnie z zatwierdzoną semantyką — **zostają w drzewie**.

Zweryfikowane zachowanie (symulacja, macierze 2×2):

| Zawartość `A` | Wynik `A · I` | `A` odtworzone? |
|---|---|---|
| komórki **symboliczne** | `(a₀₀ + (a₀₁ · 0))` | ✗ nigdy |
| komórki **stałe** | `1`, `2`, `11`, `12` | ✓ w pełni |
| **mieszane** | `q`, ale `((q · 0) + 1)` | częściowo |

Dlaczego macierz stałych *jednak* się odtwarza: `multiply(constant, constant(0))` **zwija się** do `constant(0)` (kategoria A, §11.2), a wtedy `add(x, 0) → x` już zadziała. Dla operandu symbolicznego zwijanie nie ma czego policzyć, więc `Multiply` zostaje i blokuje kolejne `add`.

**Wniosek dla testów:** test w rodzaju „`A · I` zwraca komórki `A` niezmienione" (obecny w pierwszej wersji jako `MultiplyByIdentityKeepsOperands`) **jest sprzeczny z tą decyzją** dla macierzy symbolicznych i został zastąpiony dwoma testami opisującymi rzeczywiste zachowanie (§17.2).

**Kiedy `A · I → A` stanie się prawdziwe:** dopiero po wprowadzeniu `EquationSimplifier` ze śledzeniem dziedziny, który będzie mógł usunąć `x · 0` tam, gdzie `x` jest określone wszędzie. To nie jest brak w tej warstwie — to świadomie przesunięta odpowiedzialność.

### 11.4 Zmierzony koszt tej decyzji

Zasymulowałem złożenie FK dla łańcucha 6-DOF (sześć transformacji jednorodnych 4×4, obrót wokół Z plus translacja), z pełnym zestawem reguł i licząc **unikalne węzły DAG** (współdzielone poddrzewa liczone raz):

| Zestaw reguł | Węzły DAG |
|---|---|
| z anihilatorem `x·0 → 0` | **87** |
| bez anihilatora (decyzja zatwierdzona) | **375** |

Koszt to **4,3×**. Uznaję go za akceptowalny: 375 węzłów to wciąż rozmiar w pełni obsługiwalny, wynik jest **poprawny**, a przyszły `EquationSimplifier` — mający pełny obraz drzewa i możliwość śledzenia dziedziny — usunie te człony poprawnie, zamiast robić to teraz na ślepo. Odnotowuję jednak, że argument o „eksplozji drzewa" z §11.1 jest przez tę decyzję osłabiony, nie unieważniony: bez **żadnej** normalizacji byłoby ~700+ węzłów, z kategorią A+B jest 375.

### 11.5 Szum numeryczny w zwijaniu trygonometrii (ustalenie z review)

Zwijanie `sin`/`cos` stałych **nie daje dokładnych zer i jedynek** dla typowych argumentów z URDF:

| Wyrażenie | Wartość `double` | Dokładne? |
|---|---|---|
| `sin(π)` | `1.2246467991473532e-16` | ✗ |
| `cos(π/2)` | `6.123233995736766e-17` | ✗ |
| `cos(π)` | `-1.0` | ✓ |
| `sin(0)` | `0.0` | ✓ |
| `cos(0)` | `1.0` | ✓ |

Konsekwencja jest **szersza niż sam szum w wynikach** i warto ją nazwać wprost: skoro `cos(π/2)` zwija się do `6.12e-17`, a nie do `0.0`, to dla takiej komórki **nie zadziała również zatwierdzona reguła** `add(x, 0) → x` ani `multiply(x, 1) → x` — bo nie ma tam ani zera, ani jedynki, tylko liczby bliskie.

Praktyczny podział:

- **`kr640.urdf`** — wszystkie jointy mają `rpy="0 0 0"`, więc `sin(0)`/`cos(0)` zwijają się **dokładnie** i normalizacja z kategorii B działa w pełni.
- **`kr4_r600.urdf`** — `rpy` zawiera `π` i `π/2`, więc rotacje z origin dają stałe rzędu `1e-16` zamiast zer. Normalizacja w dużej mierze **nie zadziała**, a macierze będą zawierać szum numeryczny.

**Decyzja: przyjmujemy dokładny wynik `std::sin`/`std::cos`, bez żadnej tolerancji.** Uzasadnienie: wprowadzenie progu („jeśli |v| < ε, to 0") uczyniłoby tolerancję **częścią semantyki symbolicznej** — czyli decyzją o wiele poważniejszą niż optymalizacja, i wymagającą osobnego rozstrzygnięcia (jaka wartość ε? czy dotyczy tylko trygonometrii, czy każdej stałej? czy jest częścią kontraktu `constant()`?).

**Odłożona decyzja do przyszłego rozstrzygnięcia:** czy wyniki funkcji trygonometrycznych bliskie `0`, `1` i `−1` mają podlegać kanonizacji — i jeśli tak, to na którym etapie (fabryka, `EquationSimplifier`, czy dopiero generator kodu).

### 11.6 Czego świadomie NIE ma

`x · 0 → 0` i `0 · x → 0` (§11.3), `x - x → 0`, `x / x → 1` (wymagają porównania strukturalnego operandów, czyli analizy równoważności), `0 - x → negate(x)` (przekształcenie, nie zwijanie), jakiekolwiek przestawianie, faktoryzacja, rozwijanie, tożsamości trygonometryczne, kanonizacja wartości bliskich zeru (§11.5). Wszystko to należy do przyszłego `EquationSimplifier`.

### 11.7 Gdzie to należy — fabryka czy przyszły simplifier?

**Decyzja: w `ExpressionFactory`.** Uzasadnienie:

- Kategoria A to nie upraszczanie, tylko **ewaluacja tego, co i tak jest znane w chwili budowania**. Odkładanie jej oznacza świadome budowanie węzła `sin(constant(1.5707963...))` po to, by później go zwinąć — praca wykonana dwa razy. Dodatkowo to właśnie tędy przechodzi kontrola skończoności stałych (§13), więc zwijanie musi być tu, żeby inwariant obejmował także wyniki obliczeń.
- Kategoria B nie wymaga wiedzy o całym drzewie — sprawdza wyłącznie bezpośrednie dzieci.
- Obie kategorie są **lokalne i bezstanowe** — mieszczą się w definicji fabryki („zbuduj najprostszy węzeł reprezentujący tę operację na tych operandach").

Kontrargument, który uznaję: to sprawia, że fabryka nie jest już czystym konstruktorem, a jej testy muszą sprawdzać, że `add(x, zero)` zwraca `x`, a nie węzeł `Add`. Uważam to za akceptowalną cenę — i tak trzeba to przetestować, a alternatywa (fabryka „czysta" plus obowiązkowy przebieg zwijania po każdym mnożeniu macierzy) jest bardziej złożona, nie mniej.

## 12. Projekt `SymbolicMatrix`

### 12.1 Inicjalizacja i wymiary

`std::array<Expression, Rows*Columns>` z inicjalizacją domyślną — każda komórka to stała 0 na współdzielonym węźle (§9).

**Doprecyzowanie po review:** stwierdzenie „zero alokacji" z pierwszej wersji było nieścisłe. Pierwsze w programie utworzenie współdzielonego węzła zera wykonuje **jedną** alokację (function-local static). Każda kolejna domyślnie skonstruowana macierz nie alokuje już nic — same inkrementy licznika referencji.

**Zerowe wymiary są zakazane:**

```
static_assert(Rows > 0);
static_assert(Columns > 0);
```

Bez tego szablon formalnie dopuszczałby `SymbolicMatrix<3, 0>`, a mnożenie (§12.5) zaczyna od `lhs(i, 0)` — czyli od indeksu, który dla `K = 0` jest poza zakresem. `static_assert` odrzuca taki typ już przy instancjacji, zamiast pozwalać mu istnieć i psuć się dopiero przy użyciu.

### 12.2 `operator()(row, column)` — dostęp, definicja widoczna dla szablonu

**Nazwa zmieniona z `at()` na `operator()` (decyzja z review).** Powód: `at()` w kontenerach standardowych oznacza dostęp *sprawdzany w czasie wykonania*, zwykle rzucający `std::out_of_range`. Nasza semantyka jest inna — sprawdzenie znika w buildzie release (`assert`), więc nazwa `at()` obiecywałaby gwarancję, której nie dajemy. `operator()` jest w tej roli neutralny i konwencjonalny dla macierzy.

Definicja **inline w ciele klasy** (szablon musi mieć definicję widoczną w miejscu instancjacji — to bezpośrednia przyczyna dzisiejszego `undefined reference`). Indeksowanie wierszami: `values_[row * Columns + column]`.

Dwa przeciążenia: `Expression& operator()(...)` (zapis) i `const Expression& operator()(...) const` (odczyt).

### 12.3 Błędne indeksy

**`assert`, nie wyjątek i nie `std::expected`.** Uzasadnienie: indeks poza zakresem macierzy o rozmiarze znanym w czasie kompilacji to **błąd programisty**, nie sytuacja domenowa — dokładnie to rozróżnienie, które ustaliliśmy dla `KinematicChainBuilder` (§10/§11 tamtego proposalu: błędy domenowe → `std::expected`, naruszenia kontraktu → `assert`). Żaden poprawny kod nie odpytuje `(7, 2)` macierzy 4×4.

Rozważana alternatywa (wariant A z review): zachować nazwę `at()` i rzucać `std::out_of_range`. Odrzucona — macierze są tu strukturą wewnętrzną, wymiary są znane w czasie kompilacji, a błędny indeks jest bugiem implementacji, nie sytuacją do obsłużenia w czasie wykonania. Kontrola w release'ie kosztowałaby przy każdym z 64 dostępów na jedno mnożenie macierzy.

### 12.4 Nazwane konstruktory

```
static SymbolicMatrix zeros();                              // wszystkie komórki = constant 0
static SymbolicMatrix identity() requires (Rows == Columns); // przekątna = constant 1, reszta 0
```

`identity()` ograniczone `requires` do macierzy kwadratowych — dla `SymbolicMatrix<3,1>` nie ma sensu i nie powinno się kompilować. To dla tej warstwy kluczowe: transformacja jednorodna startuje od jednostkowej, a nie od zerowej.

**Współdzielenie jedynki (korekta z review):** `identity()` tworzy `Expression` dla `1.0` **raz** i wstawia tę samą wartość na całej przekątnej, zamiast wołać `factory.constant(1.0)` w każdej iteracji:

```
const Expression one = factory.constant(1.0);
for (std::size_t i = 0; i < Rows; ++i)
    result(i, i) = one;
```

Dla 4×4 to jedna alokacja zamiast czterech, a wszystkie komórki przekątnej są `sameNode`.

### 12.5 Mnożenie

```
template <std::size_t R, std::size_t K, std::size_t C>
SymbolicMatrix<R, C> multiply(const SymbolicMatrix<R, K>& lhs,
                              const SymbolicMatrix<K, C>& rhs,
                              const ExpressionFactory& factory);
```

Wymiary uzgadniane w czasie kompilacji (`K` wspólne) — niezgodność wymiarów to błąd kompilacji, nie runtime'u.

**Kolejność operandów w wyniku (deterministyczna, wymagana przez testy):** dla `C[i][j]` sumowanie left-fold po rosnącym `k`:

```
C[i][j] = ((A[i][0]·B[0][j] + A[i][1]·B[1][j]) + A[i][2]·B[2][j]) + ...
```

Czyli: iloczyny budowane jako `multiply(A[i][k], B[k][j])` (lewy operand z `A`), suma akumulowana `add(dotychczasowa, nowy)`. Dzięki normalizacji z §11 komórki jednostkowe znikają w trakcie, a nie po fakcie.

**Wolna funkcja z jawną fabryką, nie `operator*`.** Uzasadnienie:
- Zgodność z konwencją repo — `ForwardKinematicsBuilder::build(chain, transformBuilder)` już dziś przyjmuje bezstanowego współpracownika jawnie.
- `operator*` musiałby wziąć fabrykę z powietrza (domyślnie skonstruowaną), więc gdyby fabryka kiedyś zyskała stan lub konfigurację, **nie byłoby jak jej przekazać**.

  *(Korekta po review: pierwsza wersja twierdziła, że jawna fabryka pozwala „podmienić politykę normalizacji" na inną klasę. To przesada — parametr ma typ `const ExpressionFactory&`, a metody nie są ani wirtualne, ani szablonowe, więc podstawienie innego typu jest niemożliwe. Realna korzyść jest węższa: można dodać stan lub konfigurację do tej samej klasy bez dotykania miejsc wywołania.)*
- `operator*` ukrywa koszt: `A * B` wygląda na tanie, a alokuje do 112 węzłów.

Alternatywa (`operator*` + wewnętrzna domyślna fabryka) jest czytelniejsza w zapisie i możliwa do dołożenia później **na bazie** tej funkcji, gdyby okazała się potrzebna. **Do zatwierdzenia** (§17.3).

### 12.6 Czego celowo nie ma

Brak transpozycji, wyznacznika, odwracania, śladu, dodawania macierzy, mnożenia przez skalar. Nic z tego nie jest potrzebne do złożenia FK, a prompt jawnie zakazuje budowania ogólnej biblioteki algebry liniowej. `JointTransformBuilder` potrzebuje: `identity()`, `operator()` do zapisu, `multiply()`. Tyle wystarczy.

## 13. Obsługa błędów

| Sytuacja | Czy realna? | Rozstrzygnięcie |
|---|---|---|
| Indeks macierzy poza zakresem | błąd programisty | `assert` |
| `constantValue()` na węźle niebędącym stałą | błąd programisty | `assert` |
| **Stała niefinitywna (`NaN`, `±Inf`)** | **błąd programisty / przepełnienie** | **`assert(std::isfinite(value))` w `constant()`** |
| Dzielenie przez stałą zero | patrz niżej | `assert` |
| Dzielenie przez wyrażenie, które *może* być zerem | nierozstrzygalne statycznie | poza zakresem — nie sprawdzamy |
| Pusta nazwa symbolu | błąd programisty | `assert` |
| Brak pamięci | wyjątek `std::bad_alloc` z `make_shared` | propagowany, nieobsługiwany |

**Kontrola skończoności stałych (decyzja z review).** `ExpressionFactory::constant()` odrzuca `NaN` i `±Inf`:

```
Expression ExpressionFactory::constant(double value) const
{
    assert(std::isfinite(value));
    ...
}
```

Uzasadnienie: w symbolicznym FK stałe pochodzą z poprawnego URDF oraz z obliczeń trygonometrycznych na wartościach skończonych. **Nie ma dziś żadnego sensownego zastosowania dla literalnego `NaN` czy `Inf`** — ich pojawienie się oznacza, że coś już poszło źle wyżej.

Kluczowe dla szczelności tego inwariantu: **wyniki zwijania stałych (§11.2) też przechodzą przez `constant()`**, a nie omijają go, budując `ConstantNode` bezpośrednio. Dzięki temu asercja obejmuje również wartości powstałe z obliczeń — np. przepełnienie w `add(constant(1e308), constant(1e308))` zostanie złapane, a nie wpuszczone do grafu jako `Inf`.

Efekt uboczny, na który liczymy: znika cała klasa pytań o porównywanie `NaN` w `structurallyEqual` (§8.3).

**Wniosek: ta warstwa nie potrzebuje `std::expected` ani enuma błędów.** Nie ma tu **ani jednego błędu domenowego** — nie ma danych wejściowych od użytkownika, które mogłyby być „poprawne, ale nieakceptowalne". Każda z powyższych sytuacji to naruszenie kontraktu przez wywołujący kod. Tworzenie `SymbolicError` z jedną–dwiema wartościami, które i tak nigdy nie powinny wystąpić, dodałoby ceremonii bez wartości.

To jest spójne z linią przyjętą w `KinematicChainBuilder`: `std::expected` **tylko** dla przewidywalnych niepowodzeń domenowych; `assert` dla bugów.

**Uwaga o dzieleniu:** warto odnotować, że złożenie FK **w ogóle nie używa dzielenia** — transformacje jednorodne to wyłącznie `sin`, `cos`, `+`, `−`, `×`, negacja. `Divide` jest w zestawie na przyszłość (rozwiązywanie IK). Dlatego `assert` na dzieleniu przez literalne zero jest w praktyce bezkosztowy na tym etapie.

## 14. Koszt i współdzielenie podwyrażeń

- **Kopiowanie `Expression`:** rozmiar jednego `shared_ptr` + jeden atomowy inkrement licznika. Nigdy nie kopiuje drzewa ani nie alokuje. To realna poprawa względem stanu obecnego, gdzie kopia symbolu kopiuje `std::string`, czyli **alokuje**.

  *(Zmierzone `sizeof(Expression) == 16 B` to wynik dla tej platformy — x86-64/libstdc++ — a nie gwarancja języka. Podaję go jako dowód rzędu wielkości względem obecnych 40 B, nie jako inwariant architektury.)*
- **Kiedy powstaje nowy węzeł:** dokładnie raz na wywołanie fabryki, które nie zostało zwinięte przez normalizację ani nie trafiło we współdzieloną stałą. `make_shared<const ExpressionNode>` — jedna alokacja na węzeł (blok kontrolny i obiekt razem).
- **Współdzielony węzeł zera:** `constant(0.0)` zwraca **ten sam** węzeł co konstruktor domyślny `Expression` (korekta z review), zamiast alokować kolejne zera. Zero jest w tej warstwie najczęstszą stałą — wypełnia domyślnie każdą komórkę każdej macierzy — więc alokowanie go osobno za każdym razem byłoby czystym marnotrawstwem. Implementacyjnie: `constant(v)` zwraca `Expression{}` dla `v == 0.0`.

  Dwa niuanse do odnotowania: (a) `constant(-0.0)` zwróci węzeł `+0.0`, bo `-0.0 == 0.0` w IEEE-754 — bit znaku ginie, co przy `structurallyEqual` i tak było nierozróżnialne; (b) nie rozszerzam tego na `constant(1.0)`, mimo że jedynka też jest częsta — to już byłby początek ogólnego interningu, którego świadomie nie wprowadzamy. `identity()` rozwiązuje swój przypadek lokalnie, tworząc jedno wyrażenie i współdzieląc je na przekątnej (§12.4).
- **Współdzielenie poddrzew:** naturalne, gdy ten sam `Expression` trafi w więcej niż jedno miejsce. Przy mnożeniu macierzy `A[i][k]` jest używany w `C` wielokrotnie — każde użycie to współwłasność tego samego węzła, nie kopia. Dlatego wynik FK jest **DAG-iem, nie drzewem**; przy liczeniu rozmiaru trzeba o tym pamiętać (naiwne rekurencyjne zliczanie policzy współdzielone węzły wielokrotnie).
- **Interning / hash-consing: nie teraz, jawnie odłożone.** Dwa niezależne `constant(1.0)` dają dwa węzły. Argument za odłożeniem: interning wymaga globalnej tablicy z pytaniami o czas życia i bezpieczeństwo wątkowe, a jedyną korzyścią przy obecnej skali (setki węzłów) jest oszczędność pamięci rzędu kilku kilobajtów. Gdyby okazał się potrzebny, jego naturalne miejsce to **wnętrze `ExpressionFactory`** — dlatego właśnie fabryka jest obiektem, a nie zbiorem wolnych funkcji (§8.2). Brak arena allocatora i globalnego cache'u — z tego samego powodu.

## 15. Ryzyka i odłożone decyzje

| Ryzyko / decyzja | Ocena | Status |
|---|---|---|
| Rekurencyjne zwalnianie głębokiego drzewa może przepełnić stos | Głębokość FK dla 6-DOF: dziesiątki poziomów. Zagrożenie realne dopiero przy tysiącach. | Odłożone; iteracyjny destruktor gdyby kiedyś trzeba |
| Konstruktor domyślny = `0` maskuje niewypełnione komórki | Realne w macierzy 4×4, gdzie `(3,3)` musi być `1` | Mitygacja: `identity()` / `zeros()` jako domyślna droga (§9) |
| Drzewa FK są ~4,3× większe bez anihilatora `x·0 → 0` | Zmierzone: 87 → 375 węzłów dla 6-DOF. Poprawność zachowana, rozmiar wciąż mały | Zaakceptowane świadomie — usunięcie tych członów należy do `EquationSimplifier`, który może śledzić dziedzinę (§11.3) |
| Zwijanie trygonometrii daje szum `~1e-16` zamiast zer | Dla `kr4_r600.urdf` blokuje **także** zatwierdzone reguły `x+0→x` i `x·1→x` | Odłożona decyzja o kanonizacji (§11.5) |
| `structurallyEqual` a `NaN` | Rozwiązane u źródła — `constant()` odrzuca wartości niefinitywne, więc `NaN` nie wchodzi do grafu | Zamknięte (§13) |
| Brak interningu → powtarzalne stałe zajmują osobne węzły | Kilka kB przy obecnej skali. Zero jest wyjątkiem — współdzielone (§14) | Odłożone (§14) |
| Wynik FK jest DAG-iem, nie drzewem | Wpływa na przyszły simplifier i codegen (trzeba pamiętać o współdzieleniu) | Odnotowane, poza zakresem |
| `SymbolicRotation` / `SymbolicVector3` nieużywane | Nieszkodliwe aliasy | Zostawiam — `JointTransformBuilder` prawdopodobnie zbuduje rotację 3×3 przed osadzeniem jej w 4×4 |

## 16. Pełne przykładowe nagłówki po zmianach

> Poniższe są **projektami do review**, nie plikami do zapisania. Zgodnie z promptem nie modyfikuję źródeł.

### 16.1 `src/ik_equations/symbolic/Expression.hpp`

```cpp
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <variant>

namespace kinemaforge::ik {

enum class ExpressionType
{
    Constant,
    Symbol,
    Add,
    Subtract,
    Multiply,
    Divide,
    Negate,
    Sin,
    Cos
};

// Declaration order matters and is load-bearing:
//   ExpressionNode is forward-declared so Expression can hold a
//   shared_ptr to it while incomplete; the node structs then store
//   Expression by value (complete by then); ExpressionNode is defined
//   last. A `using ExpressionNode = std::variant<...>` alias cannot
//   work here — an alias is not forward-declarable, so the cycle has
//   nowhere to break.
struct ExpressionNode;

// Immutable value handle over a shared expression node. Copying an
// Expression copies one pointer — never the tree underneath it.
//
// The node is never null: a default-constructed Expression is the
// constant 0, and every other path takes a node by value. Nothing can
// mutate a node once built, so the graph is acyclic by construction and
// sub-expressions are safe to share.
class Expression
{
public:
    // The constant 0. Every cell of a fresh SymbolicMatrix starts here,
    // all sharing one node.
    Expression();

    ExpressionType type() const noexcept;
    const ExpressionNode& node() const noexcept;

private:
    // Private on purpose: every composite node must go through
    // ExpressionFactory, so the contracts it enforces (finite constants,
    // non-empty symbol names, no division by literal zero) hold for the
    // whole graph rather than only where callers remembered to use it.
    explicit Expression(ExpressionNode node);

    static std::shared_ptr<const ExpressionNode> sharedZeroNode();

    friend class ExpressionFactory;

    std::shared_ptr<const ExpressionNode> node_;
};

struct ConstantNode { double value{}; };
struct SymbolNode   { std::string name; };

struct AddNode      { Expression lhs, rhs; };
struct SubtractNode { Expression lhs, rhs; };
struct MultiplyNode { Expression lhs, rhs; };
struct DivideNode   { Expression lhs, rhs; };

struct NegateNode { Expression operand; };
struct SinNode    { Expression operand; };
struct CosNode    { Expression operand; };

struct ExpressionNode
{
    std::variant<
        ConstantNode,
        SymbolNode,
        AddNode,
        SubtractNode,
        MultiplyNode,
        DivideNode,
        NegateNode,
        SinNode,
        CosNode
    > value;
};

// --- predicates -----------------------------------------------------

bool isConstant(const Expression& expression) noexcept;
bool isZero(const Expression& expression) noexcept;
bool isOne(const Expression& expression) noexcept;

// Precondition: isConstant(expression).
double constantValue(const Expression& expression);

// --- comparison -----------------------------------------------------

// True when both handles refer to the very same shared node. O(1).
// Implemented as &lhs.node() == &rhs.node(), so it needs no friendship.
bool sameNode(const Expression& lhs, const Expression& rhs) noexcept;

// True when both trees have the same shape and the same leaf values.
//
// Structural, not algebraic and not numeric:
//   x + y            vs  y + x          -> false (equal algebraically)
//   constant(.1+.2)  vs  constant(.3)   -> false (close numerically)
//
// Short-circuits on sameNode first, so shared sub-trees cost O(1).
// Constants compare with exact ==; non-finite values cannot enter the
// graph (ExpressionFactory::constant rejects them), so NaN never arises.
bool structurallyEqual(const Expression& lhs, const Expression& rhs);

} // namespace kinemaforge::ik
```

> **Uwaga techniczna (zweryfikowana kompilatorem):** powyższy porządek deklaracji nie jest kwestią stylu — jest wymuszony. Wersja „naturalna" (`class Expression;` → struktury węzłów → wariant → definicja `Expression`) **nie kompiluje się**: `struct AddNode { Expression lhs, rhs; }` wymaga kompletnego `Expression`, a ten wymaga `ExpressionNode`, który wymaga `AddNode`. Jedyne ogniwo, które da się przeciąć, to `shared_ptr` w `Expression` — dlatego `ExpressionNode` musi być deklarowalną z wyprzedzeniem strukturą, a nie aliasem wariantu. Szczegóły i wynik weryfikacji: §6.1a.
>
> Konsekwencja dla kodu klienckiego: dostęp do wariantu to `expression.node().value`, o jeden poziom głębiej niż w obecnym nagłówku.

### 16.2 `src/ik_equations/symbolic/ExpressionFactory.hpp`

```cpp
#pragma once

#include "ik_equations/symbolic/Expression.hpp"

#include <string>

namespace kinemaforge::ik {

// The only way to build a composite expression: Expression's node
// constructor is private and befriends this class.
//
// Applies two kinds of local, O(1) normalization while building:
//   * constant folding — sin(c), c1 + c2, ... collapse to a constant
//   * neutral elements — x + 0 -> x, x * 1 -> x, x / 1 -> x, ...
//
// Both look at the immediate operands only. Anything needing traversal,
// operand reordering or equivalence checking belongs to a future
// EquationSimplifier, not here.
//
// Deliberately NOT applied: x * 0 -> 0. It would erase domain
// information — (1/q) * 0 is undefined at q = 0, but the folded 0
// claims it is defined everywhere. That matters once IK equations start
// carrying variable denominators whose roots mark singularities.
// Folding two known finite constants (c * 0 -> 0) stays, since nothing
// is lost there.
class ExpressionFactory
{
public:
    // Rejects non-finite values: NaN and +/-Inf have no legitimate use
    // here, and keeping them out removes a whole class of comparison
    // questions downstream. Folded results route through this method
    // too, so overflow is caught rather than admitted into the graph.
    // Returns the shared zero node for 0.0.
    Expression constant(double value) const;

    Expression symbol(std::string name) const;

    Expression add(Expression lhs, Expression rhs) const;
    Expression subtract(Expression lhs, Expression rhs) const;
    Expression multiply(Expression lhs, Expression rhs) const;
    Expression divide(Expression lhs, Expression rhs) const;

    Expression negate(Expression operand) const;
    Expression sin(Expression operand) const;
    Expression cos(Expression operand) const;
};

} // namespace kinemaforge::ik
```

### 16.3 `src/ik_equations/symbolic/SymbolicMatrix.hpp`

```cpp
#pragma once

#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/ExpressionFactory.hpp"

#include <array>
#include <cassert>
#include <cstddef>

namespace kinemaforge::ik {

// Fixed-size matrix of symbolic expressions. Cells default to the
// constant 0, all sharing one node, so only the very first matrix in the
// program pays an allocation for its zeros.
//
// Access is operator(), not at(): the bounds check is an assert that
// disappears in release, so borrowing at()'s checked-access connotation
// from the standard library would promise a guarantee this does not give.
//
// Deliberately not a linear algebra library: it carries only what
// composing homogeneous transforms needs.
template <std::size_t Rows, std::size_t Columns>
class SymbolicMatrix
{
    static_assert(Rows > 0, "SymbolicMatrix requires a non-zero row count");
    static_assert(Columns > 0, "SymbolicMatrix requires a non-zero column count");

public:
    static constexpr std::size_t rows = Rows;
    static constexpr std::size_t columns = Columns;

    static SymbolicMatrix zeros() { return SymbolicMatrix{}; }

    static SymbolicMatrix identity() requires (Rows == Columns)
    {
        SymbolicMatrix result;
        const ExpressionFactory factory;
        const Expression one = factory.constant(1.0);   // built once, shared
        for (std::size_t i = 0; i < Rows; ++i)
            result(i, i) = one;
        return result;
    }

    Expression& operator()(std::size_t row, std::size_t column)
    {
        assert(row < Rows && column < Columns);
        return values_[row * Columns + column];
    }

    const Expression& operator()(std::size_t row, std::size_t column) const
    {
        assert(row < Rows && column < Columns);
        return values_[row * Columns + column];
    }

private:
    std::array<Expression, Rows * Columns> values_;
};

// C[i][j] = ((A[i][0]*B[0][j] + A[i][1]*B[1][j]) + ...), left-folded over
// ascending k, so the produced tree shape is deterministic.
//
// Takes the factory explicitly rather than defining operator*: the cost
// stays visible at the call site (one 4x4 product builds up to 112
// nodes), and a factory that later gains configuration can be passed in
// without touching callers.
//
// Note that A * identity() does NOT give back A's cells for symbolic
// input: x * 0 is deliberately kept (see ExpressionFactory), so the
// off-diagonal products survive as Multiply nodes.
template <std::size_t R, std::size_t K, std::size_t C>
SymbolicMatrix<R, C> multiply(
    const SymbolicMatrix<R, K>& lhs,
    const SymbolicMatrix<K, C>& rhs,
    const ExpressionFactory& factory)
{
    SymbolicMatrix<R, C> result;
    for (std::size_t i = 0; i < R; ++i)
    {
        for (std::size_t j = 0; j < C; ++j)
        {
            Expression sum = factory.multiply(lhs(i, 0), rhs(0, j));
            for (std::size_t k = 1; k < K; ++k)
                sum = factory.add(sum, factory.multiply(lhs(i, k), rhs(k, j)));
            result(i, j) = sum;
        }
    }
    return result;
}

} // namespace kinemaforge::ik
```

### 16.4 `src/ik_equations/symbolic/SymbolicTransform.hpp`

**Bez zmian** — obecna treść jest właściwa:

```cpp
#pragma once

#include "ik_equations/symbolic/SymbolicMatrix.hpp"

namespace kinemaforge::ik {

using SymbolicRotation  = SymbolicMatrix<3, 3>;
using SymbolicVector3   = SymbolicMatrix<3, 1>;
using SymbolicTransform = SymbolicMatrix<4, 4>;

} // namespace kinemaforge::ik
```

**Rozstrzygnięcie pytania z promptu (§7 promptu):** `SymbolicTransform` **zostaje aliasem**, nie dostaje osobnego typu-opakowania.

**Korekta uzasadnienia po review.** Pierwsza wersja twierdziła, że inwariantu transformacji jednorodnej („ostatni wiersz = [0 0 0 1]") *nie da się wyegzekwować*. To było za mocne i nieprawdziwe: dałoby się go **utrzymywać** przez kontrolowane konstruktory i zamknięty zestaw operacji (jeśli konstruujemy wyłącznie z poprawnych transformacji, a jedyną operacją jest mnożenie dwóch takich, to własność zachowuje się z konstrukcji — bez potrzeby dowodzenia czegokolwiek o wynikowych wyrażeniach).

Właściwy argument jest inny i słabszy: **obecnie to się nie opłaca.** Taka warstwa oznaczałaby zamknięty typ z własnymi konstruktorami, własnym mnożeniem i barierą przy każdym dostępie zapisującym (`operator()` zwracające referencję modyfikowalną z definicji łamie każdy inwariant). Jedynym dzisiejszym konsumentem jest `JointTransformBuilder`, który buduje macierz komórka po komórce — czyli dokładnie wzorzec, który taki typ musiałby zablokować. Wracamy do tego, jeśli i kiedy pojawi się operacja, która realnie zyskuje na gwarancji (np. odwracanie transformacji, gdzie struktura pozwala uniknąć ogólnego odwracania macierzy).

## 17. Strategia testów

Dwa nowe pliki. **Testy sprawdzają strukturę drzewa, nie tekst** — projekt nie ma dziś żadnego renderera, więc oparcie asercji na serializacji wymagałoby najpierw zbudowania czegoś, czego prompt zakazuje.

### 17.1 `tests/test_symbolic_expression.cpp`

| Test | Sprawdza |
|---|---|
| `DefaultConstructedIsConstantZero` | `type() == Constant`, `isZero()` |
| `CreatesConstant` | `constantValue()` zwraca zadaną wartość |
| `CreatesSymbol` | `type() == Symbol`, nazwa w węźle |
| `CopyingSharesNodeInsteadOfCloning` | `sameNode(original, copy)` |
| `CopiedExpressionOutlivesSource` | źródło wychodzi z zakresu, kopia dalej poprawna |
| `DefaultConstructedInstancesShareOneNode` | dwa `Expression{}` → `sameNode` (potwierdza singleton zera) |
| `BinaryNodeExposesBothOperands` | `Add`/`Subtract`/`Multiply`/`Divide` mają właściwe `lhs`/`rhs` |
| `UnaryNodeExposesOperand` | `Negate`/`Sin`/`Cos` mają właściwy operand |
| `OperandOrderIsPreserved` | `subtract(a, b)` ≠ `subtract(b, a)` strukturalnie |
| `RepeatedOperandIsSharedNotDuplicated` | `multiply(x, x)` → oba dzieci `sameNode` |
| `StructurallyEqualMatchesIdenticalTrees` | dwa niezależnie zbudowane, identyczne drzewa |
| `StructurallyEqualRejectsDifferentShape` | `x+y` vs `x*y` |
| `StructurallyEqualIsNotAlgebraic` | `x+y` vs `y+x` → `false` (świadome udokumentowanie ograniczenia) |
| `SameNodeIsStrongerThanStructuralEquality` | dwie osobne `constant(1.0)`: `structurallyEqual` tak, `sameNode` nie |

### 17.2 `tests/test_symbolic_matrix.cpp`

| Test | Sprawdza |
|---|---|
| `DefaultMatrixCellsAreZero` | wszystkie komórki `isZero()` |
| `DefaultMatrixCellsShareOneNode` | brak alokacji na komórkę |
| `SubscriptReadsAndWritesCell` | zapis przez `operator()`, odczyt tej samej wartości |
| `SubscriptIsRowMajor` | zapis `(0,1)` nie zmienia `(1,0)` |
| `ZerosMatchesDefault` | `zeros()` ≡ konstrukcja domyślna |
| `IdentityHasOnesOnDiagonal` | przekątna `isOne()`, reszta `isZero()` |
| `MultiplyConstantMatrixByIdentityReturnsSameValues` | `A · I` dla `A` ze stałych — zwijanie + `add(x,0)→x` odtwarzają dokładnie `A` |
| `MultiplySymbolicMatrixByIdentityKeepsZeroProducts` | `A · I` dla `A` symbolicznej — wynik to `(a₀₀ + (a₀₁ · 0))`, **nie** `a₀₀`; utrwala granicę z §11.3 |
| `MultiplySmallMatrices` | 2×2, ręcznie zweryfikowany kształt wyniku |
| `MultiplyProducesLeftFoldedSum` | kolejność sumowania po rosnącym `k` |
| `MultiplyPutsLeftOperandFirst` | w `MultiplyNode` `lhs` pochodzi z lewej macierzy |
| `MultiplyFourByFour` | `SymbolicTransform · SymbolicTransform`, wymiary i wybrane komórki |
| `MultiplyOfConstantMatricesFoldsToConstants` | iloczyn dwóch macierzy stałych daje same stałe |
| `IdentityDiagonalSharesOneNode` | wszystkie komórki przekątnej `sameNode` (§12.4) |

### 17.3 `tests/test_expression_factory.cpp`

| Test | Sprawdza |
|---|---|
| `BuildsAddSubtractMultiplyDivide` | właściwy `type()` dla każdej |
| `BuildsNegateSinCos` | j.w. dla jednoargumentowych |
| `FoldsConstantArithmetic` | `add(c2,c3)` → `Constant(5)`, analogicznie dla `-`, `*`, `/` |
| `FoldsConstantTrig` | `sin(constant(0))` → `Constant(0)`, `cos(constant(0))` → `Constant(1)` |
| `FoldsNegateOfConstant` | `negate(constant(2))` → `Constant(-2)` |
| `DropsAdditiveIdentity` | `add(x,0)` i `add(0,x)` → `sameNode` z `x` |
| `DropsSubtractiveIdentity` | `subtract(x,0)` → `x` |
| `DropsMultiplicativeIdentity` | `multiply(x,1)`, `multiply(1,x)` → `x` |
| `DropsDivisionByOne` | `divide(x,1)` → `x` |
| `KeepsSymbolicMultiplicationByZero` | `multiply(sin(q),0)` **pozostaje** węzłem `Multiply` — anihilator odrzucony, dziedzina zachowana (§11.3) |
| `FoldsConstantMultipliedByZero` | `multiply(constant(3),constant(0))` → `Constant(0)` — zwijanie stałych działa dalej |
| `ConstantZeroSharesTheDefaultNode` | `sameNode(constant(0.0), Expression{})` (§14) |
| `DoesNotReorderOperands` | `subtract(a,b)` zachowuje kolejność |
| `DoesNotSimplifyBeyondScope` | `subtract(x,x)` **pozostaje** węzłem `Subtract` (dowód, że nie wchodzimy w analizę równoważności) |
| `TrigFoldingIsExactNotCanonicalized` | `sin(constant(π))` → `Constant(1.22e-16)`, **nie** `Constant(0)` — utrwala decyzję z §11.5 |

**Brak testu błędnego indeksu — zatwierdzone.** Sprawdzenie zakresu to `assert` (§12.3), więc test wymagałby `EXPECT_DEATH`/`ASSERT_DEATH`: mechanizmu działającego tylko bez `NDEBUG`, kapryśnego na Windows/MinGW i nieużywanego dziś nigdzie w tym repo. Kontrakt jest udokumentowany w nagłówku; jego złamanie jest bugiem, nie sytuacją do przetestowania. (Gdyby wybrano wariant A z review — `at()` rzucające `std::out_of_range` — test wróciłby jako zwykły `EXPECT_THROW`; wybrano wariant B.)

**Testy niewykonalne z założenia.** Nie da się napisać testu sprawdzającego, że `Expression{ExpressionNode{...}}` jest niedostępne z zewnątrz — prywatny konstruktor (§10.1) czyni taki kod **niekompilowalnym**, a nie zawodzącym w czasie wykonania. To celowe: inwariant jest egzekwowany przez kompilator, więc test byłby zbędny.

## 18. Wpływ na CMake i strukturę plików

### Nowe pliki

| Plik | Rola |
|---|---|
| `src/ik_equations/symbolic/Expression.cpp` | implementacja `Expression`, predykatów, porównań i węzła zera |
| `src/ik_equations/symbolic/ExpressionFactory.cpp` | implementacja fabryki wraz z normalizacją |
| `tests/test_symbolic_expression.cpp` | §17.1 |
| `tests/test_symbolic_matrix.cpp` | §17.2 |
| `tests/test_expression_factory.cpp` | §17.3 |

### Zmodyfikowane

| Plik | Zmiana |
|---|---|
| `src/ik_equations/symbolic/Expression.hpp` | przeprojektowany (§16.1) |
| `src/ik_equations/symbolic/ExpressionFactory.hpp` | przeprojektowany (§16.2) |
| `src/ik_equations/symbolic/SymbolicMatrix.hpp` | przeprojektowany (§16.3) |
| `CMakeLists.txt` | +2 linie: `Expression.cpp`, `ExpressionFactory.cpp` |
| `tests/CMakeLists.txt` | +3 linie: trzy pliki testowe |

### Bez zmian

`SymbolicTransform.hpp`, `JointTransformBuilder.hpp`, `ForwardKinematicsBuilder.hpp`, `IkEquationBuilder.hpp/.cpp` — nic w nich nie wymaga korekty, bo używają wyłącznie `SymbolicTransform` jako typu, a jego nazwa i domyślna konstruowalność są zachowane.

### Podział header/`.cpp` — rozstrzygnięty (decyzja z review)

Pierwsza wersja zostawiała to pytanie otwarte („do rozważenia") i odsyłała do proposalu implementacyjnego. Słusznie zakwestionowane: to decyzja **architektoniczna** — dotyczy podziału odpowiedzialności i zależności kompilacyjnych — więc należy do tego dokumentu.

Docelowy układ plików:

```
Expression.hpp          deklaracje
Expression.cpp          definicje
ExpressionFactory.hpp   deklaracje
ExpressionFactory.cpp   definicje (normalizacja)
SymbolicMatrix.hpp      header-only (szablon)
SymbolicTransform.hpp   header-only (aliasy)
```

Do `Expression.cpp` trafiają: konstruktor domyślny, konstruktor z `ExpressionNode`, `type()`, `node()`, `isConstant`, `isZero`, `isOne`, `constantValue`, `sameNode`, `structurallyEqual` oraz współdzielony węzeł zera.

Uzasadnienie rozstrzygające: **`structurallyEqual` jest rekurencyjne i nietrywialne** — nie ma powodu, by kompilować je w każdej jednostce translacji dołączającej `Expression.hpp`. To samo dotyczy singletonu zera: `.cpp` daje jedną, oczywistą lokalizację zamiast function-local static w nagłówku.

`SymbolicMatrix` musi zostać header-only, bo jest szablonem — bez tego wracamy dokładnie do dzisiejszego `undefined reference` (§2.7).

## 19. Decyzje — stan po review

### Zatwierdzone bez zmian

| # | Decyzja | Sekcja |
|---|---|---|
| 1 | Nazwy węzłów `*Expression` → `*Node` | §6.1 |
| 2 | Usunięcie baz `BinaryExpression`/`UnaryExpression`, płaskie agregaty | §6.1 |
| 3 | `multiply(lhs, rhs, factory)` jako wolna funkcja zamiast `operator*` | §12.5 |
| 5 | `structurallyEqual` z dokładnym `==` na `double` (jako predykat **strukturalny**, nie numeryczny ani algebraiczny) | §8.3 |
| 7 | Brak `std::expected` w całej warstwie | §13 |
| 8 | `SymbolicTransform` zostaje aliasem (z poprawionym uzasadnieniem) | §16.4 |

### Odrzucone / zmienione w wyniku review

| # | Pierwotna propozycja | Rozstrzygnięcie | Sekcja |
|---|---|---|---|
| 4 | `multiply(x, 0) → 0` | **Odrzucone.** Kasuje informację o dziedzinie — `(1/q)·0` jest nieokreślone dla `q = 0`. Zwijanie `constant·0 → 0` zostaje. Koszt zmierzony: 87 → 375 węzłów. | §11.3, §11.4 |
| 6 | Pominąć test `EXPECT_DEATH` | **Zatwierdzone warunkowo i warunek spełniony** — dostęp oparty na `operator()` + `assert`, więc test odpada. | §12.2, §17.2 |

### Nowe decyzje wprowadzone przez review

| Decyzja | Sekcja |
|---|---|
| Konstruktor `Expression(ExpressionNode)` **prywatny**, `ExpressionFactory` jako `friend` — inwariant fabryki staje się inwariantem modelu, wymuszonym przez kompilator | §10.1, §16.1 |
| Podział header/`.cpp` rozstrzygnięty tu, nie w proposalu implementacyjnym: dochodzi `Expression.cpp` | §18 |
| `at()` → `operator()` — `at()` obiecywałoby sprawdzanie w czasie wykonania, którego nie ma w release | §12.2 |
| `static_assert(Rows > 0)` / `static_assert(Columns > 0)` — zakaz zerowych wymiarów | §12.1 |
| `constant()` odrzuca `NaN`/`±Inf` (`assert(std::isfinite)`), także dla wyników zwijania — usuwa całą klasę pytań o porównywanie `NaN` | §13 |
| `structurallyEqual` zaczyna od `sameNode` (O(1) dla współdzielonych poddrzew) | §8.3 |
| `identity()` buduje jedno wyrażenie `1.0` i współdzieli je na przekątnej | §12.4 |
| `constant(0.0)` zwraca współdzielony węzeł zera zamiast alokować | §14 |
| `sizeof(Expression) == 16 B` opisane jako pomiar dla tej platformy, nie inwariant architektury | §14 |
| „Zero alokacji" dla domyślnej macierzy doprecyzowane — pierwszy węzeł zera kosztuje jedną alokację | §12.1 |

### Nowa odłożona decyzja

**Kanonizacja wyników trygonometrycznych bliskich `0`, `1`, `−1`** (§11.5). `sin(π) = 1.22e-16`, `cos(π/2) = 6.12e-17` — dla `kr4_r600.urdf` oznacza to, że **nawet zatwierdzone reguły** `x + 0 → x` i `x · 1 → x` nie zadziałają na rotacjach z `origin.rpy`. Na tym etapie przyjmujemy dokładny wynik `std::sin`/`std::cos` bez tolerancji, bo próg stałby się częścią semantyki symbolicznej. Do rozstrzygnięcia osobno — również co do etapu (fabryka / `EquationSimplifier` / generator kodu).

## 20. Status dokumentu

Wszystkie punkty z review naniesione. Otwarty pozostaje jeden temat — kanonizacja szumu trygonometrycznego (§19, „Nowa odłożona decyzja") — świadomie **poza** zakresem tego proposalu.

Jeśli powyższe jest do zaakceptowania, kolejny krok to proposal implementacyjny w standardowym formacie repo: pełny kod `Expression.hpp/.cpp`, `ExpressionFactory.hpp/.cpp`, `SymbolicMatrix.hpp`, trzy pliki testowe i zmiany w obu `CMakeLists.txt`.

Żaden plik źródłowy nie został zmodyfikowany. CMake nietknięty. Testy niedodane. Brak commita.
