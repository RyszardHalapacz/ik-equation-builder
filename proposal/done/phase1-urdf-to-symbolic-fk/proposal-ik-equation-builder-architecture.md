# Proposal: `IkEquationBuilder` — architektura

## Prompt

> Ostatni element Fazy 1: `IkEquationBuilder` — fasada. To powinien być mały proposal. Nie dochodzi żadna nowa matematyka. Fasada ma jedynie spiąć istniejące elementy. Najważniejsze decyzje: jawny stan przez `std::optional`, reguły unieważniania stanu, silna gwarancja przy błędzie, jednolity `std::expected`, sposób dostępu do rezultatów.

Zgadzam się z całym kierunkiem. Ten dokument rozstrzyga to, co prompt zostawia otwarte, i dokłada dwie rzeczy, których nie obejmował: §7 (fabryka) i §8 (co z `main.cpp`).

## Status weryfikacji

**Ten dokument nie zawiera kodu produkcyjnego i niczego nie kompilowałem.** Wszystko poniżej wynika z lektury plików w bieżącym stanie repo. Jedno ustalenie z §6.2 **zmienia** to, co fasada może obiecać, więc warto je przeczytać przed resztą.

---

## 1. Cel i zakres

```
URDF → RobotDescription → KinematicChain → SymbolicTransform
```

przez **jedno publiczne API**. Zero nowej matematyki: fasada woła komponenty, które są już zaimplementowane i zweryfikowane numerycznie.

**Poza zakresem:** jakakolwiek zmiana w `UrdfModelLoader`, `KinematicChainBuilder`, `JointTransformBuilder`, `ForwardKinematicsBuilder`, warstwie symbolicznej; nowe funkcje kinematyczne; IK.

## 2. Stan obecny — zweryfikowany w plikach

`IkEquationBuilder.cpp` zawiera **wyłącznie** `IkEquationBuilder::IkEquationBuilder() = default;`. Pięć metod jest zadeklarowanych i niezdefiniowanych; linkuje się tylko dlatego, że `main.cpp` jedynie domyślnie konstruuje obiekt.

Pola są dziś zwykłymi obiektami:

```cpp
RobotDescription robotDescription_;
KinematicChain kinematicChain_;
SymbolicTransform forwardKinematics_;
```

Sygnatury komponentów:

| Komponent | Zwraca | Przy błędzie |
|---|---|---|
| `UrdfModelLoader::load` | `RobotDescription` | **rzuca `std::runtime_error`** |
| `KinematicChainBuilder::build` | `std::expected<KinematicChain, KinematicChainError>` | wartość błędu |
| `ForwardKinematicsBuilder::build` | `SymbolicTransform` | nie może zawieść |

`KinematicChainError` to enum czterowartościowy: `BaseLinkNotFound`, `ToolLinkNotFound`, `NoPathFound`, `InvalidRobotDescription`.

---

## 3. Jawny stan przez `std::optional`

Przyjęte bez zastrzeżeń:

```cpp
std::optional<RobotDescription> robotDescription_;
std::optional<KinematicChain>   kinematicChain_;
std::optional<SymbolicTransform> forwardKinematics_;
```

Powód, który wart jest zapisania precyzyjniej niż „pusty obiekt wygląda jak wynik": **domyślnie skonstruowany `SymbolicTransform` to macierz samych zer** — nawet nie jednostkowa. Gdyby fasada zwróciła go po nieudanej sekwencji wywołań, wołający dostałby macierz, która jest cichym, geometrycznie bezsensownym wynikiem, a nie oczywistą pustką. To ten sam rodzaj cichego błędu, który cały projekt konsekwentnie eliminuje.

Cztery rozróżnialne stany: brak modelu → model → chain → FK.

## 4. Reguły unieważniania

```
loadRobotModel  (sukces) → nowy model, kasuje chain, kasuje FK
selectChain     (sukces) → nowy chain, kasuje FK
buildForwardKinematics (sukces) → nowe FK
```

Przyjęte. Uzasadnienie w jednym zdaniu: `KinematicChain` niesie nazwy linków i offsety **konkretnego** robota, a FK niesie symbole **konkretnego** łańcucha — każdy przeżywający element byłby odpowiedzią na nieaktualne pytanie.

## 5. Gwarancja transakcyjna

Przyjęte: budujemy wynik lokalnie, podmieniamy pola **dopiero po sukcesie**.

```
nieudany load        → poprzedni stan nietknięty
nieudany selectChain → poprzedni stan nietknięty
zła kolejność wywołań→ stan nietknięty
```

To wymaga dyscypliny w jednym miejscu: `loadRobotModel` musi najpierw wywołać loader do zmiennej lokalnej, a dopiero po powrocie bez wyjątku wyzerować `kinematicChain_` i `forwardKinematics_`. Odwrotna kolejność — wyczyścić, potem ładować — daje po nieudanym ładowaniu obiekt bez modelu **i** bez poprzedniego łańcucha, czyli gorszy niż przed wywołaniem.

---

## 6. Model błędów

### 6.1 Jednolity `std::expected` — przyjęte

```cpp
enum class IkEquationBuilderErrorCode
{
    RobotModelNotLoaded,
    KinematicChainNotSelected,
    UrdfLoadFailed,
    ChainBuildFailed
};

struct IkEquationBuilderError
{
    IkEquationBuilderErrorCode code{};
    std::string message;
    std::optional<KinematicChainError> chainError;
};

std::expected<void, IkEquationBuilderError> loadRobotModel(const std::filesystem::path&);
std::expected<void, IkEquationBuilderError> selectChain(const std::string& baseLink,
                                                        const std::string& toolLink);
std::expected<void, IkEquationBuilderError> buildForwardKinematics();
```

Fasada jest **granicą publiczną modułu**, więc to ona ma prawo narzucić jeden styl. Wyjątek loadera zostaje przechwycony i zamieniony na `UrdfLoadFailed`; wołający nie musi wiedzieć, że w środku ktoś rzuca.

Zachowanie `chainError` jako osobnego, **typowanego** pola jest ważne: `NoPathFound` i `BaseLinkNotFound` to dla wołającego różne sytuacje, a rozróżnianie ich przez parsowanie stringa byłoby regresem wobec tego, co `KinematicChainBuilder` już daje.

### 6.2 Ustalenie, które zawęża obietnicę: typowany błąd loadera **już nie istnieje**

Sprawdziłem `UrdfModelLoader.cpp`. Strukturalny `mt::kinematics::LoadError` z `LoadErrorCode` jest **konsumowany na granicy loadera** — funkcja `describe()` składa z niego komunikat, po czym leci `throw std::runtime_error(...)`. Poza loader wychodzi **wyłącznie string**.

Konsekwencja: `IkEquationBuilderError` przy `UrdfLoadFailed` może nieść tylko `message`. **Nie ma czego zachować**, bo kod błędu zginął piętro niżej.

Symetria z `chainError` jest więc pozorna: chain builder oddaje typowany błąd, loader nie. Zapisuję to jawnie, żeby nikt nie czytał braku `loadError` w strukturze jako przeoczenia.

**Nie proponuję tego naprawiać w tym proposalu.** Typowany błąd loadera to zmiana w `UrdfModelLoader` — komponentu spoza zakresu fasady — i zasługuje na własny dokument. Do zapisania w `STATUS.md` jako znana luka obok istniejącego wpisu o odrzucanym `DiagnosticBag`.

### 6.3 Co z wyjątkami, których fasada się nie spodziewa

`loadRobotModel` łapie `std::runtime_error` (albo `std::exception`) z loadera. **Nie** łapie wszystkiego bez różnicy: `std::bad_alloc` czy błąd programisty powinny lecieć dalej, a nie zamieniać się w `UrdfLoadFailed` z mylącym komunikatem. Do rozstrzygnięcia w proposalu implementacyjnym, czy łapiemy `std::exception`, czy węziej — rekomendacja: `std::exception`, bo loader nie dokumentuje typu, ale **nie** `catch (...)`.

---

## 7. Fabryka wyrażeń — świadomie **nie** przewodzimy jednej instancji

Kuszące jest, żeby fasada stworzyła jeden `ExpressionFactory` i wstrzyknęła go do `JointTransformBuilder` i `ForwardKinematicsBuilder` — oba mają konstruktory, które to przyjmują.

**Nie robimy tego, i to jest decyzja, nie zaniedbanie.** Oba konstruktory biorą fabrykę **przez wartość**, więc przekazanie tego samego obiektu **i tak da dwie kopie**. Dziś jest to bez znaczenia (klasa nie ma pól), ale gdyby fabryka kiedyś zyskała stan, taki kod wyglądałby na współdzielenie, którym nie jest — a to gorsze niż jawny brak współdzielenia, bo tworzy fałszywe poczucie bezpieczeństwa.

Zostawiamy konstrukcję domyślną i wpis w `STATUS.md` („Factory ownership is value-semantic"), który mówi prawdę. Prawdziwe współdzielenie wymaga przeprojektowania własności fabryki — osobna zmiana, gdy zajdzie potrzeba.

## 8. Trzy drobne rozstrzygnięcia

**Kopiowanie i przenoszenie:** domyślne. Fasada to worek na `optional` plus bezstanowe buildery; kopia jest sensownym snapshotem, w przeciwieństwie do `ExpressionEvaluator`, gdzie kopiowanie cache'u sesji było mylące. Nic nie deklarujemy jawnie.

**Wątkowość:** niebezpieczna dla współbieżnych modyfikacji, jak każdy obiekt ze stanem. Jedno zdanie w komentarzu klasy, bez mechanizmów.

**`main.cpp`:** dziś tylko konstruuje obiekt i drukuje komunikat. Skoro README przedstawia fasadę jako docelowy punkt wejścia, sensowne jest, żeby `main` faktycznie przeszedł ścieżkę URDF → FK na jednym z plików z `data/urdf`. **Proponuję to zrobić** — jest to kilka linii, a bez tego jedyny „program" projektu nadal nie demonstruje niczego. Do rozstrzygnięcia w review; jeśli odpadnie, `main.cpp` zostaje bez zmian.

---

## 9. Dostęp do rezultatów — rozstrzygnięcie

Trzy warianty:

| Wariant | Za | Przeciw |
|---|---|---|
| **A.** `const KinematicChain* kinematicChain() const noexcept` | prosty; `nullptr` odpowiada dokładnie na pytanie „czy jest?" | surowy wskaźnik w nowoczesnym API |
| **B.** `std::expected<std::reference_wrapper<const T>, Error>` | spójny z metodami mutującymi | `->value().get()` przy każdym użyciu; **błąd nie niesie żadnej informacji**, bo powód nieobecności jest zawsze jeden |
| **C.** `const std::optional<KinematicChain>& kinematicChain() const noexcept` | bez wskaźników; odzwierciedla stan wewnętrzny 1:1 | zwraca referencję do składowej |

**Rekomendacja: A.**

Argument przeciw B jest merytoryczny, nie estetyczny: `kinematicChain()` może być nieobecny z **dokładnie jednego** powodu — nie wywołano `selectChain`. `std::expected` istnieje po to, żeby przenieść informację *dlaczego*; gdy dlaczego jest jedno, degeneruje się do droższego `bool` z obowiązkową ceremonią po stronie wołającego.

Mieszanie `expected` (operacje) z wskaźnikiem (stan) nie jest niespójnością — to dwa różne pytania. „Czy ta operacja się powiodła i dlaczego nie" oraz „czy ten element stanu istnieje" zasługują na różne odpowiedzi.

Wariant C jest bliski i do przyjęcia, jeśli review woli zero wskaźników. **Nie** proponuję mieszać A i C między akcesorami — jeden model, konsekwentnie.

---

## 10. Plan testów — 17 pozycji

15 z promptu plus dwie dołożone.

### 10.1 Ścieżka szczęśliwa (5)

`LoadsRobotModel`, `SelectsChainAfterLoadingModel`, `BuildsForwardKinematicsEndToEnd`, `BuildsKr4ThroughFacade`, `BuildsKr640ThroughFacade`

Test end-to-end ma dokładnie odpowiadać README:

```cpp
IkEquationBuilder builder;
ASSERT_TRUE(builder.loadRobotModel(path));
ASSERT_TRUE(builder.selectChain("base_link", "tool0"));
ASSERT_TRUE(builder.buildForwardKinematics());
ASSERT_NE(builder.forwardKinematics(), nullptr);
```

### 10.2 Zła kolejność (4)

`RejectsChainSelectionBeforeLoadingModel`, `RejectsForwardKinematicsBeforeSelectingChain`, `RejectsChainAccessBeforeSelection`, `RejectsForwardKinematicsAccessBeforeBuild`

Każdy sprawdza **kod błędu**, nie tylko fakt niepowodzenia — `RobotModelNotLoaded` i `KinematicChainNotSelected` to różne diagnozy.

### 10.3 Unieważnianie (3)

`LoadingNewRobotClearsChain`, `LoadingNewRobotClearsForwardKinematics`, `SelectingNewChainClearsForwardKinematics`

### 10.4 Zachowanie stanu przy błędzie (3)

`FailedLoadPreservesPreviousState`, `FailedChainSelectionPreservesPreviousState`, `PropagatesChainBuilderError`

Ostatni sprawdza, że `chainError` niesie **konkretną** wartość — np. `NoPathFound` dla `flange → tool0` w KR4, co jest już pinowane na poziomie chain buildera i tutaj musi przejść przez fasadę bez spłaszczenia.

### 10.5 Dołożone (2)

`ReportsUrdfLoadFailure` — nieistniejąca ścieżka daje `UrdfLoadFailed` i **niepusty `message`**. Bez tego jedyna informacja, jaka zostaje z błędu loadera (§6.2), nie jest niczym przypięta.

`ReusesFacadeForSecondRobot` — pełny przebieg dla KR4, potem pełny przebieg dla KR640 na tej samej instancji, z porównaniem, że FK **różni się**. Testy z §10.3 sprawdzają każdą regułę unieważniania osobno; ten sprawdza, że ich złożenie daje poprawny obiekt, a nie mieszankę dwóch robotów. To najbardziej prawdopodobny realny sposób użycia fasady i jednocześnie najłatwiejszy do zepsucia.

**Razem: 17 nowych. Oczekiwany stan: 204 + 17 = 221.**

---

## 11. Plan zmian w plikach

**Zmienione:** `src/ik_equations/IkEquationBuilder.hpp` (typy błędów, `optional`, sygnatury), `IkEquationBuilder.cpp` (pięć definicji), `tests/CMakeLists.txt` (jedna linia), `STATUS.md`, opcjonalnie `main.cpp` (§8).

**Dodane:** `tests/test_ik_equation_builder.cpp`.

**Bez zmian — jawnie:** wszystkie komponenty potokowe i cała warstwa symboliczna. Główny `CMakeLists.txt` też nie, bo `IkEquationBuilder.cpp` już jest na liście źródeł.

---

## 12. Do rozstrzygnięcia w review

1. **§9 — wariant akcesorów.** Rekomendacja: A (`const T*`). Argument: powód nieobecności jest zawsze jeden, więc `expected` nie niesie informacji.
2. **§8 — czy `main.cpp` ma demonstrować pełną ścieżkę.** Rekomendacja: tak.
3. **§6.3 — co łapiemy z loadera.** Rekomendacja: `std::exception`, nie `catch (...)`.

**Rozstrzygnięte, niewymagające decyzji:** `optional` na stanie, reguły unieważniania, gwarancja transakcyjna, jednolity `std::expected`, typowany `chainError`, brak przewodzenia fabryki (§7), domyślne kopiowanie i przenoszenie.

## 13. Rekomendacja końcowa

Zatwierdzić w kształcie: **stan przez `std::optional`, kaskadowe unieważnianie, podmiana pól dopiero po sukcesie, jednolity `std::expected` z czterema kodami i zachowanym typowanym `chainError`, akcesory zwracające wskaźnik.**

Jedno zastrzeżenie do zapamiętania przy zamykaniu Fazy 1: fasada **nie zmienia tego, co projekt może o sobie powiedzieć**. Ona pakuje. Wszystko, co Faza 1 udowodniła, zostało udowodnione wcześniej — i dwa ograniczenia zakresu zapisane w `STATUS.md` (walidacja zaczyna się od `KinematicChain`; konwencja RPY oparta na wspólnym przekonaniu) obowiązują po niej tak samo jak przed.

Po zatwierdzeniu: `proposal-ik-equation-builder-implementation.md` z pełnym kodem.
