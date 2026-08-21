# Proposal: polityka ostrzeżeń kompilatora (R1, v3)

## Prompt

> Review kodu, uwagi 8 i 9: CMake nie uruchamia mechanizmu ostrzeżeń, na którym
> kod twierdzi, że polega. Przyszła obsługa `continuous` ma niebezpieczną
> pułapkę — `mapJointType` z fallbackiem `return JointType::Fixed`.

Etap `F2.3.R`, dokument **R1**. Jeden dokument, nie para architektura +
implementacja — zakres jest mechaniczny, a decyzje są nieliczne i mieszczą się
przy kodzie. Precedens: `../proposal-loader-test-coverage.md`.

## Historia wersji

**v1 → v2** po review (verdykt: *approve with changes*). Przyjęte wszystkie
cztery poprawki plus doprecyzowanie:

| # | Poprawka z review | Status |
|---|---|---|
| 1 | warningi na targetach, nie `add_compile_options` | przyjęte — argument okazał się mocniejszy, patrz 5.1 |
| 2 | `mapJointType` bez `std::optional` — `throw` wprost w mapperze | przyjęte, v1 było gorsze |
| 3 | test forged enum przez fasadę jest niewykonalny | przyjęte, wraz z konsekwencją której review nie wyciąga — patrz 6 |
| 4 | `\.` w diagramie → przerysować linie | przyjęte, 5.2 |
| 5 | doprecyzować, że `logic_error` nie „przerywa programu" | przyjęte, 4.1 |

Wycofane z v1: `std::optional<JointType>`, nowy nagłówek
`detail/JointTypeMapping.hpp`, dwa testy runtime.

**v2 → v3** po review (verdykt: *approve with one required correction*):

| # | Poprawka z review | Status |
|---|---|---|
| 6 | gałąź MSVC nie zapewniała deklarowanej ochrony — `/W4` **nie** włącza C4062 | przyjęte jako **wymagane**, patrz 5.1; to podważało główny kontrakt R1 |
| 7 | jawne `-Wswitch` po stronie GCC/Clang, mimo że wchodzi przez `-Wall` | przyjęte, 5.1 |
| 8 | gate miał niepoprawną komendę CMake i rekonfigurował istniejący `build` | przyjęte, i powód jest głębszy niż literówka — patrz 10 |

## Status weryfikacji

Wszystko poniżej odtworzone na obecnym `main` (`9d2ceb2`), nie wywnioskowane z
lektury.

```text
g++ -std=c++23 -Wall -Wextra -Werror -fsyntax-only  na 16 plikach rdzenia
    → dokładnie 2 błędy, nic więcej
    → exit code 1

grep -n '^\s*//.*\\\s*$' po całym src/
    → dokładnie 1 trafienie (IkEquationBuilder.hpp:57)
```

## 1. Cel i zakres

**W zakresie:**

1. Włączyć `-Wall -Wextra` na targetach KinemaForge i `-Werror` w CI.
2. Naprawić dwa istniejące naruszenia, które dziś blokują (1).
3. Utwardzić `mapJointType` tak, żeby dodanie `continuous` do dolnego enuma nie
   mogło przejść po cichu.

**Poza zakresem:** jakakolwiek zmiana logiki, geometrii czy API poza punktem 3;
plik workflow CI — ten proposal ustala, *co* CI ma wymuszać, nie w jakim
narzędziu; `continuous` jako funkcjonalność (osobny proposal).

**Dlaczego R1 jest pierwsze w etapie**, wbrew kolejności z review (tam FIX-06):
to dwie jednoliniowe poprawki plus konfiguracja, a jednocześnie mechanizm
ochronny dla R2–R5. Zrobione na końcu oznaczałoby, że cała remediacja pisze się
bez ochrony, którą właśnie wprowadza. Review potwierdziło tę kolejność.

## 2. Stan obecny — zweryfikowany w plikach

### 2.1 CMakeLists.txt nie ustawia żadnych ostrzeżeń

`CMakeLists.txt` ma `CMAKE_CXX_STANDARD 23`, `add_link_options` dla MinGW i
`FetchContent` dla pugixml. **Nie ma `add_compile_options` ani
`target_compile_options` w ogóle.**

### 2.2 Kod powołuje się na ochronę, której nie ma

`src/ik_equations/model/IkEquationSystem.cpp:133`:

```cpp
// No default: a new IkTaskKind trips -Wswitch, which -Werror turns into a
// build failure. Each valid case returns, so a value forged with
// static_cast<IkTaskKind>(99) falls through to the rejection below instead
// of quietly succeeding -- the two protections cover different things and
// neither replaces the other.
```

Zdanie o `-Wswitch`/`-Werror` jest dziś **nieprawdziwe w standardowym buildzie**.
Druga połowa komentarza (jawne odrzucenie podrobionej wartości) jest prawdziwa i
działa — to ta część, która faktycznie chroni.

Sprawdzone zachowanie GCC: bez flag niepełny `switch` po enumie nie daje
ostrzeżenia; z `-Wall` daje `[-Wswitch]`.

### 2.3 Dwa naruszenia blokujące

**(a) `src/ik_equations/IkEquationBuilder.hpp:57`** — linia diagramu w
komentarzu klasy kończy się backslashem, czyli kontynuacją linii:

```cpp
//                /          \
```

Skutku runtime nie ma (następna linia też jest komentarzem), ale:

```text
error: multi-line comment [-Werror=comment]
```

To **publiczny nagłówek**, ten, który ma włączać MotionBridge. Każdy konsument z
`-Wall` dostanie ostrzeżenie z naszego API.

**(b) `src/ik_equations/builders/KinematicChainBuilder.cpp:59`**

```cpp
stack.push_back(Frame{baseLink});
```

`Frame` ma trzy pola; pozostałe dwa mają inicjalizator domyślny
(`nextChildIndex = 0`) lub są `std::optional`, więc zachowanie jest poprawne —
brakujące pole jest value-initialized. Ale:

```text
error: missing initializer for member '...Frame::incomingJointIndex'
       [-Werror=missing-field-initializers]
```

### 2.4 Pułapka `mapJointType`

`src/ik_equations/UrdfModelLoader.cpp:12`:

```cpp
JointType mapJointType(mt::kinematics::JointType type)
{
    switch (type)
    {
    case mt::kinematics::JointType::revolute:  return JointType::Revolute;
    case mt::kinematics::JointType::prismatic: return JointType::Prismatic;
    case mt::kinematics::JointType::fixed:      return JointType::Fixed;
    }
    return JointType::Fixed;
}
```

Dolny enum `mt::kinematics::JointType` ma `revolute`, `prismatic`, `fixed`.
Górny `kinemaforge::ik::JointType` ma dodatkowo `Continuous`.

Gdy `continuous` zostanie dodane do dolnego enuma (a jest na liście prac) i ktoś
zapomni dopisać `case`, przy obecnym CMake: zero ostrzeżeń, zero błędów
kompilacji, **`continuous → Fixed`**. Robot się załaduje, a joint przestanie
istnieć jako stopień swobody. Dokładnie ten rodzaj cichej porażki, którego cały
projekt konsekwentnie unika.

## 3. Decyzja: `-Werror` tylko w CI

| Wariant | Skutek |
|---|---|
| A. `-Werror` wszędzie | nowa wersja kompilatora, nowe ostrzeżenie → **nie da się zbudować projektu**, dopóki ktoś nie posprząta |
| B. `-Werror` tylko w CI | lokalnie widać ostrzeżenia i można je zignorować w trakcie pracy; do `main` nie wejdzie kod, który je łamie |
| C. brak `-Werror` | ostrzeżenia stają się tłem, którego nikt nie czyta |

**Wybór: B**, potwierdzony w review. Argument przeciw A jest konkretny, nie
hipotetyczny: projekt kompiluje się MinGW-em, którego wersja na maszynie
deweloperskiej zmienia się niezależnie od repo. Wariant A wiąże możliwość pracy z
wersją narzędzia.

## 4. Decyzja: `mapJointType` — sygnatura **zostaje**, dochodzi `throw`

> **Zmiana wobec v1.** v1 twierdziło „sygnatura musi się zmienić" i proponowało
> `std::optional<JointType>` plus sprawdzenie u wołającego. Review słusznie to
> odrzuciło i v2 przyjmuje wariant prostszy.

Samo usunięcie `return JointType::Fixed` nie wystarczy — bez niego funkcja z
niewyczerpanym `switch` „spada z końca", a to UB dla funkcji zwracającej wartość.
Ale nie potrzeba do tego `optional`:

```cpp
JointType mapJointType(mt::kinematics::JointType type)
{
    switch (type)
    {
    case mt::kinematics::JointType::revolute:  return JointType::Revolute;
    case mt::kinematics::JointType::prismatic: return JointType::Prismatic;
    case mt::kinematics::JointType::fixed:     return JointType::Fixed;
    }

    throw std::logic_error(
        "UrdfModelLoader: unmapped mt::kinematics::JointType value " +
        std::to_string(static_cast<int>(type)));
}
```

Cztery własności naraz:

```text
brak default        → nowy enumerator uruchamia -Wswitch → -Werror w CI → build fail
wartość podrobiona  → logic_error
brak optional       → wołający nie może zapomnieć sprawdzić
brak UB             → każda ścieżka albo zwraca, albo rzuca
```

Dlaczego to jest **lepsze**, a nie tylko krótsze: `std::optional` w tej pozycji
komunikuje „spodziewana porażka domenowa, obsłuż ją". A ten przypadek oznacza
defekt programu, nie błąd danych użytkownika — `throw` wprost w mapperze mówi
dokładnie to. v1 zauważało tę asymetrię w tekście i mimo to proponowało typ,
który jej przeczy.

Konsekwencja: `mapJoint` **nie zmienia się wcale** poza jedną linią, która i tak
już tam jest (`result.type = mapJointType(joint.type);`). Znika też potrzeba
nagłówka `detail/JointTypeMapping.hpp` z v1 — `mapJointType` zostaje w anonimowej
przestrzeni nazw, gdzie należy.

### 4.1 Co dokładnie robi ten `logic_error`

> **Doprecyzowanie z review.** v1 pisało, że wyjątek „przerwie program". To
> nieprecyzyjne.

`IkEquationBuilder` łapie **dokładnie** `std::runtime_error`, celowo nie
`std::exception` (żeby nie zamienić `std::bad_alloc` w błąd URDF-a).
`std::logic_error` nie dziedziczy po `std::runtime_error` — to dwie równoległe
gałęzie pod `std::exception`. Zatem:

```text
logic_error
  → przechodzi przez IkEquationBuilder bez złapania
  → zostaje przekazany wołającemu
  → std::terminate DOPIERO jeśli nikt go nie złapie
```

To jest zamierzone. Cicha konwersja błędu programisty na `UrdfLoadFailed` byłaby
zaraportowaniem naszej pomyłki jako problemu z plikiem użytkownika.

Nowy include: żaden. `<stdexcept>` i `<string>` są już w
`UrdfModelLoader.cpp:5–6`.

### 4.2 `describeCode` — ten sam kształt, ale fallback zostaje

`UrdfModelLoader.cpp:48` ma identyczną strukturę: wyczerpujący `switch` po
`LoadErrorCode` plus `return "unknown URDF load error";`. Tutaj fallback jest
**akceptowalny i zostaje**: funkcja produkuje tekst diagnostyczny, nie decyzję
semantyczną. Zwrócenie ogólnego napisu dla nieznanego kodu nie gubi stopnia
swobody robota. `-Wswitch` i tak zadziała po dodaniu kodu błędu, bo `switch` nie
ma `default`.

Reguła, którą to ilustruje: fallback jest zły wtedy, gdy podstawia *milcząco
poprawnie wyglądającą semantykę*. Dla komunikatu — nie podstawia.

Zapisane jawnie, bo to odstępstwo od „bez cichych fallbacków" i chcę je mieć
zaakceptowane, nie przemilczane.

## 5. Proponowane zmiany

### 5.1 `CMakeLists.txt` — warningi na targetach

> **Zmiana wobec v1.** v1 proponowało `add_compile_options` z uwagą „zacząć
> globalnie i sprawdzić, czy gtest hałasuje". Review odrzuciło tę strategię i
> **miało mocniejszy argument, niż v1 zakładało.**

Sprawdzone: `tests/CMakeLists.txt:9` woła `FetchContent_MakeAvailable(googletest)`
**wewnątrz** katalogu `tests/`, do którego `add_subdirectory(tests)` z korzenia
propaguje flagi katalogowe. Czyli googletest dostałby nasze warningi **z
konstrukcji**, nie przez przypadek. `pugixml` byłby bezpieczny tylko dlatego, że
jego `FetchContent_MakeAvailable` stoi w korzeniu wyżej niż proponowany blok — to
szczęście wynikające z kolejności linii, nie projekt.

Rozwiązanie: target `INTERFACE` z polityką, podpinany `PRIVATE` do własnych
targetów.

```cmake
# --- Warning policy -----------------------------------------------------
#
# Carried by an INTERFACE target rather than add_compile_options, so it reaches
# exactly the targets that opt in. Directory-scoped options would also land on
# FetchContent dependencies -- googletest is made available inside
# tests/CMakeLists.txt, which add_subdirectory(tests) pulls into this
# directory's option scope. Their warnings are not ours to fix, and a target
# whitelist makes that a property of the design rather than of line order.
#
# -Wall -Wextra are always on: several comments in this codebase state that a
# new enumerator "trips -Wswitch", and that claim has to be true.
#
# -Werror is opt-in through KINEMAFORGE_WERROR, which CI sets. Unconditionally
# would tie the ability to build this project to the compiler version installed
# on a developer machine: a newer GCC adding one new warning would stop all work
# until someone cleans up. In CI that tradeoff is correct, locally it is not.
option(KINEMAFORGE_WERROR "Treat compiler warnings as errors (used by CI)" OFF)

add_library(kinemaforge_warnings INTERFACE)

if (MSVC)
    # /w14062 is REQUIRED, not decoration. C4062 ("enumerator not handled in
    # switch, and there is no default label") is a level-4 warning that
    # Microsoft ships OFF BY DEFAULT, so /W4 alone does not enable it -- and
    # C4062 is precisely the diagnostic this whole document exists to
    # guarantee. Without it the MSVC build would silently accept a new
    # `continuous` enumerator with no case, which is the exact regression R1
    # is meant to make impossible. /w1<nnnn> promotes the warning to level 1
    # so it is reported at any /W level.
    #
    # C4061 is the same diagnostic for a switch that DOES have a default and
    # still misses enumerators, also off by default. This codebase's house
    # style is "no default", so C4061 fires only where that style is already
    # broken -- which makes it worth having.
    target_compile_options(kinemaforge_warnings INTERFACE
        /W4
        /w14062
        /w14061
    )
    if (KINEMAFORGE_WERROR)
        target_compile_options(kinemaforge_warnings INTERFACE /WX)
    endif()
else()
    # -Wswitch is listed explicitly even though -Wall already implies it. This
    # particular diagnostic is part of an architectural contract stated in
    # source comments (IkEquationSystem.cpp, UrdfModelLoader.cpp), so it is
    # named here rather than left to arrive as a side effect of a bundle whose
    # contents are a compiler-version detail.
    target_compile_options(kinemaforge_warnings INTERFACE
        -Wall
        -Wextra
        -Wswitch
    )
    if (KINEMAFORGE_WERROR)
        target_compile_options(kinemaforge_warnings INTERFACE -Werror)
    endif()
endif()
```

**Asymetria wagi między gałęziami, powiedziana wprost.** Do v2 gałąź GCC/MinGW
dawała:

```text
-Wall → -Wswitch → brakujący case → warning → -Werror w CI → build fail
```

a gałąź MSVC:

```text
/W4 → C4062 nadal OFF → brakujący case → brak warningu
    → /WX nie ma czego zamienić w błąd → build PRZECHODZI
```

Czyli dokument obiecywał ochronę, której na jednym z kompilatorów nie dostarczał.
To ten sam kształt błędu, który R1 naprawia w `mapJointType` — deklarowane
zabezpieczenie bez pokrycia w mechanizmie. Poprawka jest wymagana, nie
kosmetyczna.

Podpięcie — `PRIVATE`, w trzech miejscach:

```cmake
target_link_libraries(kinemaforge_ik PUBLIC pugixml::pugixml)
target_link_libraries(kinemaforge_ik PRIVATE kinemaforge_warnings)

target_link_libraries(${PROJECT_NAME} PRIVATE kinemaforge_ik kinemaforge_warnings)
```

oraz w `tests/CMakeLists.txt`:

```cmake
target_link_libraries(kinemaforge_tests PRIVATE
    kinemaforge_ik GTest::gtest_main kinemaforge_warnings)
```

Trzy własności tego wariantu:

- **`PRIVATE`, nie `PUBLIC`.** Nasza polityka ostrzeżeń nie jest częścią
  kontraktu `kinemaforge_ik` — MotionBridge ma używać swojej.
- **Kolejność linii przestaje mieć znaczenie.** Definicja może stać gdziekolwiek
  przed pierwszym `target_link_libraries`; nie ma pułapki „blok wstawiony za
  wysoko obejmuje zależności".
- **Lista jest jawną białą listą.** Nowy target bez tej linii nie dostaje
  ostrzeżeń — co jest wadą wymagającą dyscypliny, ale wadą widoczną, w
  przeciwieństwie do cichego objęcia zależności.

`kinemaforge_warnings` jest zdefiniowane w korzeniu, a `add_subdirectory(tests)`
stoi na końcu, więc target jest widoczny w `tests/` bez dodatkowych zabiegów.

### 5.2 `src/ik_equations/IkEquationBuilder.hpp` — przerysowanie diagramu

> **Zmiana wobec v1.** v1 proponowało dopisanie kropki (`\.`), żeby zminimalizować
> diff. Review: „wygląda jak przypadkowa kropka", i to jest publiczna
> dokumentacja klasy. Zgoda — czytelność wygrywa z rozmiarem diffu.

Dziś, linie 54–60:

```cpp
//              RobotDescription
//                     |
//               KinematicChain
//                /          \
//     ForwardKinematics    TCP configuration
//                \          /
//              TcpForwardKinematics
```

Po zmianie — żadna linia nie kończy się backslashem, rozwidlenie i scalenie
czytają się wyraźniej niż na skosach, całość pozostaje czystym ASCII (nagłówek
publiczny, nie chcę wprowadzać znaków rysunkowych Unicode):

```cpp
//              RobotDescription
//                     |
//               KinematicChain
//                 +---+---+
//                 |       |
//     ForwardKinematics   TCP configuration
//                 |       |
//                 +---+---+
//                     |
//              TcpForwardKinematics
```

Zasięg jest jednomiejscowy i sprawdzony, nie założony: `grep -n '^\s*//.*\\\s*$'`
po całym `src/` zwraca **dokładnie jedno** trafienie. Drugi diagram w tym samym
pliku kończy linie znakami `|` i `/`, więc problemu nie ma.

### 5.3 `src/ik_equations/builders/KinematicChainBuilder.cpp:59`

```cpp
// Root frame: no children visited yet, and no incoming joint.
stack.push_back(Frame{baseLink, 0, std::nullopt});
```

Jawne `std::nullopt` jest wartościowe niezależnie od ostrzeżenia — komentarz przy
definicji `Frame` mówi „nullopt only for the root frame", a to jedyne miejsce,
gdzie ten korzeń powstaje.

### 5.4 `src/ik_equations/UrdfModelLoader.cpp:12`

Ciało z sekcji 4, plus komentarz kontraktowy:

```cpp
// Two protections, covering different things:
//
//   * no `default` -- adding an enumerator to mt::kinematics::JointType makes
//     -Wswitch fire, which -Werror turns into a build failure in CI. This is
//     what must stop `continuous` from silently mapping to Fixed once the
//     lower layer learns about it.
//   * the throw -- a value forged with static_cast has no enumerator to be
//     caught at compile time. std::logic_error, not a LoadErrorCode: the lower
//     layer's parse_joint_type already rejected every string it does not know,
//     so a value arriving here is a defect in this repository, not in the
//     user's file. logic_error does NOT derive from std::runtime_error, which
//     is exactly what IkEquationBuilder catches, so it propagates to the caller
//     instead of being reported as a URDF problem.
//
// Neither protection replaces the other. Same shape as componentIndex() in
// IkEquationSystem.cpp, which rejects rather than throws because there the
// caller has a std::expected to carry the error.
```

Przy okazji: wyrównana spacja w `case ...fixed:`, która dziś odstaje o jeden znak.

## 6. Weryfikacja — i uczciwe stwierdzenie, czego nie da się przetestować

> **Zmiana wobec v1.** v1 planowało dwa testy runtime. Review wykazało, że drugi
> jest niewykonalny bez sztucznego seamu w produkcyjnym API. **Konsekwencja,
> której review nie wyciąga do końca: pierwszy jest niewykonalny z tego samego
> powodu.** Po wycofaniu nagłówka `detail/` z v1 `mapJointType` zostaje w
> anonimowej przestrzeni nazw, więc test go nie widzi.

Publiczna ścieżka do `mapJointType` jest jedna:

```text
plik URDF → parse_joint_type() → tylko trzy legalne enumeratory
          → RobotModel → UrdfModelLoader::load → mapJoint → mapJointType
```

`parse_joint_type` przepuszcza wyłącznie `"revolute"`, `"prismatic"`, `"fixed"`.
**Nie istnieje publiczny punkt wstrzyknięcia** wartości `JointType(99)`. Zatem
`throw` z sekcji 4 jest nieosiągalny przez jakiekolwiek publiczne API — i to jest
własność, nie luka. Jest to obrona w głąb na wypadek, gdyby przyszła zmiana
dodała ścieżkę omijającą `parse_joint_type`.

Nie piszemy więc testu runtime na tę ścieżkę i **nie przebudowujemy produkcyjnego
API, żeby taki test umożliwić.** To byłoby dokładnie odwrócenie zależności:
projekt kodu podporządkowany możliwości napisania testu na przypadek, który przez
konstrukcję nie może wystąpić.

### 6.1 Właściwa weryfikacja jest kompilacyjna

Własność, na której nam zależy — „dodanie `continuous` nie może przejść po
cichu" — jest własnością **czasu kompilacji** i tak też się ją sprawdza:

```text
1. dopisz tymczasowo `continuous` do mt::kinematics::JointType
2. cmake -B build -DKINEMAFORGE_WERROR=ON && cmake --build build
3. build MUSI się wywalić na -Wswitch w mapJointType
4. cofnij zmianę z (1)
```

To jest ręczna procedura akceptacyjna tego proposala, nie test w `ctest`.

> **Do rozstrzygnięcia w review:** czy warto ją zautomatyzować — osobny target
> CMake kompilujący jeden plik z dodanym enumeratorem i oczekujący porażki
> (`build_command` + `WILL_FAIL`). Rekomendacja: **nie teraz.** Taki test wymaga
> drugiej kopii enuma albo generowania źródła, co jest większą komplikacją niż
> ochrona, którą daje. Krok 1–4 wykonany raz przy implementacji i odnotowany w
> `STATUS.md` wystarcza, bo `-Wswitch` jest mechanizmem kompilatora, nie naszym
> kodem — nie ma się co regresować.

### 6.2 Testy w `ctest`

**Zero nowych.** Wszystkie 273 muszą przechodzić dalej, w tym z
`-DKINEMAFORGE_WERROR=ON`.

To jest świadomy wynik, nie zapomnienie: ten proposal nie dodaje ani nie zmienia
żadnego zachowania obserwowalnego przez publiczne API. Zmienia konfigurację
buildu, dwa komentarze/inicjalizatory i jedną nieosiągalną ścieżkę błędu.

## 7. Plan zmian w plikach

```text
CMakeLists.txt                                       +40  polityka + 2 podpięcia
tests/CMakeLists.txt                                  ~1  podpięcie warningów
.gitignore                                            ~1  build/ -> build*/  (10.2)
src/ik_equations/IkEquationBuilder.hpp                ~7  przerysowany diagram
src/ik_equations/builders/KinematicChainBuilder.cpp   ~2  Frame{baseLink, 0, nullopt}
src/ik_equations/UrdfModelLoader.cpp                 ~18  throw + komentarz kontraktowy
```

Bez zmian w: FK, warstwie symbolicznej, TCP, modelu równań, `robot_model*`,
`tests/*.cpp`.

Wobec v1 wypadają: `src/ik_equations/detail/JointTypeMapping.hpp` (nowy plik),
`tests/test_urdf_model_loader.cpp`, `tests/test_ik_equation_builder.cpp`.

## 8. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| **Gałąź MSVC jest niezweryfikowana** | **średnie** | Na maszynie deweloperskiej **nie ma MSVC**: `cl.exe` nie jest na PATH, brak katalogu instalacyjnego Visual Studio, a build używa `g++` z MinGW dołączonego do CLiona (generator Ninja) — sprawdzone w `build/CMakeCache.txt`. Flagi `/w14062` i `/w14061` są więc oparte na dokumentacji Microsoftu, **nie na uruchomieniu**. Dopóki MSVC nie jest realną konfiguracją buildu, ta gałąź pozostaje kodem obronnym o nieprzetestowanym działaniu i tak należy ją traktować |
| `-Wall -Wextra` na `kinemaforge_tests` odkryje ostrzeżenia z makr gtest rozwiniętych w naszych TU | **średnie** — rdzeń sprawdzony (2 błędy), `tests/` nie | zmierzyć przy implementacji. Jeśli hałas pochodzi z nagłówków gtest, wciągnąć je jako `SYSTEM`; jeśli z naszego kodu w testach — naprawić |
| `/w14061` okaże się hałaśliwe na MSVC | niskie | Nie da się zmierzyć bez MSVC (wiersz powyżej). Jeśli kiedyś zaszumi — usunąć `/w14061`, zostawić `/w14062`, bo tylko drugie jest częścią kontraktu |
| Biała lista targetów wymaga dyscypliny przy nowym targecie | niskie | wada widoczna, w przeciwieństwie do cichego objęcia zależności |
| `logic_error` przechodzi przez fasadę do wołającego | **zamierzone** | 4.1; alternatywą jest cicha zamiana błędu programisty na błąd danych |
| Ochrona z 6.1 nie jest testem automatycznym | niskie | `-Wswitch` jest mechanizmem kompilatora; procedura ręczna wykonana raz i odnotowana |
| Przerysowany diagram w publicznym nagłówku to zmiana dokumentacji API | niskie | treść identyczna, czytelność wyższa |

## 9. Do rozstrzygnięcia w review (v3)

Zamknięte w v3: MSVC/C4062 (wymagana poprawka — przyjęta), jawne `-Wswitch`
(przyjęte), gate CMake (przyjęte), `optional` w `mapJointType` (wycofane w v2),
nagłówek `detail/` (wycofany w v2), testy runtime (wycofane w v2).

Pozostaje otwarte:

1. Czy `kinemaforge_tests` faktycznie ma dostać `-Wall -Wextra` — review mówi
   tak; do potwierdzenia po zmierzeniu hałasu z gtest (ryzyko #2). To jedyna
   rzecz w tym dokumencie, którą **da się** zmierzyć przed implementacją, i mogę
   to zrobić na życzenie.
2. Czy `/w14061` zostaje obok `/w14062` — rekomenduję tak, z zastrzeżeniem, że
   nikt tego nie uruchomi, dopóki MSVC nie jest realną konfiguracją (ryzyko #1
   i #3).
3. Czy `KINEMAFORGE_WERROR` to właściwa nazwa i czy CI podnosi ją dla wszystkich
   konfiguracji, czy tylko dla jednej referencyjnej.
4. Czy `.gitignore` zmieniamy na `build*/` w ramach tego proposala (10.2), czy
   osobno.
5. Czy `describeCode` zostaje z fallbackiem (4.2) — jawne odstępstwo od reguły
   „bez cichych fallbacków", proszę o akceptację albo sprzeciw.
6. Czy przyjęty kształt diagramu (5.2) jest akceptowalny — kwestia gustu w
   publicznym nagłówku.

## 10. Rekomendacja końcowa

Wykonać jako pierwszy dokument etapu `F2.3.R`. Zakres jest mały i mechaniczny,
ale efekt nie jest kosmetyczny: bez tego kroku zdanie z
`IkEquationSystem.cpp:133` pozostaje nieprawdziwe, a regresja z uwagi 9 —
`continuous → Fixed` bez jednego ostrzeżenia — jest w pełni możliwa i będzie
możliwa dokładnie w momencie, w którym ktoś zacznie implementować `continuous`.

### 10.1 Gate

> **Poprawka z review.** v2 miało tu `cmake -B build -DKINEMAFORGE_WERROR=ON, build`
> — nie jest to poprawna komenda CMake. Ale problem był głębszy niż literówka:
> **rekonfiguracja istniejącego `build/` zapisałaby `KINEMAFORGE_WERROR=ON` do
> cache'a**, a wtedy każdy kolejny lokalny build byłby buildem `-Werror`. To
> wprost przeczy decyzji z sekcji 3, dla której cała opcja istnieje. Osobny
> katalog rozwiązuje jedno i drugie.

Build normalny — bez `-Werror`, tak jak pracuje się codziennie:

```powershell
cmake --build build
ctest --test-dir build --output-on-failure
```

Build weryfikujący politykę — **osobny katalog, nietykający lokalnej
konfiguracji**:

```powershell
cmake -B build-werror -G Ninja -DKINEMAFORGE_WERROR=ON
cmake --build build-werror
ctest --test-dir build-werror --output-on-failure
```

Warunki przejścia:

```text
oba buildy przechodzą
ctest 273/273 w obu (bez nowych testów — patrz 6.2)
procedura 6.1 (dopisz enumerator → build-werror MUSI padnąć) wykonana i odnotowana
warningi nie są nakładane na pugixml ani googletest              sprawdzone
```

### 10.2 Wynik implementacji — dwa rozjazdy wobec v3

Implementacja przeszła gate w całości, ale **dwa twierdzenia z v3 okazały się
niedokładne** i zapisuję je tutaj, żeby dokument zgadzał się z tym, co zaszło.

**(1) „Dokładnie dwa błędy, nic więcej" było niepełne.** Pomiar z sekcji „Status
weryfikacji" używał `-fsyntax-only`, który nie wykonuje analizy nieużywanych
funkcji. Realny build ujawnił **dwa dodatkowe** ostrzeżenia:

```text
JointTransformBuilder.cpp:22   'isUnitLength' defined but not used  [-Wunused-function]
JointTransformBuilder.cpp:186  'isActuated'   defined but not used  [-Wunused-function]
```

Naprawione przez `[[maybe_unused]]` — to idiomatyczna odpowiedź dla helperów
wołanych wyłącznie z `assert()`. **Ale przyczyna jest ważniejsza od poprawki:**
te funkcje są nieużywane, ponieważ `build/` ma `CMAKE_BUILD_TYPE=Release`, czyli
`-DNDEBUG`, czyli oba `assert()` w `JointTransformBuilder` są wykompilowane.
Kontrakt „asserted, not re-validated" nie obowiązuje w buildzie, w którym
puszczane są testy. Odnotowane w `../../STATUS.md`; decyzja o domyślnym typie
buildu wykracza poza R1 i styka się z R5.

To jest najlepsze możliwe uzasadnienie R1: ostrzeżenia znalazły realny problem w
pierwszej minucie działania.

**(2) Komenda gate'u nie działa na tej maszynie tak, jak ją zapisano.**
`cmake -B build-werror -G Ninja -DKINEMAFORGE_WERROR=ON` kończy się
`CMAKE_CXX_COMPILER not set, after EnableLanguage`. Powód: `ninja` **nie jest na
PATH** — to plik bundlowany z CLionem, a istniejący `build/` miał jego ścieżkę
zapisaną w cache'u. `g++` jest na PATH, ale sam nie wystarcza. Komenda działająca:

```powershell
cmake -B build-werror -G Ninja -DKINEMAFORGE_WERROR=ON `
      -DCMAKE_CXX_COMPILER="C:/Program Files/JetBrains/CLion 2026.1/bin/mingw/bin/g++.exe" `
      -DCMAKE_MAKE_PROGRAM="C:/PROGRA~1/JETBRA~1/CLION2~1.1/bin/ninja/win/x64/ninja.exe"
```

Do przeniesienia do `COMMANDS.md`, bo gate ma być odtwarzalny bez zaglądania do
tego proposala. Uboczna obserwacja: `build-werror/` ma **puste**
`CMAKE_BUILD_TYPE`, więc asserty są tam żywe — dwie konfiguracje różnią się
semantyką asercji, nie tylko flagami ostrzeżeń.

### 10.3 Wykonany gate — wyniki

```text
build (Release, bez -Werror)      0 ostrzeżeń, exit 0,  ctest 273/273
build-werror (WERROR=ON)          0 ostrzeżeń, exit 0,  ctest 273/273

mutation check:
  + continuous do mt::kinematics::JointType
  → UrdfModelLoader.cpp:36: error: enumeration value 'continuous'
                            not handled in switch [-Werror=switch]
  → exit 1                                            OCHRONA DZIAŁA
  → mutacja cofnięta, oba buildy znów czyste
```

Trafienie jest dokładnie jedno i w dokładnie właściwym miejscu — `mapJointType`.
`parse_joint_type` używa łańcucha `if` po stringach, a `describeCode` switchuje po
`LoadErrorCode`, więc żadne z nich nie reaguje, i tak ma być.

### 10.4 Konsekwencja dla `.gitignore`

`.gitignore` zawiera dziś dokładnie `build/` i `.claude/`, więc `build-werror/`
pojawiłby się jako nieśledzony katalog z tysiącami plików. Do zmiany razem z tym
proposalem:

```gitignore
build*/
.claude/
```

Drobiazg, ale bez niego pierwszy `git status` po wykonaniu gate'u jest bezużyteczny.
