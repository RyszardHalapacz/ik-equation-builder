# Proposal: ścisłe parsowanie `<limit>` — implementacja (R3, v3 — APPROVED)

## Prompt

> Architektura R3 zatwierdzona (v3, APPROVED). Napisać proposal implementacyjny.
>
> Review zapowiedziało, że będzie patrzeć szczególnie na: (1) jak dokładnie
> wykrywane jest `2+ × <limit>`, (2) implementację `parseFiniteNumber`/
> `parseScalar`, (3) czy `parseVector3` zachował 1:1 dotychczasową semantykę,
> (4) czy wszystkie 20 nowych przypadków testuje osobne granice, (5) czy
> `build-werror` pozostaje czysty.

Dokument realizuje `proposal-loader-limits-strict-parsing-architecture.md` (v3).
Kod poniżej jest pełny — nie ma go jeszcze na dysku.

## Historia wersji

**v1 → v2** po review (verdykt: *approve with minor changes*, bez zmian w
algorytmie C++):

| # | Poprawka z review | Gdzie |
|---|---|---|
| 1 | usunąć nieprawdziwe „O(1)" przy `next_sibling("limit")` — jest liniowe względem liczby rodzeństw | 4.1 |
| 2 | **dodać `AcceptsLimitValuesWithSurroundingWhitespace`** — kontrakt `parse_scalar` obiecuje `whitespace* number whitespace* END`, a testy pinowały tylko `END` | 6.6, 6.8 |
| 3 | „cztery alokacje" → cztery **konstrukcje** `std::string` (SSO) | 7 |
| 4 | licznik `56 testów loadera` był sprzed R2 | 7, 8 |

Punkt 2 był realną luką w planie testów, nie kosmetyką: istniejący test białych
znaków przechodzi przez `parse_vector3`, czyli po refaktorze z §3.2 **inną
warstwę** niż ta, którą `<limit>` używa.

**v2 → v3** po review (verdykt: *approve with one test change*; zero uwag do kodu
produkcyjnego):

| # | Poprawka z review | Gdzie |
|---|---|---|
| 5 | cztery testy „brakuje jednego atrybutu" **nie pinują kolejności** — każde wejście ma jeden błąd, więc przechodzą przy dowolnej kolejności sprawdzania | 6.2 (sprostowanie) |
| 6 | dodać test kolizyjny na deterministyczną kolejność | **6.2a** (nowa) |
| 7 | `+20` w §1 nie nadążyło za `+21` z v2 | 1 |

Liczby po poprawkach: **22 nowe testy**, baseline **290**, razem **312**.
Zmierzone `ctest -N`: 71 testów loadera po R2, z czego 56 sprzed R2.

## Status weryfikacji

Punkt wyjścia: `main` + niezacommitowana implementacja R2. Oba buildy czyste,
**290/290** testów. Liczby linii odnoszą się do stanu po R2.

## 1. Kolejność zmian

```text
1. robot_model_loader.hpp   LoadErrorCode +5 -1, profil w komentarzu
2. robot_model_loader.cpp   parseFiniteNumber / parse_scalar / parse_vector3
3. robot_model_loader.cpp   blok <limit>
4. UrdfModelLoader.cpp      describeCode +5 -1
5. test_urdf_model_loader.cpp  +22
```

Krok 4 nie jest opcjonalny i nie da się go przeoczyć: po R1 brak `case` w
`describeCode` wywala build na `-Werror=switch`.

## 2. `robot_model_loader.hpp`

### 2.1 Kody błędów

```cpp
enum class LoadErrorCode : std::uint8_t {
    file_not_found,
    parse_failure,
    unsupported_joint_type,
    malformed_vector,   // attribute present but not three finite numbers
    degenerate_axis,    // actuated joint whose axis has zero length

    // Joint limits. See the strict profile documented on load_urdf.
    missing_limit_element,     // revolute/prismatic without <limit>
    duplicate_limit_element,   // more than one <limit> in one joint
    missing_limit_attribute,   // a required <limit> attribute is absent
    malformed_limit_value,     // present, but not one finite number
    invalid_limit_constraint,  // lower > upper, or velocity/effort < 0

    // Topology. A URDF describes a tree; these are the ways it can fail to.
    invalid_link_declaration,  // <link> with an empty or duplicate name
    undeclared_parent_link,    // joint parent has no matching <link>
    undeclared_child_link,     // joint child has no matching <link>
    duplicate_child_link,      // one link is the child of two joints
    cyclic_topology,           // the graph contains a cycle
    disconnected_topology,     // not one connected component / not one root
};
```

`invalid_limits` usunięty. Wstawka jest **w środku enuma**, nie na końcu:
`LoadErrorCode` nie jest serializowany ani nigdzie utrwalany, a grupowanie
tematyczne jest warte więcej niż stabilność wartości liczbowych. Jedyny konsument
poza tym plikiem to `describeCode`, który dopasowuje po nazwie.

### 2.2 Profil jako kontrakt publiczny

Do bloku komentarza nad `load_urdf`, po istniejących akapitach o geometrii i
topologii (rekomendacja z review, punkt 1 z §9 architektury — krótka wersja, bez
argumentacji):

```cpp
// ...and joint limits satisfy the KinemaForge strict profile:
//   * a revolute or prismatic joint carries EXACTLY ONE <limit> element
//   * that element has all four of lower, upper, velocity and effort
//   * each value is exactly one finite number -- "1 2" and "nan" are rejected,
//     and a missing attribute is an error rather than a zero
//   * lower <= upper, velocity >= 0, effort >= 0
//   * a <limit> on a fixed joint is ignored, contents and all
//
// This is deliberately narrower than older URDF, where lower and upper default
// to 0, and it is NOT the URDF 1.2 rule, where effort and velocity are optional.
// It is this project's own profile: the loader does not read <robot version>
// and claims conformance to no particular release. A joint whose limits default
// to [0, 0] cannot be told apart from a mistake, and this project derives IK
// equations from those numbers.
```

## 3. `robot_model_loader.cpp` — warstwy parsera

### 3.1 `parseFiniteNumber` — wyodrębniony rdzeń

Wstawić **przed** `parse_vector3`, po `is_space`:

```cpp
// Parses one finite number starting at `p`, advancing it past the token.
//
// Deliberately does NOT skip surrounding whitespace and does NOT require
// reaching the end of the input: those are the callers' rules, and they differ.
// parse_scalar demands the end, parse_vector3 must be able to stop mid-string
// and keep reading. Folding either rule in here would make one of the two
// callers wrong.
//
// std::from_chars refuses a leading '+', so one is skipped by hand -- but at
// most one, and it may not be followed by another sign, otherwise "+-1" would
// slip through as -1.
bool parse_finite_number(char const*& p, char const* end, double& out) noexcept {
    if (p == end) return false;

    if (*p == '+') {
        ++p;
        if (p == end || *p == '+' || *p == '-') return false;
    }

    auto const [next, ec] = std::from_chars(p, end, out);
    if (ec != std::errc{})   return false;   // garbage, or out of range ("1e400")
    if (!std::isfinite(out)) return false;   // "nan", "inf"
    p = next;
    return true;
}
```

Sygnatura z `char const*&` i `double&` zamiast `std::optional<std::pair<...>>`:
funkcja musi zwrócić **dwie** rzeczy (wartość i nową pozycję) i jest wołana w
pętli w gorącym miejscu parsera. `bool` + wyjścia przez referencję są tu
czytelniejsze niż rozpakowywanie pary, a `noexcept` zostaje zachowane — cała
rodzina parserów jest `noexcept`, bo nie buduje `LoadError` (kontekst dokłada
wołający).

### 3.2 `parse_vector3` — przepisane na rdzeń, semantyka 1:1

```cpp
// Accepts exactly three finite numbers separated by whitespace:
//
//     whitespace* number whitespace+ number whitespace+ number whitespace*
//
// Rejects a wrong count, trailing garbage, non-finite values ("nan", "inf")
// and out-of-range literals ("1e400").
std::optional<Vec3> parse_vector3(std::string_view text) noexcept {
    if (text.empty()) return std::nullopt;   // present but empty attribute

    char const* p   = text.data();
    char const* end = p + text.size();

    double components[3]{};
    for (int i = 0; i < 3; ++i) {
        // Numbers must be separated by whitespace: "1-2 3" is not a vector.
        if (i > 0 && (p == end || !is_space(*p))) return std::nullopt;
        while (p < end && is_space(*p)) ++p;
        if (!parse_finite_number(p, end, components[i])) return std::nullopt;
    }

    while (p < end && is_space(*p)) ++p;
    if (p != end) return std::nullopt;

    return Vec3{components[0], components[1], components[2]};
}
```

**Dowód równoważności 1:1** — review zapowiedziało, że będzie tego pilnować.
Zmieniony jest wyłącznie środek pętli. Poprzednio:

```cpp
if (p == end) return std::nullopt;              // (A)
if (*p == '+') { ++p; if (p == end || *p == '+' || *p == '-') return ...; }  // (B)
auto const [next, ec] = std::from_chars(p, end, components[i]);              // (C)
if (ec != std::errc{})             return std::nullopt;                      // (D)
if (!std::isfinite(components[i])) return std::nullopt;                      // (E)
p = next;                                                                    // (F)
```

Nowe `parse_finite_number` zawiera (A)–(F) w tej samej kolejności i zwraca
`false` dokładnie tam, gdzie stary kod robił `return std::nullopt`. Reguły, które
**zostały w `parse_vector3`** i nie przeniosły się do rdzenia:

| Reguła | Gdzie |
|---|---|
| pusty tekst → odrzuć | `parse_vector3`, bez zmian |
| separator whitespace wymagany między składowymi | `parse_vector3`, bez zmian |
| pominięcie białych znaków przed składową | `parse_vector3`, bez zmian |
| dokładnie trzy składowe | `parse_vector3`, bez zmian |
| końcowe białe znaki dozwolone, śmieci nie | `parse_vector3`, bez zmian |

Nie ma ścieżki, w której nowa wersja przyjmuje wejście odrzucane przez starą lub
odwrotnie. Pilnuje tego pięć istniejących testów: `ParsesValidVector3`,
`AcceptsLeadingAndTrailingWhitespace`, `AcceptsLeadingPlusSign`,
`AcceptsScientificNotation`, `AcceptsCharacterReferenceTabSeparators`, plus
`RejectsMalformedVector`-y.

### 3.3 `parse_scalar` — nowe

```cpp
// Exactly one finite number, with optional surrounding whitespace:
//
//     whitespace* number whitespace*
//
// The END requirement lives here rather than in parse_finite_number, and it is
// the whole point: without it lower="1 2" would quietly become 1.
std::optional<double> parse_scalar(std::string_view text) noexcept {
    if (text.empty()) return std::nullopt;   // present but empty attribute

    char const* p   = text.data();
    char const* end = p + text.size();

    while (p < end && is_space(*p)) ++p;

    double value = 0.0;
    if (!parse_finite_number(p, end, value)) return std::nullopt;

    while (p < end && is_space(*p)) ++p;
    if (p != end) return std::nullopt;       // trailing garbage, or a second number

    return value;
}
```

## 4. `robot_model_loader.cpp` — blok `<limit>`

Helper, tuż przed `load_urdf` w tej samej anonimowej przestrzeni nazw:

```cpp
// Reads one required <limit> attribute under the strict profile. Absence and
// unparsable text are DIFFERENT errors: pugixml's as_double() cannot tell them
// apart -- it answers 0.0 to both -- which is what let lower="banana" and a
// missing lower produce the same silent zero.
std::expected<double, LoadError>
read_limit_attribute(pugi::xml_node limit, std::string_view jointName,
                     char const* name) {
    std::string const attribute = std::string("limit/") + name;

    auto const node = limit.attribute(name);
    if (!node)
        return std::unexpected(makeError(LoadErrorCode::missing_limit_attribute,
                                         jointName, attribute));

    auto const value = parse_scalar(node.value());
    if (!value)
        return std::unexpected(makeError(LoadErrorCode::malformed_limit_value,
                                         jointName, attribute, node.value()));

    return *value;
}
```

Blok w `load_urdf`, w miejsce obecnych linii 328–339:

```cpp
        // Fixed joints: <limit> is ignored entirely, contents and all. A limit
        // on a joint that cannot move has no meaning, and the domain model does
        // not consume it (hasPositionLimits is false for Fixed). Same spirit as
        // <axis> on a fixed joint, though not the same treatment: an axis is
        // still parsed for syntax because the parser runs for every type
        // anyway, while validating a limit here would mean running the whole
        // path only to discard the result.
        if (j.type != JointType::fixed) {
            auto lim = node.child("limit");
            if (!lim)
                return std::unexpected(makeError(LoadErrorCode::missing_limit_element,
                                                 j.name, "limit"));

            // Exactly one, not "at least one". node.child() returns the first
            // and would silently ignore the rest -- input that means two
            // different things, quietly resolved to one, is the defect class
            // this whole stage removes.
            if (lim.next_sibling("limit"))
                return std::unexpected(makeError(LoadErrorCode::duplicate_limit_element,
                                                 j.name, "limit"));

            // Fixed order, so a file with two bad attributes always reports the
            // same one. Never a loop over lim.attributes(), whose order follows
            // the file.
            auto const lower = read_limit_attribute(lim, j.name, "lower");
            if (!lower) return std::unexpected(lower.error());
            auto const upper = read_limit_attribute(lim, j.name, "upper");
            if (!upper) return std::unexpected(upper.error());
            auto const velocity = read_limit_attribute(lim, j.name, "velocity");
            if (!velocity) return std::unexpected(velocity.error());
            auto const effort = read_limit_attribute(lim, j.name, "effort");
            if (!effort) return std::unexpected(effort.error());

            // Relations last: they only mean something once all four are numbers.
            // lower == upper is allowed -- it is what the author wrote twice, not
            // something the parser invented. Zero velocity/effort is allowed too:
            // R3 validates data, it does not judge whether a robot with those
            // numbers is useful.
            if (*lower > *upper)
                return std::unexpected(makeError(LoadErrorCode::invalid_limit_constraint,
                                                 j.name, "limit/lower"));
            if (*velocity < 0.0)
                return std::unexpected(makeError(LoadErrorCode::invalid_limit_constraint,
                                                 j.name, "limit/velocity"));
            if (*effort < 0.0)
                return std::unexpected(makeError(LoadErrorCode::invalid_limit_constraint,
                                                 j.name, "limit/effort"));

            j.limits.lower    = *lower;
            j.limits.upper    = *upper;
            j.limits.velocity = *velocity;
            j.limits.effort   = *effort;
        }
```

### 4.1 Jak wykrywane jest `2+ × <limit>` — punkt 1 z listy review

```cpp
if (lim.next_sibling("limit"))
```

`pugi::xml_node::next_sibling(name)` szuka kolejnego **rodzeństwa o tej nazwie**,
czyli następnego `<limit>` wewnątrz tego samego `<joint>`. Zwraca pusty węzeł,
gdy go nie ma, a `xml_node` konwertuje się do `bool`.

Dlaczego nie `std::distance(node.children("limit"))` ani ręczna pętla licząca:
**interesuje nas wyłącznie istnienie drugiego `<limit>`**, więc
`next_sibling("limit")` wyraża ten kontrakt wprost, bez liczenia wszystkich
wystąpień. Liczba duplikatów i tak nie trafiłaby do komunikatu.

> **Sprostowanie wobec pierwszej redakcji tego dokumentu:** napisałem tu, że
> sprawdzenie jest „O(1) po pierwszym trafieniu". To nieprawda.
> `next_sibling(name)` przechodzi po kolejnych rodzeństwach, aż trafi na element
> o tej nazwie, więc w najgorszym przypadku jest liniowe względem liczby
> następujących rodzeństw. Nie zmienia to wyboru implementacji — koszt dotyczy
> dzieci jednego `<joint>`, których są jednostki — ale uzasadnienie miało być
> wydajnościowe, a jest kontraktowe.

Zaletą, którą warto odnotować: `next_sibling("limit")` znajdzie drugi `<limit>`
również wtedy, gdy między nimi stoją inne elementy albo komentarze.

Kolejność ma znaczenie: **najpierw obecność, potem liczność, potem atrybuty.**
Plik z dwoma `<limit>`, z których pierwszy ma zepsute `lower`, zgłosi
`duplicate_limit_element` — bo dopóki nie wiadomo, który element obowiązuje,
zgłaszanie błędu w jego atrybucie wskazywałoby przypadkowy z dwóch.

**Uwaga o zasięgu, wprost:** ta sama reguła liczności **nie** jest tu nakładana na
`<origin>`, `<axis>`, `<parent>` ani `<child>`, choć mają ten sam wzorzec
„pierwszy wygrywa" (§4.2 architektury). Poza zakresem R3, świadomie.

## 5. `UrdfModelLoader.cpp` — `describeCode`

```cpp
    case Code::unsupported_joint_type:     return "unsupported joint type in URDF";
    case Code::malformed_vector:           return "malformed vector attribute";
    case Code::degenerate_axis:            return "actuated joint has a zero-length axis";
    case Code::missing_limit_element:      return "actuated joint has no <limit> element";
    case Code::duplicate_limit_element:    return "joint has more than one <limit> element";
    case Code::missing_limit_attribute:    return "required joint limit attribute is missing";
    case Code::malformed_limit_value:      return "joint limit value is not a single finite number";
    case Code::invalid_limit_constraint:   return "joint limits violate a required relation";
    case Code::invalid_link_declaration:   return "link declaration has an empty or duplicate name";
    // ... topology codes unchanged
```

`invalid_limits` usunięty. Komunikat `invalid_limit_constraint` jest celowo
ogólny — `attribute` niesie `limit/lower`, `limit/velocity` albo `limit/effort`, a
`describe()` skleja go z resztą, więc pełny tekst wskazuje konkretną relację.

## 6. Testy — 20 nowych

Punkt 4 z listy review: **każdy testuje osobną granicę, nie wariant tej samej.**
Poniżej z uzasadnieniem, czego pilnuje każda grupa.

### 6.1 Liczność elementu (2)

```cpp
TEST(UrdfModelLoaderTest, RejectsMissingLimitElement)
TEST(UrdfModelLoaderTest, RejectsDuplicateLimitElement)
```

Dwie różne liczności: `0` i `2`. `1` jest pokryte każdym testem pozytywnym.

### 6.2 Obecność atrybutów (4)

```cpp
TEST(UrdfModelLoaderTest, RejectsMissingLowerLimit)
TEST(UrdfModelLoaderTest, RejectsMissingUpperLimit)
TEST(UrdfModelLoaderTest, RejectsMissingVelocity)
TEST(UrdfModelLoaderTest, RejectsMissingEffort)
```

Cztery osobne, nie jeden sparametryzowany: każdy pokrywa inny atrybut, a
`missing_limit_attribute` musi wskazać właściwy przez `attribute`.

> **Sprostowanie wobec v1 tego dokumentu.** Napisałem tu, że te cztery testy
> pinują *ustaloną kolejność sprawdzania*. **Nie pinują.** Każde z tych wejść ma
> dokładnie jeden brakujący atrybut, więc wszystkie cztery przechodzą tak samo
> dla implementacji `lower → upper → velocity → effort`, jak i dla
> `effort → velocity → upper → lower`. Kolejność jest kontraktem architektury i
> wymaga wejścia **kolizyjnego** — patrz §6.2a.

`RejectsMissingLowerLimit` jest jednocześnie **testem regresji na błąd (b)** —
plik z `velocity` i `effort`, bez `lower`/`upper`, dziś ładuje się z `[0, 0]`.

### 6.2a Kolejność sprawdzania — wejście kolizyjne (1)

```cpp
TEST(UrdfModelLoaderTest, ReportsFirstMissingLimitAttributeInContractOrder)
```

Jedyny test, który faktycznie pinuje kontrakt „lower → upper → velocity →
effort". Dwa wejścia w jednym teście, żeby pokryć obie połowy łańcucha bez
dokładania czterech osobnych testów:

```cpp
TEST(UrdfModelLoaderTest, ReportsFirstMissingLimitAttributeInContractOrder)
{
    // The four single-omission tests above cannot pin the order: with one fault
    // per input, any checking order produces the same result. Only a colliding
    // input can, and the order is a contract -- a caller fixing errors one at a
    // time gets a deterministic sequence, not one that depends on how the
    // validator happens to be written today.

    // lower, upper AND velocity missing -> lower wins over both.
    {
        const TemporaryUrdf file{urdfWithJoint(jointWithLimit(
            R"(<limit effort="1"/>)"))};
        const auto result = mt::kinematics::load_urdf(file.path());

        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code,
                  mt::kinematics::LoadErrorCode::missing_limit_attribute);
        EXPECT_EQ(result.error().attribute, "limit/lower");
    }

    // Second half of the chain: velocity wins over effort.
    {
        const TemporaryUrdf file{urdfWithJoint(jointWithLimit(
            R"(<limit lower="-1" upper="1"/>)"))};
        const auto result = mt::kinematics::load_urdf(file.path());

        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code,
                  mt::kinematics::LoadErrorCode::missing_limit_attribute);
        EXPECT_EQ(result.error().attribute, "limit/velocity");
    }
}
```

Razem pinuje `lower < upper`, `lower < velocity` i `velocity < effort` — cały
łańcuch, jednym testem.

### 6.3 Postać wartości (5)

```cpp
TEST(UrdfModelLoaderTest, RejectsMalformedLowerLimit)        // lower="banana"
TEST(UrdfModelLoaderTest, RejectsMalformedUpperLimit)        // upper="1 2"   ← dwie liczby
TEST(UrdfModelLoaderTest, RejectsEmptyLimitAttribute)        // lower=""
TEST(UrdfModelLoaderTest, RejectsNonFiniteVelocity)          // velocity="nan"
TEST(UrdfModelLoaderTest, RejectsNonFiniteEffort)            // effort="inf"
```

Pięć **różnych** sposobów, w jaki tekst nie jest jedną skończoną liczbą: nie-liczba,
dwie liczby, pusty, NaN, nieskończoność. `upper="1 2"` jest tym, który pinuje
granicę warstw z §3.1 — gdyby `END` wyciekło do `parse_finite_number`, ten jeden
by padł, a reszta przeszła.

`RejectsNonFiniteVelocity` i `RejectsNonFiniteEffort` są **testami regresji na
błąd (c)** — dziś oba przechodzą.

### 6.4 Zakres i przepełnienie (1)

```cpp
TEST(UrdfModelLoaderTest, RejectsOverflowingLimitValue)      // upper="1e400"
```

Odrębne od „nie-liczba": `1e400` jest poprawną składnią, którą `from_chars`
odrzuca przez `result_out_of_range`, nie przez `invalid_argument`. Inna gałąź
kodu.

### 6.5 Relacje (3)

```cpp
TEST(UrdfModelLoaderTest, RejectsInvertedLimitRange)         // lower="1" upper="-1"
TEST(UrdfModelLoaderTest, RejectsNegativeVelocity)
TEST(UrdfModelLoaderTest, RejectsNegativeEffort)
```

Trzy relacje sprawdzane osobno w kodzie, więc trzy testy. Wszystkie dają
`invalid_limit_constraint`, ale różne `attribute`.

### 6.6 Granica ścisłości — co ma przechodzić (5)

```cpp
TEST(UrdfModelLoaderTest, AcceptsZeroWidthLimitRange)        // lower="1.5" upper="1.5"
TEST(UrdfModelLoaderTest, AcceptsZeroVelocityAndEffort)      // velocity="0" effort="0"
TEST(UrdfModelLoaderTest, AcceptsLimitValuesInScientificNotation)
TEST(UrdfModelLoaderTest, AcceptsLimitValuesWithSurroundingWhitespace)
TEST(UrdfModelLoaderTest, IgnoresLimitForFixedJoint)         // fixed + lower="banana"
```

Bez tych pięciu nie da się odróżnić „ścisłe" od „za ścisłe". `IgnoresLimitForFixedJoint`
jest jedynym dowodem na decyzję z §4.5 architektury.

`AcceptsLimitValuesWithSurroundingWhitespace` **dodany po review** i uzupełnia
realną lukę: architektura obiecuje `parse_scalar` jako
`whitespace* number whitespace* END`, a testy pinowały dotąd wyłącznie `END`
(przez `upper="1 2"`). Istniejący test białych znaków sprawdza `parse_vector3`,
czyli inną warstwę — po refaktorze z §3.2 nie mówi już nic o ścieżce skalara.

```cpp
TEST(UrdfModelLoaderTest, AcceptsLimitValuesWithSurroundingWhitespace)
{
    // Pins the full parse_scalar contract, not just its END rule: leading and
    // trailing whitespace are allowed around a limit value. The '+' on upper
    // also exercises the leading-sign branch through the scalar path -- the
    // existing whitespace and plus-sign tests both go through parse_vector3,
    // which is a different layer since parse_finite_number was extracted.
    const auto joint = loadSingleJoint(urdfWithJoint(jointWithLimit(
        R"(<limit lower="  -1.5  " upper="  +1.5  ")"
        R"( velocity="  2.0  " effort="  10.0  "/>)")));

    EXPECT_DOUBLE_EQ(joint.limits.lower, -1.5);
    EXPECT_DOUBLE_EQ(joint.limits.upper, 1.5);
    EXPECT_DOUBLE_EQ(joint.limits.velocity, 2.0);
    EXPECT_DOUBLE_EQ(joint.limits.effort, 10.0);
}
```

`loadSingleJoint` to istniejący pomocnik z `test_urdf_model_loader.cpp:98` —
ładuje przez `UrdfModelLoader` i zwraca `joints.at(0)`. Sprawdzone, że to jedyny
pomocnik tego rodzaju w pliku.

### 6.7 Diagnostyka (1)

```cpp
TEST(UrdfModelLoaderTest, ReportsMalformedLimitWithAttributeContext)
```

Sprawdza `code`, `jointName`, `attribute == "limit/lower"` i `rawValue == "banana"`
— czyli że komunikat wskazuje konkretny atrybut konkretnego jointu, a nie samo
„coś z limitami".

### 6.8 Bilans i przykładowy kod

```text
6.1  liczność       2
6.2  obecność       4
6.2a kolejność      1        ← wejście kolizyjne, dodane w v3
6.3  postać         5
6.4  zakres         1
6.5  relacje        3
6.6  granica        5
6.7  diagnostyka    1
                   --
                   22        →  290 + 22 = 312 testów
```

(Architektura mówi o 20 w §6.4; doszły `AcceptsLimitValuesWithSurroundingWhitespace`
(v2) i `ReportsFirstMissingLimitAttributeInContractOrder` (v3). Obie różnice są
świadome i dotyczą testów, nie kontraktu.)

Helper i dwa reprezentatywne testy; reszta ma tę samą postać:

```cpp
// Builds a revolute joint whose <limit> is exactly the given XML, so each test
// varies one thing. urdfWithJoint supplies <link name="base"/> and
// <link name="tool"/>, which the topology invariants from R2 require.
std::string jointWithLimit(std::string_view limitXml)
{
    return std::string{R"(<joint name="j" type="revolute">)"}
         + R"(<parent link="base"/><child link="tool"/>)"
         + R"(<axis xyz="0 0 1"/>)"
         + std::string{limitXml}
         + "</joint>";
}

TEST(UrdfModelLoaderTest, RejectsMissingLowerLimit)
{
    // Regression: this used to load with limits [0, 0] -- an actuated joint
    // frozen in place, indistinguishable from a modelling mistake, and passing
    // the old lower > upper check because 0 > 0 is false.
    expectRejectedWith(urdfWithJoint(jointWithLimit(
                           R"(<limit velocity="1" effort="1"/>)")),
                       mt::kinematics::LoadErrorCode::missing_limit_attribute);
}

TEST(UrdfModelLoaderTest, RejectsDuplicateLimitElement)
{
    // Two <limit> elements mean two different things. node.child() would take
    // the first and say nothing, which is the defect class this stage removes.
    expectRejectedWith(urdfWithJoint(jointWithLimit(
                           R"(<limit lower="-1" upper="1" velocity="2" effort="10"/>)"
                           R"(<limit lower="-3" upper="3" velocity="5" effort="50"/>)")),
                       mt::kinematics::LoadErrorCode::duplicate_limit_element);
}

TEST(UrdfModelLoaderTest, IgnoresLimitForFixedJoint)
{
    // A limit on a joint that cannot move has no meaning, so its contents are
    // not even parsed. Without this test the decision lives only in a comment.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="tool"/>)"
        R"(<joint name="j" type="fixed">)"
        R"(<parent link="base"/><child link="tool"/>)"
        R"(<limit lower="banana" upper="also nonsense"/>)"
        R"(</joint></robot>)";

    const TemporaryUrdf file{urdf};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_TRUE(result.has_value());
    EXPECT_FALSE(result->model.joints().empty());
}

TEST(UrdfModelLoaderTest, ReportsMalformedLimitWithAttributeContext)
{
    const TemporaryUrdf file{urdfWithJoint(jointWithLimit(
        R"(<limit lower="banana" upper="1" velocity="1" effort="1"/>)"))};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, mt::kinematics::LoadErrorCode::malformed_limit_value);
    EXPECT_EQ(result.error().jointName, "j");
    EXPECT_EQ(result.error().attribute, "limit/lower");
    ASSERT_TRUE(result.error().rawValue.has_value());
    EXPECT_EQ(*result.error().rawValue, "banana");
}
```

## 7. Ryzyka implementacyjne

| Ryzyko | Reakcja |
|---|---|
| `parse_vector3` zmieni zachowanie mimo dowodu z §3.2 | Wszystkie **71** istniejących testów loadera musi przejść bez zmian w treści — w tym **56 sprzed R2**, czyli cały dotychczasowy zestaw wokół `parse_vector3`. Konieczność edycji któregokolwiek = refaktor nie jest 1:1 i wraca do review |
| `read_limit_attribute` buduje `std::string` na każdą nazwę atrybutu | Świadome: cztery **małe konstrukcje `std::string`** na aktuowany joint, wyłącznie na ścieżce ładowania pliku. Nie twierdzę czterech alokacji na stercie — napisy tej długości najpewniej mieszczą się w SSO, a ewentualny koszt jest tu nieistotny. Czytelność wygrywa; `LoadError::attribute` i tak jest `std::string` |
| Wstawka w środku enuma przesuwa wartości liczbowe | `LoadErrorCode` nie jest serializowany; jedyny konsument dopasowuje po nazwie |
| `next_sibling("limit")` nie złapie `<limit>` zagnieżdżonego głębiej | Poprawne — profil mówi o bezpośrednich dzieciach `<joint>`, tak jak reszta loadera |

## 8. Gate

```text
baseline (po R2)                       290
nowe w R3                               22
                                       ---
                                       312

cmake --build build                    0 ostrzeżeń
ctest --test-dir build                 312/312
cmake --build build-werror             0 ostrzeżeń          ← punkt 5 z listy review
ctest --test-dir build-werror          312/312

parse_vector3:
    71 istniejących testów loadera bez edycji treści
    (w tym 56 sprzed R2 — cały dotychczasowy zestaw parsera)

<limit>  odrzuca:   0 elementów, 2+ elementów, brak atrybutu, zły tekst,
                    nan/inf, 1e400, złamane relacje
<limit>  przyjmuje: otaczające białe znaki, wiodący '+',
                    lower == upper, velocity/effort == 0,
                    dowolną treść przy joincie Fixed

oba pliki w data/urdf ładują się z niezmienionymi wartościami limitów
    (MapsKr4VelocityAndEffortLimits, MapsFixedJointWithoutPositionLimits
     przechodzą bez edycji)
```

Punkt o 71 testach jest najważniejszy dla wiarygodności refaktoru z §3.2: jeśli
którykolwiek wymaga zmiany, `parse_vector3` **nie** zachował semantyki 1:1.
Liczba 71 jest zmierzona (`ctest -N`), nie oszacowana; 56 to stan sprzed R2, do
którego odnosił się pierwotny plan testów architektury.
