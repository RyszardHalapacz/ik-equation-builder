# Proposal: walidacja i normalizacja geometrii URDF — architektura

## 1. Cel

Ustanowić w loaderze kontrakt, że `RobotDescription` niesie **geometrię kanoniczną**:

```
origin.translation:   dokładnie trzy wartości, wszystkie skończone
origin.rpy:           dokładnie trzy wartości, wszystkie skończone, w radianach
axis (joint aktywny): dokładnie trzy wartości, skończone, niezerowe, znormalizowane do długości 1
axis (joint Fixed):   nie bierze udziału w kinematyce; brak <axis> nie jest błędem
```

Dzięki temu `JointTransformBuilder` będzie mógł ten kontrakt **asertować**, zamiast ponownie walidować XML — zgodnie z decyzją przyjętą w `proposal-joint-transform-builder-architecture.md` §11.2.

## 2. Stan obecny repozytorium

Wszystko poniżej **odczytane z kodu i zweryfikowane uruchomieniem**.

### 2.1 Parsowanie wektorów

`robot_model_loader.cpp`, przestrzeń anonimowa:

```cpp
Vec3 parse_xyz(char const* attr) noexcept {
    Vec3 v{};
    if (!attr || !*attr) return v;
    char const* p = attr; char const* end = p + std::strlen(p);
    double* dst[3] = {&v.x, &v.y, &v.z};
    for (int i = 0; i < 3; ++i) {
        while (p < end && *p == ' ') ++p;        // tylko spacja
        auto [next, ec] = std::from_chars(p, end, *dst[i]);
        if (ec != std::errc{}) break;            // ciche przerwanie
        p = next;
    }
    return v;
}
```

Wywołania:

```cpp
if (auto origin = node.child("origin"); origin) {
    j.origin_xyz = parse_xyz(origin.attribute("xyz").as_string());
    j.origin_rpy = parse_xyz(origin.attribute("rpy").as_string());
}
if (auto axis = node.child("axis"); axis)
    j.axis = parse_xyz(axis.attribute("xyz").as_string());
```

### 2.2 Model i domyślne wartości

`mt::kinematics::Joint`: `Vec3 axis = {0.0, 0.0, 1.0};` — komentarz mówi *„unit vector in parent's local frame"*, ale **nic tego nie egzekwuje**. `origin_xyz`/`origin_rpy` domyślnie `{}` = `[0,0,0]`.

Warstwa `ik`: `Vector3 {double x{}, y{}, z{};}` — domyślnie zera. `KinematicJoint::axis` kopiowana wprost przez `copyJointData`, bez żadnej obróbki.

### 2.3 Walidacja — co istnieje

Jedyna walidacja liczbowa w całym loaderze:

```cpp
if (j.type != JointType::fixed) {
    auto lim = node.child("limit");
    if (!lim) return std::unexpected(LoadError::invalid_limits);
    ...
    if (!std::isfinite(j.limits.lower) || !std::isfinite(j.limits.upper) ||
        j.limits.lower > j.limits.upper)
        return std::unexpected(LoadError::invalid_limits);
}
```

Dotyczy **wyłącznie limitów**. `axis` i `origin` nie są sprawdzane nigdzie — potwierdzone przez `grep -rn "isfinite" src/` (jedno trafienie, powyższe) oraz `grep -rn "normaliz|sqrt|length|magnitude" src/` (zero trafień poza komentarzem o wyrażeniach).

### 2.4 Model błędów i jego propagacja

```cpp
enum class LoadError : std::uint8_t {
    file_not_found, parse_failure, unsupported_joint_type,
    incomplete_kinematic_chain, invalid_limits,
};
std::expected<LoadResult, LoadError> load_urdf(std::filesystem::path const&);
struct LoadResult { RobotModel model; DiagnosticBag diagnostics; };
```

`UrdfModelLoader::load` konsumuje to tak:

```cpp
auto result = mt::kinematics::load_urdf(urdfPath);
if (!result)
    throw std::runtime_error("UrdfModelLoader: " + describe(result.error()) + " (" + path + ")");
```

Dwie konsekwencje, istotne dla §12:

1. **Kod błędu ginie na granicy warstwy `ik`** — zostaje wyłącznie tekst w wyjątku. Żaden klient nie może rozgałęzić się po przyczynie.
2. **`DiagnosticBag` ginie przy niepowodzeniu** — `std::expected` zwraca albo `LoadResult` (z workiem), albo goły `LoadError`. Przy błędzie diagnostyk nie ma jak przekazać.

Testy sprawdzają wyłącznie `EXPECT_THROW(..., std::runtime_error)` — nie inspekcjonują kodów.

### 2.5 Ścieżka `Continuous` — nieprzejezdna

```cpp
std::expected<JointType, LoadError> parse_joint_type(std::string_view s) noexcept {
    if (s == "revolute")  return JointType::revolute;
    if (s == "prismatic") return JointType::prismatic;
    if (s == "fixed")     return JointType::fixed;
    return std::unexpected(LoadError::unsupported_joint_type);
}
```

`mt::kinematics::JointType` **nie ma wartości `continuous`**. Zbudowałem syntetyczny URDF z `type="continuous"` i załadowałem prawdziwym loaderem:

```
continuous joint  ->  BŁĄD: unsupported_joint_type
```

Cały plik odrzucony. `kinemaforge::ik::JointType::Continuous` jest zatem **martwym wpisem enuma** — nic w systemie nie może go wyprodukować, mimo że `KinematicChainBuilder::isActuated` go obsługuje, a `JointTransformBuilder` ma go w planie.

Dodatkowo: gdyby `Continuous` został dodany, obecny warunek `if (j.type != fixed) → wymagaj <limit>` byłby **semantycznie błędny** — URDF nie wymaga `lower`/`upper` dla continuous. Załamałby się też komentarz w `UrdfModelLoader.cpp`:

```cpp
// The copied loader requires <limit> for every non-fixed joint, so
// presence of position limits tracks 1:1 with "is actuated".
result.limits.hasPositionLimits = (result.type != JointType::Fixed);
```

### 2.6 Konwencje testów i CMake

Testy ładują fixture'y z `data/urdf/` przez makro `KINEMAFORGE_URDF_DATA_DIR`; syntetyczne przypadki budowane są jako ręczne `RobotDescription` **tam, gdzie da się ominąć loader** (`test_kinematic_chain_builder.cpp`). Dla samego loadera nie ma dziś żadnego mechanizmu podania złego wejścia — jedyny test negatywny to `ThrowsOnMissingFile`.

Stan: **70 testów**, wszystkie zielone. `CMakeLists.txt` wymienia pliki `.cpp` jawnie.

## 3. Udowodnione problemy aktualnego parsera

Uruchomiłem dokładną kopię `parse_xyz` na pełnym zestawie przypadków z promptu:

| wejście | wynik | ocena |
|---|---|---|
| `"1 2 3"` | `[1, 2, 3]` | ✅ poprawne |
| `"1 abc 3"` | `[1, 0, 0]` | ❌ ciche obcięcie |
| `"abc 2 3"` | `[0, 0, 0]` | ❌ całość zignorowana |
| `"1 2"` | `[1, 2, 0]` | ❌ brakująca składowa → 0 |
| `"1"` | `[1, 0, 0]` | ❌ jedna wartość akceptowana |
| `""` | `[0, 0, 0]` | ❌ **nieodróżnialne od jawnego `"0 0 0"`** |
| `"1 2 3 4"` | `[1, 2, 3]` | ❌ nadmiar zignorowany |
| `"1 2 3 abc"` | `[1, 2, 3]` | ❌ śmieci zignorowane |
| `"nan 0 0"` | `[nan, 0, 0]` | ❌ **NaN wchodzi do modelu** |
| `"inf 0 0"` | `[inf, 0, 0]` | ❌ **Inf wchodzi do modelu** |
| `"-inf 0 0"` | `[-inf, 0, 0]` | ❌ **−Inf wchodzi do modelu** |
| `"1e400 0 0"` | `[0, 0, 0]` | ❌ przepełnienie → cichy wektor zerowy |
| `"  1 2 3"` | `[1, 2, 3]` | ✅ spacje wiodące OK |
| `"1 2 3  "` | `[1, 2, 3]` | ✅ spacje końcowe OK |
| `"1   2   3"` | `[1, 2, 3]` | ✅ wiele spacji OK |
| `"1\t2\t3"` | `[1, 0, 0]` | ⚠️ osiągalne **tylko przez `&#x9;`** — patrz §3.1 |
| `"+1 +2 +3"` | `[0, 0, 0]` | ❌ **`from_chars` odrzuca wiodący `+`** |
| `"1.5e2 0 0"` | `[150, 0, 0]` | ✅ zapis naukowy OK |
| `"-0 0 0"` | `[-0, 0, 0]` | ✅ akceptowane jako wektor zerowy |

Dwa wyniki, których prompt nie przewidział, a które wychodzą z pomiaru — z ważnym zastrzeżeniem metodologicznym przy pierwszym:

- **`std::from_chars` dla `double` odrzuca wiodący `+`** (zgodnie ze standardem — `from_chars` nie akceptuje znaku plus). `"+1 +2 +3"` daje `[0,0,0]`. Zweryfikowane **przez pełną ścieżkę plik → pugixml → `parse_xyz`**: XML nie transformuje znaku `+`, więc jest to realny błąd publicznego loadera.

- **Tabulator nie jest białym znakiem dla tej pętli** (pomija wyłącznie `' '`), ale **nie jest to udowodniony błąd publicznego loadera** — patrz niżej.

### 3.1 Sprostowanie: tabulatory a warstwa XML

Powyższa tabela pochodzi z bezpośredniego wywołania `parse_xyz`, czyli **z pominięciem parsera XML**. Review słusznie zakwestionowało wniosek o tabulatorach. Sprawdziłem pełną ścieżkę na prawdziwym pliku:

| zapis w pliku URDF | co dociera do `parse_xyz` | wynik |
|---|---|---|
| `xyz="1<TAB>2<TAB>3"` (literalny tabulator) | **`"1 2 3"`** | `[1,2,3]` — działa |
| `rpy="4&#x9;5&#x9;6"` (referencja znakowa) | **`"4\t5\t6"`** | `[4,0,0]` — psuje się |
| `rpy="1<NEWLINE>2<NEWLINE>3"` | **`"1  2   3"`** | `[1,2,3]` — działa |

Powód: XML wykonuje **normalizację wartości atrybutów** — literalne tabulatory, `LF` i `CR` są zamieniane na spacje (pugixml robi to domyślnie przez `parse_wconv_attribute`). Referencje znakowe (`&#x9;`) podlegają innym regułom i **przechodzą surowo**.

Wniosek: obsługa wszystkich białych znaków w `parseVector3` zostaje (jest defensywna i tania), ale **uzasadnienie się zmienia** — nie jest to naprawa błędu widocznego dla literalnych tabulatorów, tylko dla zapisu przez referencję znakową. Test musi używać `&#x9;`, inaczej przeszedłby również na obecnej implementacji i nie sprawdzałby niczego (§14.2).

**Odnotowuję metodologicznie:** to drugi raz w tym projekcie, gdy weryfikacja komponentu w izolacji dała wniosek nieprawdziwy dla pełnej ścieżki (poprzednio: build bez flag `-static` z `CMakeLists.txt`). Pozostałe pozycje tabeli w §3 zostały ponownie sprawdzone przez XML tam, gdzie warstwa ta mogła coś zmienić.

Precyzyjne wyjaśnienie zachowania przy przepełnieniu: `from_chars` przy `errc::result_out_of_range` **nie zapisuje** wartości wyjściowej. Składowa zostaje więc przy domyślnym `0`, a `break` zostawia w zerach także pozostałe. Stąd `"1e400 0 0"` → `[0,0,0]`.

## 4. Zakres i non-goals

**W zakresie:** rygorystyczne parsowanie `Vector3`, walidacja skończoności `origin`, walidacja i normalizacja `axis`, domyślna oś zgodna z URDF, rozszerzenie modelu błędów, testy.

**Poza zakresem:** `JointTransformBuilder`, `ForwardKinematicsBuilder`, `ExpressionEvaluator`, `EquationSimplifier`, `ConstraintBuilder`, solver IK, generator kodu, TCP, limity jako ograniczenia ruchu, walidacja topologii (to `KinematicChainBuilder`).

**`Continuous` — poza zakresem, z uzasadnieniem w §11.4.**

## 5. Semantyka brakujących i błędnych atrybutów

Kluczowe rozróżnienie, którego dziś nie ma: **brak** to nie to samo co **uszkodzony**.

| Zapis w URDF | Znaczenie | Zachowanie docelowe |
|---|---|---|
| brak elementu `<origin>` | domyślny origin | `translation = [0,0,0]`, `rpy = [0,0,0]` |
| `<origin/>` | oba atrybuty domyślne | j.w. |
| `<origin xyz="1 0 0"/>` | brak `rpy` | `rpy = [0,0,0]`, `translation` z parsowania |
| `<origin rpy="0 0 1"/>` | brak `xyz` | `translation = [0,0,0]`, `rpy` z parsowania |
| `<origin xyz=""/>` | atrybut **obecny, pusty** | **błąd** — `malformed_vector` |
| `<origin xyz="1 abc 3"/>` | atrybut uszkodzony | **błąd** — `malformed_vector` |
| brak `<axis>`, joint aktywny | domyślna oś wg URDF | `[1, 0, 0]` (§9) |
| **`<axis/>`** (element jest, brak `xyz`), joint aktywny | **brakujący atrybut**, nie błędna wartość | `[1, 0, 0]` — tak samo jak brak elementu |
| `<axis xyz=""/>`, joint aktywny | atrybut **obecny, pusty** | **błąd** — `malformed_vector` |
| brak `<axis>`, joint `Fixed` | oś nieistotna | brak błędu, wartość dowolna |
| **`<axis/>`**, joint `Fixed` | oś nieistotna | **brak błędu** |
| `<axis xyz=""/>`, joint `Fixed` | atrybut obecny, pusty | **błąd** — `malformed_vector` (§10: składnia walidowana zawsze) |
| `<axis xyz="0 0 0"/>`, aktywny | jawnie zdegenerowana | **błąd** — `degenerate_axis` |
| `<axis xyz="0 0 0"/>`, `Fixed` | oś nieistotna | **brak błędu** |

**Uzasadnienie dla `<axis/>`** (przypadek nierozstrzygnięty w v1, wskazany w review): traktujemy go **identycznie jak brak elementu**, czyli wartością domyślną. Rozstrzyga o tym ta sama zasada, co dla `<origin/>`: **brakujący atrybut oznacza wartość domyślną, obecny-ale-pusty oznacza uszkodzenie**. `<axis/>` nie podaje błędnej wartości — nie podaje żadnej. Rozróżnienie realizuje się przez `pugi::xml_attribute` (`if (!a)` = nieobecny) zamiast `as_string()`, które zwraca `""` w obu sytuacjach.

Rozróżnienie „brak atrybutu" od „pusty atrybut" wymaga rozdzielenia dwóch pytań, których dzisiejszy kod nie rozdziela (`as_string()` zwraca `""` w obu przypadkach):

```cpp
pugi::xml_attribute a = origin.attribute("xyz");
if (!a)            → atrybut nieobecny  → wartość domyślna
else               → parsuj a.value(); pusty string jest błędem
```

## 6. Projekt rygorystycznego parsera `Vector3`

### 6.1 Sygnatura

```cpp
// robot_model_loader.cpp, przestrzeń anonimowa — NIE w publicznym API
std::optional<Vec3> parseVector3(std::string_view text) noexcept;
```

`std::optional`, nie `std::expected<Vec3, ...>`: parser ma dokładnie **jeden** tryb porażki („to nie są trzy poprawne liczby"), a rozróżnianie „zły token" od „za mało tokenów" nie zmieniałoby zachowania żadnego klienta. Kontekst diagnostyczny (nazwa jointu, atrybut, surowy tekst) dokłada warstwa wołająca — patrz §12.

### 6.2 Reguły

Przyjmuje dokładnie trzy liczby zmiennoprzecinkowe rozdzielone białymi znakami:

```
whitespace* number whitespace+ number whitespace+ number whitespace*
```

| Reguła | Rozstrzygnięcie | Uzasadnienie |
|---|---|---|
| dokładnie trzy wartości | wymagane | mniej lub więcej to błąd danych, nie intencja |
| białe znaki | `' '`, `'\t'`, `'\n'`, `'\r'` | defensywnie — literalne tabulatory i nowe linie normalizuje już XML, ale referencja `&#x9;` przechodzi surowo (§3.1) |
| wiodące/końcowe białe znaki | dozwolone | powszechne w plikach generowanych |
| wiodący `+` | **dozwolony** | naprawa udowodnionego błędu loadera (§3, zweryfikowane przez XML); `from_chars` sam go nie przyjmuje, więc trzeba go pominąć ręcznie |
| zapis naukowy | dozwolony | `from_chars` obsługuje; już używane w `kr4_r600.urdf` (`1.2246467991473532e-16`) |
| `-0` | dozwolone | `-0.0 == 0.0`; bit znaku **nie jest** częścią kontraktu (§14.2) |
| `nan`, `inf`, `-inf` | **odrzucane** | sprawdzenie `std::isfinite` na każdej sparsowanej wartości |
| przepełnienie (`1e400`) | **odrzucane** | `from_chars` zwraca `result_out_of_range` |
| pusty tekst | **odrzucany** | atrybut obecny, ale bez wartości = uszkodzony (§5) |
| śmieci po trzeciej wartości | **odrzucane** | po sparsowaniu trzeciej dozwolone są wyłącznie białe znaki do końca |

**Skończoność sprawdzana wewnątrz parsera, nie osobno.** Konsekwencja projektowa: po przejściu przez `parseVector3` **wszystkie wartości są z konstrukcji skończone**, więc osobne kody błędu `NonFiniteOrigin`/`NonFiniteAxis` są nieosiągalne i ich nie tworzymy (§12).

### 6.3 Reguła znaku — dokładnie jeden, opcjonalny

Dopuszczenie wiodącego `+` (§6.2) nie może być zrealizowane naiwnym pominięciem znaku przed `from_chars`. Sprawdziłem, co taka implementacja akceptuje:

```cpp
if (*p == '+') ++p;                    // naiwne
std::from_chars(p, end, value);
```

| wejście | naiwna implementacja |
|---|---|
| `"+1"`, `"-1"`, `"1"` | akceptuje poprawnie ✅ |
| **`"+-1"`** | **akceptuje jako `-1`** ❌ |
| `"-+1"`, `"++1"`, `"--1"` | odrzuca ✅ |

Przecieka dokładnie jeden wariant: po pominięciu `+` `from_chars` widzi `"-1"` — poprawną liczbę.

**Reguła:** każda z trzech liczb może mieć **co najwyżej jeden** znak, będący `+` albo `-`. Po znaku musi bezpośrednio następować cyfra lub kropka. Implementacyjnie: pomijamy **najwyżej jeden** `+` i wymagamy, by kolejny znak nie był `+` ani `-` (znak `-` zostawiamy `from_chars`, które obsługuje go poprawnie).

Odrzucane: `"+-1 0 0"`, `"-+1 0 0"`, `"++1 0 0"`, `"--1 0 0"`.

## 7. Walidacja `origin`

Po zastosowaniu §5 i §6 walidacja `origin` **nie potrzebuje żadnego dodatkowego kroku**:

```
brak <origin>            → [0,0,0], [0,0,0]
brak atrybutu xyz/rpy    → [0,0,0] dla brakującego
atrybut obecny           → parseVector3; porażka → malformed_vector
```

Skończoność gwarantuje parser. **`rpy` nie jest normalizowane ani ograniczane do żadnego zakresu**, i nie ma konwersji ze stopni — URDF specyfikuje radiany, a loader ma przepisywać, nie interpretować. Potwierdzone pomiarem: `kr4_r600.urdf` niesie `3.1416` i `1.5708`, czyli π i π/2 w radianach (§2 poprzedniego proposalu).

## 8. Walidacja i normalizacja osi

### 8.1 Pipeline

```
atrybut xyz obecny?  ── nie ──→  joint aktywny → [1,0,0] (§9)
       │                         joint Fixed   → wartość nieistotna
      tak
       ↓
parseVector3  ── porażka ──→  malformed_vector
       ↓
joint Fixed?  ── tak ──→  zapisz bez dalszej obróbki, koniec
       ↓ nie
scale = max(|x|, |y|, |z|)
       ↓
scale == 0.0  ── tak ──→  degenerate_axis
       ↓ nie
(sx, sy, sz) = (x/scale, y/scale, z/scale)      ← przeskalowanie: max składowa = ±1
       ↓
norm = hypot(sx, sy, sz)                         ← norma w [1, √3]
       ↓
zapisz [sx/norm, sy/norm, sz/norm]
```

Dwuetapowe skalowanie jest **konieczne**, nie optymalizacyjne — samo `hypot(x,y,z)` zawodzi w zakresie subnormalnym (§8.2).

### 8.2 Algorytm normalizacji: skalowanie przez największą składową, potem `std::hypot`

Rozstrzygnięcie oparte na pomiarze, przez **dwa** etapy — druga wersja powstała po tym, jak review obaliło pierwszą.

**Krok 1 — dlaczego nie naiwne `sqrt(x²+y²+z²)`:**

| wejście | `sqrt(x²+y²+z²)` → wynik |
|---|---|
| `[0, 1e200, 0]` | `inf` → **`[0, 0, 0]`** ❌ |
| `[1e200, 1e200, 0]` | `inf` ❌ |
| `[1e-200, 1e-200, 0]` | `0` → dzielenie przez zero ❌ |
| `[5e-324, 0, 0]` (denormal) | `0` ❌ |

Dwa tryby cichej porażki na **skończonym** wejściu: przepełnienie w kwadratach i niedomiar. Poprawna oś zamienia się w zdegenerowaną.

**Krok 2 — dlaczego samo `std::hypot` też nie wystarcza.** Pierwsza wersja tego proposalu rekomendowała `hypot(x,y,z)` bezpośrednio i twierdziła, że poradzi sobie z każdą niezerową osią. **To było nieprawdą**, wykazane w review i potwierdzone pomiarem:

| wejście | `hypot` bezpośrednio | norma wyniku | ze skalowaniem | norma wyniku |
|---|---|---|---|---|
| `[m, m, 0]` gdzie `m = denorm_min` | `[1, 1, 0]` | **1.4142** ❌ | `[0.7071, 0.7071, 0]` | 1.0000 ✅ |
| `[m, m, m]` | `[0.5, 0.5, 0.5]` | **0.8660** ❌ | `[0.5774, 0.5774, 0.5774]` | 1.0000 ✅ |
| `[m, 0, 0]` | `[1, 0, 0]` | 1.0000 ✅ | `[1, 0, 0]` | 1.0000 ✅ |
| `[DBL_MIN, DBL_MIN, 0]` | `[0.7071, 0.7071, 0]` | 1.0000 ✅ | j.w. | 1.0000 ✅ |
| `[1e200, 1e200, 0]` | `[0.7071, 0.7071, 0]` | 1.0000 ✅ | j.w. | 1.0000 ✅ |

Przyczyna: `hypot(m, m, 0)` zwraca dokładnie `m`, bo prawdziwa wartość `√2·m` **nie jest reprezentowalna** w zakresie subnormalnym — brakuje tam bitów mantysy. Dzielenie przez zaokrągloną normę daje wektor o normie `√2`, nie `1`. `hypot` chroni przed przepełnieniem i niedomiarem *podczas liczenia długości*, ale nie może wytworzyć precyzji, której nie ma w wyniku.

**Przyjęty algorytm — wstępne przeskalowanie:**

```cpp
const double scale = std::max({std::abs(x), std::abs(y), std::abs(z)});
if (scale == 0.0)
    return degenerate_axis;                    // jedyny przypadek odrzucenia (§8.3)

const double sx = x / scale, sy = y / scale, sz = z / scale;
const double norm = std::hypot(sx, sy, sz);    // norma leży w [1, √3]

axis = {sx / norm, sy / norm, sz / norm};
```

Po przeskalowaniu przynajmniej jedna składowa ma wartość dokładnie `±1`, więc druga norma mieści się w przedziale `[1, √3]` — daleko od obu krańców zakresu `double`. Ani przepełnienie, ani utrata precyzji w zakresie subnormalnym nie są możliwe.

**Kontrola: `scale == 0.0` zastępuje sprawdzenie `norm == 0.0`** — obie warunki są równoważne (największa składowa co do modułu jest zerem wtedy i tylko wtedy, gdy wszystkie są), ale skalowanie i tak wymaga tej wartości, więc sprawdzenie jest darmowe i musi wystąpić przed dzieleniem.

### 8.3 Tolerancja osi zerowej — dokładnie zero, bez progu

**Korekta względem `proposal-joint-transform-builder-architecture.md`, gdzie rekomendowałem próg `1e-12`.** Tamta rekomendacja opierała się na milczącym założeniu, że bardzo mała oś sprawia problem numeryczny. Pomiar (§8.2) pokazuje, że **z algorytmem ze skalowaniem nie sprawia** — nawet `[denorm_min, denorm_min, denorm_min]` normalizuje się poprawnie do `[0.5774, 0.5774, 0.5774]` o normie 1.

Skoro nie ma trybu porażki numerycznej, próg byłby **arbitralnym osądem intencji autora URDF**: dlaczego `1e-12` miałoby być błędem, a `1e-11` poprawną osią X? Nie da się tego obronić.

**Rozstrzygnięcie: odrzucamy wyłącznie `scale == 0.0`** (§8.2), co zachodzi wtedy i tylko wtedy, gdy wszystkie trzy składowe są `±0.0`. Reguła jest nie-arbitralna, bez magicznej stałej, i pokrywa jedyny przypadek, który faktycznie zawodzi.

### 8.4 Bez snappingu — normalizacja osi osiowych jest już dokładna

Prompt pyta, czy po normalizacji „przyciągać" bardzo małe składowe do zera. Sprawdziłem, czy jest taka potrzeba:

| wejście | po normalizacji | dokładnie osiowa? |
|---|---|---|
| `[0, 0, 1]` | `[0, 0, 1]` | ✅ |
| `[0, 0, 5]` | `[0, 0, 1]` | ✅ |
| `[0, 0, 3]`, `[0, 0, 7]` | `[0, 0, 1]` | ✅ |
| `[0, 0, 0.1]`, `[0, 0, 0.3]` | `[0, 0, 1]` | ✅ |
| `[0, 0, −5]` | `[0, 0, −1]` | ✅ |
| `[2, 0, 0]` | `[1, 0, 0]` | ✅ |

Dla wektora osiowego `hypot` zwraca dokładnie `|składowa|`, a dzielenie przez samego siebie daje dokładnie `1.0`. **Zero przypadków wymagających snappingu.** Wymaganie z promptu („`[0,0,5]` → dokładna oś Z") jest spełnione bez żadnej dodatkowej maszynerii.

**Rozstrzygnięcie: bez snappingu.** Konsekwencja pozytywna: fast path w `JointTransformBuilder` (porównanie składowych z `0.0` i `±1.0`) zadziała dla wszystkich osiowych wejść. Konsekwencja neutralna: oś `[1e-17, 0, 1]` **nie** zostanie uznana za osiową i pójdzie ogólnym Rodriguesem — co jest poprawne, bo to faktycznie oś minimalnie odchylona, a snapping po cichu zmieniłby geometrię.

## 9. Domyślna oś zgodna z URDF

Specyfikacja URDF dla `<axis>`: *„defaults to (1,0,0)"*. Obecny model deklaruje `{0.0, 0.0, 1.0}` — potwierdzone uruchomieniem: revolute bez `<axis>` dostaje `[0, 0, 1]`.

To **realna niezgodność**: ten sam plik URDF da w KinemaForge inną kinematykę niż w dowolnym referencyjnym parserze. Niewidoczna dziś tylko dlatego, że oba testowe roboty podają osie jawnie.

**Jedno źródło prawdy: parser.** Domyślna wartość nadawana jest tam, gdzie podejmowana jest decyzja „atrybut nieobecny → wartość domyślna" (§5), czyli w `load_urdf`. Domyślny inicjalizator w strukturze `Joint` zostaje zmieniony na `{1.0, 0.0, 0.0}` **dla spójności**, ale nie jest mechanizmem nadawania wartości — parser zawsze zapisuje oś jawnie dla jointów aktywnych.

Dla `Fixed` oś nie ma znaczenia i nie jest zapisywana ani walidowana.

## 10. Zachowanie dla `Fixed`

```
brak <axis>              → brak błędu, wartość pozostaje domyślna
<axis xyz="0 0 0"/>      → brak błędu; oś nie jest walidowana ani normalizowana
<axis xyz="1 abc 3"/>    → BŁĄD malformed_vector
```

Trzeci wiersz wymaga uzasadnienia, bo można by argumentować, że skoro oś nie jest używana, jej treść jest obojętna. Rozstrzygam inaczej: **uszkodzona składnia to zawsze błąd**, niezależnie od tego, czy wartość jest potem czytana. Milczące przyjęcie `"1 abc 3"` maskuje literówkę, którą autor prawdopodobnie chciał zobaczyć. Pomijamy walidację **semantyczną** (zerowość) dla `Fixed`, nie **składniową**.

## 11. Zachowanie dla `Revolute`, `Continuous`, `Prismatic`

### 11.1 Revolute i Prismatic

Pełny pipeline z §8: parsowanie → norma → odrzucenie zdegenerowanej → normalizacja → zapis.

### 11.2 Kontrakt wynikowy `RobotDescription`

Do udokumentowania w nagłówkach jako komentarz — i to komentarz, który po tym etapie staje się **prawdziwy**, w odróżnieniu od dzisiejszego `// unit vector in parent's local frame`:

```
Po pomyślnym load_urdf:
  * origin.translation i origin.rpy każdego jointu mają wyłącznie skończone wartości
  * dla jointu aktywnego axis jest skończona, niezerowa i NUMERYCZNIE znormalizowana
  * dla jointu Fixed axis nie niesie znaczenia
```

**Dwie poprawki do obecnego komentarza w `robot_model.hpp`, obie wskazane w review:**

**1. Zły układ odniesienia.** Dzisiejsze *„in parent's local frame"* jest **nieprawidłowe**. URDF specyfikuje oś **w układzie jointu**, czyli po zastosowaniu `origin` — i to jest dokładnie powód, dla którego `JointTransformBuilder` buduje `T_origin · Rotation(axis, q)` **bez** wcześniejszego obracania osi przez `origin.rpy`. Docelowo:

```cpp
Vec3 axis = {1.0, 0.0, 0.0};  // Unit vector expressed in the joint frame.
```

**2. „Jednostkowa" to kontrakt numeryczny, nie bitowy.** Rozróżnienie, którego pierwsza wersja tego dokumentu nie robiła:

| Rodzaj osi | Gwarancja |
|---|---|
| osiowa (`±X`, `±Y`, `±Z` po normalizacji) | **dokładna** — `hypot` zwraca dokładnie `|składowa|`, dzielenie daje bitowo `±1.0` i `0.0` (§8.4) |
| dowolna (np. `[1,2,3]`) | **numeryczna** — norma równa `1.0` z dokładnością do kilku ULP, nie zawsze bitowo |

Kontrakt brzmi więc „`axis` jest numerycznie znormalizowana", a **nie** „`hypot(x,y,z) == 1.0` jest zawsze prawdą". Konsekwencja dla przyszłego `assert` w `JointTransformBuilder`: musi używać tolerancji, np.

```cpp
assert(std::abs(std::hypot(a.x, a.y, a.z) - 1.0) <= tolerance);
```

Konkretna wartość `tolerance` (rzędu kilku `std::numeric_limits<double>::epsilon()`) należy do proposalu implementacyjnego `JointTransformBuilder`, nie tutaj — istotne jest, żeby kontrakt **nie obiecywał** dokładności bitowej, której dla osi dowolnych nie ma.

### 11.3 `Continuous` — stan i wpływ

Zweryfikowany (§2.5): **niedostępny**. `type="continuous"` odrzuca cały plik jako `unsupported_joint_type`.

### 11.4 Dlaczego `Continuous` nie wchodzi do tego proposalu

Prompt dopuszcza rozszerzenie zakresu tylko, jeśli jest *bezpośrednio konieczne do ustanowienia kontraktu loadera*. **Nie jest** — kontrakt z §11.2 jest w pełni ustanawialny dla `Revolute`, `Prismatic` i `Fixed`; dla `Continuous` byłby spełniony pusto, bo takiego jointu nie da się dziś wyprodukować.

Naprawa `Continuous` ciągnie natomiast za sobą **zmiany w semantyce limitów**, czyli obszarze niezwiązanym z geometrią:

- nowa wartość w `mt::kinematics::JointType` i przypadek w `parse_joint_type`,
- warunek `if (type != fixed) → wymagaj <limit>` staje się błędny (URDF nie wymaga `lower`/`upper` dla continuous),
- `RobotModel::within_limits` sprawdza `v < lower || v > upper` — dla continuous bez znaczenia; trzeba zdecydować, czy pomijać, czy trzymać sentinel,
- `hasPositionLimits = (type != Fixed)` w `UrdfModelLoader.cpp` przestaje być prawdziwe wraz ze swoim komentarzem,
- przypadek w `mapJointType`.

**Rekomendacja: osobny proposal**, ale odnotowany jako znana luka z konkretną konsekwencją — dopóki nie powstanie, `kinemaforge::ik::JointType::Continuous` pozostaje nieosiągalny, a gałąź `Continuous` w `JointTransformBuilder` da się przetestować wyłącznie ręcznie zbudowanym `KinematicJoint`.

## 12. Model błędów

### 12.1 Ile nowych kodów

Prompt wymienia sześć kandydatów. Analiza osiągalności po wprowadzeniu §6:

| Kandydat | Rozstrzygnięcie |
|---|---|
| `MalformedVector` | **nowy kod** — pokrywa złe tokeny, złą liczbę wartości, pusty atrybut, śmieci, przepełnienie **oraz `nan`/`inf`** (parser odrzuca je jako niepoprawne wartości) |
| `NonFiniteOrigin` | **nie tworzymy** — nieosiągalny; po `parseVector3` wszystkie wartości są skończone z konstrukcji |
| `NonFiniteAxis` | **nie tworzymy** — j.w. |
| `DegenerateAxis` | **nowy kod** — inna kategoria: składnia poprawna, semantyka zdegenerowana |
| `UnsupportedJointType` | istnieje |
| `InvalidJointLimit` | istnieje jako `invalid_limits` |

**Netto: dwa nowe kody.** To bezpośrednie zastosowanie wytycznej z promptu — nie mnożyć wartości enuma, jeśli różnią się wyłącznie opisem. `NonFiniteOrigin` i `NonFiniteAxis` różniłyby się od `MalformedVector` **niczym**, bo po rygorystycznym parserze nie mają jak wystąpić.

### 12.2 Enum czy struktura z kontekstem

Ocena obecnego użycia (§2.4): jedynym konsumentem `LoadError` jest `describe()` w `UrdfModelLoader`, który zamienia kod na tekst i rzuca wyjątek. **Żaden klient nie rozgałęzia się po kodzie.**

Sam kod jest jednak niewystarczający diagnostycznie. Komunikat „failed to parse URDF vector" dla pliku z 30 jointami nie mówi użytkownikowi nic użytecznego. Potrzebny jest kontekst: **który joint, który atrybut, jaka wartość**.

**Rekomendacja:**

```cpp
enum class LoadErrorCode : std::uint8_t {
    file_not_found, parse_failure, unsupported_joint_type,
    incomplete_kinematic_chain, invalid_limits,
    malformed_vector, degenerate_axis,
};

struct LoadError {
    LoadErrorCode code{};
    std::string   jointName;   // pusty, gdy nie dotyczy konkretnego jointu
    std::string   attribute;   // np. "origin/xyz", "axis/xyz"
    std::string   rawValue;    // surowy tekst atrybutu, przycięty (patrz niżej)
};
```

**Ograniczenie długości `rawValue` (uwaga z review).** Atrybut w uszkodzonym pliku może być dowolnie długi — nie ma powodu kopiować megabajta do wyjątku i logu. Przycinamy do **256 znaków**, z jawnym wielokropkiem sygnalizującym obcięcie:

```
rawValue = "1 abc 3"                    // krótki — bez zmian
rawValue = "0 0 0 0 0 0 0 …"            // przycięty do 256 znaków + "…"
```

256 zamiast 128, bo poprawny wektor trzech liczb w pełnej precyzji (`1.2246467991473532e-16` × 3 + separatory) mieści się w ~70 znakach — 256 daje spory zapas na zobaczenie *kontekstu* błędu, a nadal jest wartością nieistotną dla pamięci.

### 12.2a Konsekwencja dla `noexcept` — zadanie dla proposalu implementacyjnego

`LoadError` przestaje być trywialnym `uint8_t` i zaczyna zawierać `std::string`, czyli **może alokować przy konstrukcji i kopiowaniu**. Dzisiejsze funkcje w `robot_model_loader.cpp` są oznaczone `noexcept`:

```cpp
std::expected<JointType, LoadError> parse_joint_type(std::string_view) noexcept;
Vec3 parse_xyz(char const*) noexcept;
```

Jeśli po zmianie któraś z nich zacznie budować `LoadError` z nazwą jointu lub surowym tekstem, `std::bad_alloc` wewnątrz `noexcept` **zakończy program przez `std::terminate`** zamiast propagować błąd.

To nie wymaga zmiany architektury — wymaga świadomej decyzji przy implementacji. **Do planu proposalu implementacyjnego:** przejrzeć wszystkie funkcje `noexcept` w ścieżce błędu i albo zdjąć `noexcept` z tych, które konstruują `LoadError` z kontekstem, albo przenieść budowę kontekstu do warstwy wołającej, która `noexcept` nie ma.

Uzasadnienie: kategoria zostaje w enumie (klient **może** kiedyś rozgałęziać), a kontekst — jedyne, co realnie poprawia diagnostykę — dochodzi jako dane. Rozdzielenie na osobne kody dla każdego atrybutu byłoby gorszym rozwiązaniem tego samego problemu.

**Koszt do odnotowania:** `LoadError` przestaje być trywialnie kopiowalnym `uint8_t`. `std::expected<LoadResult, LoadError>` nadal działa, ale typ błędu robi się „ciężki". Przy skali (błąd powstaje raz, przy porażce ładowania) bez znaczenia.

**Rozważona i odrzucona alternatywa:** przekazanie kontekstu przez istniejący `DiagnosticBag`. Odrzucona, bo `LoadResult` (a więc i worek) istnieje **wyłącznie na ścieżce sukcesu** — przy błędzie `std::expected` zwraca sam `LoadError`. Naprawa wymagałaby zmiany sygnatury na coś w stylu `std::expected<LoadResult, std::pair<LoadError, DiagnosticBag>>`, co jest gorsze od struktury.

### 12.3 Gdzie powstaje tekst

Bez zmian względem dziś: **warstwa `mt::kinematics` nie formatuje komunikatów**, zwraca dane. Tekst powstaje w `UrdfModelLoader::load`, gdzie `describe()` rozszerza się o kontekst:

```
UrdfModelLoader: malformed vector in joint 'joint_4', attribute 'axis/xyz': "1 abc 3" (data/urdf/broken.urdf)
```

Zachowuje to obecny podział: dolna warstwa jest niezależna od prezentacji, górna składa komunikat dla użytkownika.

## 13. Granica odpowiedzialności

| Warstwa | Odpowiedzialność | Zmiana w tym etapie |
|---|---|---|
| `mt::kinematics::load_urdf` | parsowanie, walidacja składni i wartości, **normalizacja osi**, kanoniczny `RobotModel` | **rozszerzana** |
| `UrdfModelLoader` | adaptacja do `RobotDescription`, komunikaty błędów | rozszerzany o kontekst błędu |
| `KinematicChainBuilder` | walidacja **topologii**, przypisanie zmiennych symbolicznych | bez zmian |
| `JointTransformBuilder` | matematyka transformacji, `assert` na inwariantach | bez zmian (jeszcze nie istnieje) |

Weryfikacja, czy obecny podział to umożliwia: **tak.** `load_urdf` już dziś jest jedynym miejscem czytającym XML i już zwraca `std::expected` z kodem błędu. Normalizacja osi wchodzi tam naturalnie, obok istniejącej walidacji limitów. Żadna klasa nie musi zostać rozbita ani przeniesiona.

**Normalizacja występuje dokładnie raz**, w `load_urdf`. `copyJointData` w `KinematicChainBuilder` kopiuje już kanoniczną wartość.

## 14. Projekt testów

### 14.1 Strategia dla wejść uszkodzonych

Cztery rozważane warianty:

| Wariant | Ocena |
|---|---|
| pliki w `data/urdf/` | Zaśmieca katalog z prawdziwymi robotami; test odsyła do pliku, więc czytelnik nie widzi wejścia |
| osobny `tests/data/invalid/` | Lepsze, ale nadal ~15 plików i skok do innego pliku przy czytaniu testu |
| **plik tymczasowy pisany przez helper RAII** *(rekomendowane)* | Uszkodzony XML stoi **w treści testu**, więc test jest samowyjaśniający; brak plików w repo; sprzątanie w destruktorze |
| `loadFromString()` w API produkcyjnym | Odrzucone — prompt słusznie ostrzega; nie ma dla tego realnego przypadku użycia poza testami |

Szkic helpera (w pliku testowym, nie w API):

```cpp
class TemporaryUrdf {
public:
    explicit TemporaryUrdf(std::string_view contents);   // zapisuje do unikalnej ścieżki tymczasowej
    ~TemporaryUrdf();                                    // usuwa plik
    const std::filesystem::path& path() const;
};
```

Pozwala to testować przez **publiczne zachowanie loadera**, bez udostępniania `parseVector3` tylko dla testów — czyli zgodnie z preferencją z promptu.

### 14.2 Parsowanie wektorów

Testowane przez loader, na minimalnym URDF różniącym się jednym atrybutem.

| Test | Wejście (atrybut `origin/xyz`) | Oczekiwanie | Wykrywany błąd |
|---|---|---|---|
| `ParsesValidVector3` | `"1 2 3"` | sukces, `[1,2,3]` | regresja podstawowa |
| `AcceptsLeadingAndTrailingWhitespace` | `"  1 2 3  "` | sukces, `[1,2,3]` | zbyt gorliwe odrzucanie |
| `AcceptsCharacterReferenceTabSeparators` | w pliku: `<origin xyz="1&#x9;2&#x9;3"/>` | sukces, `[1,2,3]` | **musi używać encji** — literalny tabulator XML normalizuje do spacji, więc test z nim przeszedłby też na starej implementacji i nie chroniłby niczego (§3.1) |
| `AcceptsLeadingPlusSign` | `"+1 +2 +3"` | sukces, `[1,2,3]` | **wykryta luka §3** — dziś daje `[0,0,0]` |
| `RejectsMultipleLeadingSigns` | `"+-1 0 0"` | `malformed_vector` | **naiwne pominięcie `+` przepuszcza to jako `-1`** — §6.3; jedyny wariant, który przecieka |
| `AcceptsScientificNotation` | `"1.5e2 0 0"` | sukces, `[150,0,0]` | regresja |
| `AcceptsNegativeZero` | `"-0 0 0"` | **sukces**; wektor traktowany jak zerowy | odrzucanie poprawnego zapisu. Test **nie** przypina bitu znaku — `-0.0 == 0.0`, a późniejsze etapy nie mogą od niego zależeć |
| `RejectsMalformedVectorText` | `"1 abc 3"` | `malformed_vector` | ciche obcięcie |
| `RejectsMissingVectorComponent` | `"1 2"` | `malformed_vector` | dopełnianie zerem |
| `RejectsExtraVectorComponent` | `"1 2 3 4"` | `malformed_vector` | ignorowanie nadmiaru |
| `RejectsTrailingGarbage` | `"1 2 3 abc"` | `malformed_vector` | j.w. |
| `RejectsEmptyVectorAttribute` | `""` | `malformed_vector` | nieodróżnialne od `"0 0 0"` |
| `RejectsOverflowingVectorComponent` | `"1e400 0 0"` | `malformed_vector` | cichy wektor zerowy |
| `RejectsNaNVectorComponent` | `"nan 0 0"` | `malformed_vector` | NaN w modelu |
| `RejectsPositiveInfinityVectorComponent` | `"inf 0 0"` | `malformed_vector` | Inf w modelu |
| `RejectsNegativeInfinityVectorComponent` | `"-inf 0 0"` | `malformed_vector` | −Inf w modelu |

### 14.3 Origin

| Test | Wejście | Oczekiwanie |
|---|---|---|
| `DefaultsMissingOriginToZero` | brak `<origin>` | `translation` i `rpy` = `[0,0,0]` |
| `DefaultsMissingOriginXyzToZero` | `<origin rpy="0 0 1"/>` | `translation = [0,0,0]`, `rpy = [0,0,1]` |
| `DefaultsMissingOriginRpyToZero` | `<origin xyz="1 0 0"/>` | `translation = [1,0,0]`, `rpy = [0,0,0]` |
| `KeepsRpyInRadians` | `<origin rpy="3.14159 0 0"/>` | `rpy.x ≈ 3.14159`, bez konwersji |

### 14.3a Testy podłączenia parsera — po jednym na miejsce użycia

**Korekta po review.** Pierwsza wersja uznawała `RejectsNonFiniteOriginTranslation` / `RejectsNonFiniteOriginRpy` / `RejectsNonFiniteAxis` za zbędne, bo „wszystko idzie przez ten sam `parseVector3`". Na poziomie funkcji to prawda — **na poziomie integracji nie**.

Pełna tabela patologii z §14.2 przechodzi przez **jeden** atrybut (`origin/xyz`). Potwierdza to, że nowy parser istnieje i jest podłączony **w tym jednym miejscu**. Nie potwierdza, że autor implementacji pamiętał o zastąpieniu starego `parse_xyz` również w dwóch pozostałych wywołaniach (§2.1). Implementacja podmieniająca tylko pierwsze wywołanie przeszłaby cały §14.2.

Dlatego: **pełna tabela patologii dla jednego miejsca + po jednym teście integracyjnym dla każdego pozostałego.**

| Test | Wejście | Wykrywany błąd |
|---|---|---|
| `RejectsMalformedOriginRpy` | `<origin rpy="1 abc 3"/>` | **parser niepodłączony do `origin/rpy`** |
| `RejectsMalformedAxis` | `<axis xyz="1 abc 3"/>`, joint aktywny | **parser niepodłączony do `axis/xyz`** |

Dwa testy, nie trzy — `origin/xyz` jest już w pełni pokryte całą tabelą z §14.2, więc osobny `RejectsMalformedOriginTranslation` byłby jej dokładnym duplikatem (uwaga z review). Pozostałe dwa miejsca użycia dostają po jednym teście, bez mnożenia wszystkich wariantów (`nan`, `inf`, przepełnienie, zła liczba składowych) razy trzy.

### 14.4 Oś

| Test | Wejście | Oczekiwanie | Wykrywany błąd |
|---|---|---|---|
| `DefaultsMissingAxisToXForActuatedJoint` | revolute bez `<axis>` | `[1,0,0]` | **wykryta niezgodność §9** — dziś `[0,0,1]` |
| `DefaultsEmptyAxisElementToXForActuatedJoint` | revolute z `<axis/>` (element bez atrybutu) | `[1,0,0]` | mylenie „brak atrybutu" z „pusta wartość" — §5 |
| `RejectsEmptyAxisAttribute` | revolute z `<axis xyz=""/>` | `malformed_vector` | j.w., od drugiej strony |
| `RejectsZeroAxisForRevoluteJoint` | `<axis xyz="0 0 0"/>`, revolute | `degenerate_axis` | oś zerowa w pipeline |
| `RejectsZeroAxisForPrismaticJoint` | j.w., prismatic | `degenerate_axis` | j.w. |
| `IgnoresZeroAxisForFixedJoint` | j.w., fixed | **sukces** | odrzucanie poprawnych danych |
| `RejectsMalformedAxisForFixedJoint` | `<axis xyz="1 abc 3"/>`, fixed | `malformed_vector` | §10 — składnia walidowana zawsze |
| `NormalizesNonUnitXAxis` | `[2,0,0]` | dokładnie `[1,0,0]` | brak normalizacji |
| `NormalizesNonUnitYAxis` | `[0,3,0]` | dokładnie `[0,1,0]` | j.w. |
| `NormalizesNonUnitZAxis` | `[0,0,5]` | dokładnie `[0,0,1]` | j.w.; `EXPECT_DOUBLE_EQ`, bo §8.4 gwarantuje dokładność |
| `NormalizesNegativeAxis` | `[0,0,−5]` | dokładnie `[0,0,−1]` | zgubienie znaku |
| `NormalizesArbitraryAxis` | `[1,2,3]` | `[0.2673, 0.5345, 0.8018]`, norma = 1 | zły wzór |
| `NormalizesVeryLargeAxis` | `[1e200, 0, 0]` | `[1,0,0]` | **naiwne `sqrt` daje `[0,0,0]`** — §8.2 |
| `NormalizesVerySmallAxis` | `[1e-200, 0, 0]` | `[1,0,0]` | **naiwne `sqrt` daje dzielenie przez zero** — §8.2 |
| `NormalizesMultiComponentSubnormalAxis` | `[denorm_min, denorm_min, 0]` | `[1/√2, 1/√2, 0]`, norma `1.0 ± ULP` | **samo `hypot` daje `[1,1,0]` o normie √2** — §8.2. Test z jedną niezerową składową tego **nie** wykryje, bo dzieli się przez samą siebie |
| `NormalizesFullySubnormalAxis` | `[denorm_min, denorm_min, denorm_min]` | `[1/√3 ×3]`, norma `1.0 ± ULP` | j.w.; samo `hypot` daje `[0.5, 0.5, 0.5]` o normie 0.866 |
| `KeepsAlreadyUnitAxisBitExact` | `[0,0,1]` | dokładnie `[0,0,1]` | wprowadzenie błędu numerycznego tam, gdzie go nie było |

Dwa testy wielkości/małości są bezpośrednim zabezpieczeniem decyzji z §8.2 — bez nich implementacja z naiwnym `sqrt` przeszłaby cały pozostały zestaw.

### 14.5 Regresja na rzeczywistych robotach

| Test | Oczekiwanie |
|---|---|
| `KeepsKr640GeometryValid` | ładuje się; wszystkie 6 osi aktywnych jednostkowych i **bitowo identycznych** z dzisiejszymi (`[0,0,1]`, `[0,1,0]`, `[1,0,0]`) |
| `KeepsKr4GeometryValid` | j.w.; wszystkie osie `[0,0,1]`; `rpy` bez zmian |

### 14.6 `Continuous`

| Test | Oczekiwanie |
|---|---|
| `ReportsContinuousAsUnsupportedUntilImplemented` | `unsupported_joint_type`. **Test jawnie tymczasowy** — z komentarzem `TODO: usunąć wraz z proposalem obsługi Continuous (§11.4)`. Nazwa i komentarz mają nie pozwolić, żeby znana luka została kiedyś wzięta za docelowe zachowanie |

Test `RejectsZeroAxisForContinuousJoint` z listy w promptcie **nie może powstać** — plik z `type="continuous"` jest odrzucany zanim dojdzie do walidacji osi (§2.5).

## 15. Plan zmian w plikach

### Zmienione

| Plik | Zmiana |
|---|---|
| `src/kinematics/robot_model_loader.cpp` | `parse_xyz` → `parseVector3` (rygorystyczny); rozdzielenie „atrybut nieobecny" od „pusty"; **normalizacja osi przez skalowanie względem największej składowej, a następnie `std::hypot`** (§8.1/§8.2 — samo `hypot` nie wystarcza); odrzucenie osi przy `scale == 0.0`; domyślna oś `[1,0,0]`; budowa `LoadError` z kontekstem |
| `src/kinematics/robot_model_loader.hpp` | `LoadError` enum → `LoadErrorCode` + struktura `LoadError`; aktualizacja sygnatury `load_urdf` |
| `src/kinematics/robot_model.hpp` | domyślna `axis` `{0,0,1}` → `{1,0,0}`; komentarz „unit vector" staje się egzekwowanym kontraktem |
| `src/ik_equations/UrdfModelLoader.cpp` | `describe()` obsługuje nowe kody i **formatuje kontekst** (joint, atrybut, wartość) |
| `src/ik_equations/model/RobotDescription.hpp` | komentarz dokumentujący kontrakt kanoniczny (§11.2) |
| `tests/test_urdf_model_loader.cpp` | testy z §14 + helper `TemporaryUrdf` |

### Bez zmian

`robot_model.cpp`, `UrdfModelLoader.hpp` (sygnatura `load` bez zmian — nadal rzuca), `KinematicChain.hpp`, `KinematicChainBuilder.*`, cała warstwa symboliczna, `test_kinematics.cpp`, `test_kinematic_chain_builder.cpp`.

### CMake

**Bez zmian w żadnym z dwóch plików** — nie powstają nowe jednostki translacji ani nowe pliki testowe. Wszystko mieści się w istniejących `robot_model_loader.cpp` i `test_urdf_model_loader.cpp`.

## 16. Migracja istniejących URDF-ów

Zweryfikowane pomiarem na obu robotach:

| Pytanie | Odpowiedź |
|---|---|
| Czy osie pozostaną numerycznie identyczne? | **Tak, bitowo.** Wszystkie osie w KR640 (`[0,0,1]`, `[0,1,0]`, `[1,0,0]`) i KR4 (`[0,0,1]`) mają już normę dokładnie `1.0`; dzielenie przez `1.0` jest dokładne |
| Czy zmiana domyślnej osi wpłynie na testy? | **Nie.** Oba roboty podają `<axis>` jawnie dla każdego jointu aktywnego. Zmiana domyślnej wartości jest niewidoczna dla obecnych fixture'ów |
| Czy któryś fixture ma aktywny joint bez `<axis>`? | **Nie** — sprawdzone w obu plikach |
| Czy normalizacja zmieni wartości `double`? | **Nie** dla tych danych (norma = 1.0). Dla hipotetycznego `[0,0,5]` zmieniłaby, ale takiego przypadku w fixture'ach nie ma |
| Czy testy porównują osie dokładnie? | **Tak** — `EXPECT_DOUBLE_EQ(joint.axis.z, 1.0)` w `MapsRevoluteJointFields`, `MapsKr4JointAxis`. Pozostaną zielone bez zmian |
| Czy nowe kody błędów zmieniają publiczne API? | **Warstwy `ik` — nie.** `UrdfModelLoader::load` nadal rzuca `std::runtime_error`, zmienia się wyłącznie treść komunikatu. **Warstwy `mt::kinematics` — tak**, `LoadError` zmienia kształt, ale jedynym konsumentem jest `UrdfModelLoader` w tym samym repo |

**Wniosek: zerowy wpływ na istniejące testy i dane.** Wszystkie 70 testów powinno pozostać zielonych bez modyfikacji.

## 17. Ryzyka i trade-offy

| Ryzyko | Ocena | Mitygacja |
|---|---|---|
| Rygorystyczny parser odrzuci URDF, który dziś się ładuje | Realne dla plików z tabulatorami lub `+` — ale te **dziś są cicho psute**, nie ładowane poprawnie. Odrzucenie jest poprawą | Tabulatory i `+` jawnie **dozwolone** (§6.2), więc realny zbiór odrzuceń to wyłącznie dane faktycznie uszkodzone |
| `LoadError` jako struktura zamiast enuma | Cięższy typ w `std::expected` | Bez znaczenia — powstaje raz, na ścieżce porażki |
| Zmiana domyślnej osi zmieni zachowanie istniejących plików | Zerowe dla naszych fixture'ów (§16); realne dla zewnętrznych URDF-ów bez `<axis>` | To **naprawa niezgodności**, nie regresja — dziś dajemy inny wynik niż każdy referencyjny parser |
| Helper `TemporaryUrdf` zapisuje pliki podczas testów | ~20 plików na przebieg; na Windows z antywirusem może spowolnić | Alternatywa (`tests/data/invalid/`) odnotowana w §14.1; łatwa zamiana, gdyby okazało się to uciążliwe |
| Brak `Continuous` blokuje pełne przetestowanie pipeline'u | Realne, odnotowane | Osobny proposal (§11.4); test `ReportsContinuousAsUnsupportedUntilImplemented` z komentarzem TODO utrwala stan jako **tymczasowy** |
| Naiwna implementacja normy przejdzie większość testów | **Wysokie** — `sqrt(x²+y²+z²)` zawodzi tylko dla skrajnych wielkości | Dwa dedykowane testy (§14.4) celujące dokładnie w te przypadki |

## 18. Otwarte pytania

Żadna z decyzji tego dokumentu nie wymaga rozstrzygnięcia spoza kodu i specyfikacji URDF. Trzy kwestie warte odnotowania, ale nieblokujące:

1. **Czy `DiagnosticBag` powinien być dostępny również na ścieżce porażki?** Dziś ginie przy błędzie (§2.4). Nie blokuje tego etapu — kontekst błędu niesie struktura `LoadError` — ale to realne ograniczenie, gdyby kiedyś chciano zwracać ostrzeżenia razem z błędem.
2. **Kolejność prac dla `Continuous`** — przed czy po `JointTransformBuilder`? Rekomendacja: po, bo builder i tak obsłuży `Continuous` na poziomie kodu, a jego test przejdzie na ręcznie budowanym joincie.
3. **Czy `tests/data/invalid/` byłby lepszy od plików tymczasowych?** Zależy od tego, jak zachowa się antywirus przy ~20 zapisach na przebieg testów. Decyzja odwracalna, nie architektoniczna.

## 19. Rekomendacja końcowa

```
approve
```

### Rozstrzygnięte na podstawie kodu, pomiaru i specyfikacji URDF

Rygorystyczny parser trzech wartości, z **dokładnie jednym opcjonalnym znakiem** na liczbę (§6.3) i dopuszczeniem wszystkich białych znaków; rozdzielenie „atrybut nieobecny" od „pusty"; skończoność sprawdzana **wewnątrz** parsera, przez co `NonFiniteOrigin`/`NonFiniteAxis` są nieosiągalne i nie powstają; **normalizacja przez skalowanie największą składową, a następnie `std::hypot`** — samo `hypot` zawodzi dla wartości subnormalnych; odrzucanie osi wyłącznie przy `scale == 0.0`, bez arbitralnego progu; brak snappingu; domyślna oś `[1,0,0]` nadawana w parserze, także dla `<axis/>` bez atrybutu (§5); `Fixed` bez walidacji semantycznej osi, ale z walidacją składniową; dwa nowe kody błędu plus kontekst diagnostyczny w strukturze z przyciętym `rawValue`; walidacja i normalizacja dokładnie raz, w `load_urdf`.

### Korekty względem wcześniejszych wersji

| Gdzie | Było | Jest | Powód |
|---|---|---|---|
| algorytm normalizacji | `sqrt(x²+y²+z²)` (stan obecny) → `std::hypot(x,y,z)` (v1 tego dokumentu) | **skalowanie przez największą składową, potem `hypot`** | `sqrt` zawodzi na przepełnieniu i niedomiarze; **samo `hypot` też zawodzi** w zakresie subnormalnym — `[denorm_min, denorm_min, 0]` daje wektor o normie √2. Wykryte w review, potwierdzone pomiarem (§8.2) |
| tolerancja osi zerowej | próg `1e-12` (`proposal-joint-transform-builder-architecture.md` §9.3) | **dokładnie `scale == 0.0`** | Z algorytmem ze skalowaniem nie ma trybu porażki numerycznej dla osi niezerowej. Próg byłby arbitralnym osądem intencji |
| tabulatory | „udowodniony błąd loadera" (v1) | **osiągalne wyłącznie przez `&#x9;`** | Weryfikowałem `parse_xyz` w izolacji; XML normalizuje literalne tabulatory do spacji. Reguła parsera zostaje, uzasadnienie i test się zmieniają (§3.1) |
| jednostkowość osi | sugerowana bitowa | **kontrakt numeryczny**, dokładny tylko dla osi osiowych | Dla `[1,2,3]` norma to `1.0 ± kilka ULP`; przyszły `assert` musi mieć tolerancję (§11.2) |
| `-0` | test przypinający bit znaku | **tylko akceptacja zapisu** | `-0.0 == 0.0`; bit znaku nie może być kontraktem, bo blokowałby przyszłą kanonizację |

Dwie pierwsze wprowadzam wbrew własnym wcześniejszym rekomendacjom. W obu przypadkach zawiodła ta sama metoda: **testowanie w izolacji zamiast pełnej ścieżki** — raz funkcji parsującej bez warstwy XML, raz `hypot` na wejściach z jedną niezerową składową, które maskują błąd, bo dzielą się przez samą siebie.

### Zależność

Ten proposal jest **prerequisite** dla implementacji `JointTransformBuilder`. Kolejność: ten dokument → jego proposal implementacyjny → proposal implementacyjny `JointTransformBuilder` → implementacja.

Żaden plik źródłowy nie został zmodyfikowany. CMake nietknięty. Brak commita.
