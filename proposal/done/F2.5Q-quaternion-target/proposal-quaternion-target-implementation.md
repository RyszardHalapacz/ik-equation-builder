# Proposal: target kwaternionowy (F2.5Q), implementacja

**Etap:** F2.5Q · **v2 — APPROVED, ZAIMPLEMENTOWANE 2026-08-21** · część 2 z 2

> **v2, po review.** Kod produkcyjny bez zmian — review przeniosło go na snapshot
> repo i skompilowało czysto (`C++23`, `-Wall -Wextra -Wswitch -Werror`). Trzy
> poprawki, wszystkie po stronie testów i opisu:
>
> 1. **Blocker: `QuaternionErrorMessageCarriesFullPrecision` faktycznie by nie
>    przeszedł.** Asercja na podciąg `"1.0000001"` sprawdzała **zapis
>    dziesiętny**, a nie kontrakt. Odtworzone u mnie: dla skalowanego kwaternionu
>    skośnego komunikat brzmi `||q|| = 1.0000000999999998`, więc podciągu tam nie
>    ma. Test przepisany na **round trip** (7.2).
> 2. **Opis przypisywał testowi NaN pilnowanie kolejności kroków** — niesłusznie.
>    NaN wychodzi jako `NonFiniteQuaternion` także wtedy, gdy norma jest liczona
>    pierwsza, byle kontrola skończoności nastąpiła przed zwrotem. Kolejność
>    pilnuje przypadek `inf`. Poprawione w 3.2, 7.2 i w tabeli ryzyk.
> 3. `<utility>` dopisane wprost w nowym pliku testowym, plus `<string>` w
>    rozszerzeniu `test_target_validation.cpp` (`std::stod`, `npos`).
>
> Sprawdzone przy okazji, żeby nie zostawić drugiego takiego testu: `find("1.5")`
> w `RejectsNonNormalizedQuaternion` jest bezpieczne — norma renderuje się
> dokładnie jako `1.5`; progi z testów krawędziowych wychodzą na `9e-09` i
> `1.1e-08`, a `−q` daje normę dokładnie `1`.

> **Zaimplementowane 2026-08-21.** Kod wszedł dokładnie w brzmieniu z tego
> dokumentu, z jedną zmianą redakcyjną przy `NegatedQuaternionProducesSameTarget`:
> komentarz mówi „exactly equal values" zamiast „bit w bit", bo `EXPECT_EQ` na
> `double` sprawdza równość numeryczną, a nie reprezentację (`+0.0 == -0.0`);
> równość bitowa nie jest tu potrzebnym kontraktem domenowym. Gate osiągnięty:
> **365/365 w obu konfiguracjach, zero ostrzeżeń**, `build-werror/` z żywymi
> asercjami, więc `assert` z sekcji 5 był wykonywany przez cały zestaw testów.

> **Zaimplementowane 2026-08-21.** Kod wszedł w brzmieniu z tego dokumentu, z
> jedną zmianą redakcyjną przy `NegatedQuaternionProducesSameTarget`: komentarz
> mówi o dokładnej równości **wartości**, nie „bit w bit", bo `EXPECT_EQ` na
> `double` nie odróżnia `+0.0` od `-0.0` — a równość bitowa nie jest tu potrzebnym
> kontraktem domenowym. Gate osiągnięty: **365/365 w obu konfiguracjach, zero
> ostrzeżeń**; `build-werror/` ma żywe asercje, więc `assert` z sekcji 5 był
> wykonywany przez cały zestaw testów.

Architektura: [`proposal-quaternion-target-architecture.md`](proposal-quaternion-target-architecture.md)
— **v2, APPROVED** w review 2026-08-21. Ten dokument nie podejmuje decyzji
projektowych; wykłada kod, który je realizuje, i uzasadnia wyłącznie te wybory,
które w architekturze były opisane słowem, a tutaj muszą stać się instrukcją.

## Prompt

> Napisać proposal implementacyjny F2.5Q po zatwierdzeniu architektury.

## 0. Co dokładnie realizuje ten dokument

Pięć zdań kontraktu z architektury, w kolejności, w jakiej kod ma je spełnić:

```text
1. Quaternion jest typem modelu, nie celem   -> osobny nagłówek, poza IkTarget
2. validate: skończoność PRZED normą         -> NaN nie może przejść (arch. 3.1)
3. przyjęty kwaternion jest normalizowany    -> zanim dotknie wzoru (arch. 3.3)
4. jedne drzwi, konwersja niepubliczna       -> anonimowa przestrzeń nazw (arch. 5)
5. zero zmian w IkTarget i ConstraintBuilder -> weryfikowalne diffem (arch. 7)
```

Bilans plików:

```text
NOWE        model/Quaternion.hpp            ~26 linii, sam typ
            model/QuaternionTarget.hpp      ~44 linie, jedna deklaracja + kontrakt
            model/QuaternionTarget.cpp      ~60 linii
            tests/test_quaternion_target.cpp  11 testów

ZMIENIONE   model/TargetValidation.hpp      +2 kody, +1 stała, +1 deklaracja
            model/TargetValidation.cpp      +1 helper, +1 funkcja
            tests/test_target_validation.cpp  +10 testów
            CMakeLists.txt                  +1 wiersz
            tests/CMakeLists.txt            +1 wiersz

NIETKNIĘTE  IkTarget.hpp, Equation.hpp, IkEquationSystem.{hpp,cpp},
            ConstraintBuilder.{hpp,cpp}, cała warstwa symboliczna, loader,
            budowniczowie transformacji, fasada, tests/support
```

---

## 1. `src/ik_equations/model/Quaternion.hpp` — nowy

```cpp
#pragma once

namespace kinemaforge::ik {

// (w, x, y, z) -- skalar PIERWSZY. ROS/tf2 i Eigen::Quaterniond::coeffs()
// serializują (x, y, z, w); przepisanie czwórki bez zmiany kolejności daje
// obrót, który jest poprawnym obrotem, tylko innym. Norma zostaje wtedy
// nienaruszona, więc walidacja tego NIE wykryje i nie udaje, że wykryje --
// dlatego przy wypełnianiu z zewnętrznego źródła używać inicjalizatorów
// desygnowanych:
//
//     Quaternion{.w = qw, .x = qx, .y = qy, .z = qz};
//
// Domyślnie identyczność. Poprawność (skończoność, norma) NIE jest warunkiem
// konstrukcji -- ta sama zasada, co w RotationMatrix3: sprawdza validate().
struct Quaternion
{
    double w{1.0};
    double x{};
    double y{};
    double z{};
};

} // namespace kinemaforge::ik
```

Osobny nagłówek, precedens `model/Vector3.hpp`. Brak `operator==`: nikt go dziś
nie potrzebuje, a dla kwaternionu równość bitowa jest myląca — `q` i `−q` to ten
sam obrót, więc „równe" musiałoby najpierw znaczyć „równe jako co".

---

## 2. `src/ik_equations/model/TargetValidation.hpp` — trzy dopiski

Nagłówek dostaje `#include "ik_equations/model/Quaternion.hpp"` obok
istniejącego `IkTarget.hpp`, oraz:

```cpp
enum class TargetValidationErrorCode
{
    NonFinitePosition,
    NonFiniteOrientation,
    NonOrthogonalOrientation,
    InvalidOrientationDeterminant,
    NonFiniteQuaternion,
    NonNormalizedQuaternion
};
```

```cpp
// Tolerancja normy kwaternionu. NIE jest to kOrientationTolerance ani jego
// pochodna: tamta mierzy odchylenie R^T R od I, ta -- długość czterowektora.
// Zmierzone na 200 000 losowych obrotów: kwaternion jednostkowy zapisany z
// dziewięcioma miejscami po przecinku odchyla się najwyżej o 9.03e-10, czyli
// mieści się w progu z 11-krotnym zapasem. Równość liczbowa z
// kOrientationTolerance jest zbiegiem okoliczności dwóch pomiarów; zaostrzenie
// tamtej stałej nie ma pociągać tej.
//
// Czego ten próg NIE mówi: skąd dane przyszły. Kwaternion jednostkowy po
// przejściu przez float mieści się w nim w 41.65% przypadków (1 000 000 prób,
// mediana błędu normy 1.22e-8), a identyczność i (.5,.5,.5,.5) przechodzą przez
// float z błędem dokładnie zero. To jest miara jakości zapisu, który
// dostaliśmy, nie detektor precyzji, w jakiej ten zapis powstał -- i nie ma jak
// nim być. Przyjęty kwaternion po float niesie błąd obrotu <= 9.9e-8 rad.
inline constexpr double kQuaternionNormTolerance = 1e-8;
```

```cpp
// Sprawdzane w tej kolejności, i kolejność jest obserwowalna:
//   1. cztery składowe skończone     -> NonFiniteQuaternion
//   2. | ||q|| - 1 | <= tolerancja   -> NonNormalizedQuaternion
//
// Krok 1 istnieje, bo NaN przechodzi przez KAŻDE porównanie w kroku 2:
// "NaN > tolerancja" jest fałszem, więc walidacja bez niego przyjmuje NaN jako
// kwaternion jednostkowy. Ten sam kształt błędu, który review z 2026-08-17
// znalazło w RobotModel::within_limits() (usuniętym w R4).
//
// Krok 1 jest PIERWSZY z innego powodu: dla inf norma też jest inf, a
// "inf > tolerancja" jest już PRAWDĄ -- odwrócona kolejność zwracałaby więc
// NonNormalizedQuaternion o danych, które nie są liczbami. Kod błędu ma nie
// kłamać, nawet gdy odrzucenie i tak następuje.
[[nodiscard]] std::expected<void, TargetValidationError> validate(const Quaternion& quaternion);
```

Deklaracja idzie **między** `validate(const RotationMatrix3&)` a
`validate(const PoseTarget&)` — kolejność deklaracji odpowiada kolejności, w
jakiej te rzeczy się składają: składowe, potem cel.

`TargetVisitor` w `.cpp` zostaje bez zmian. To jest sprawdzian punktu 1 z sekcji
0: gdyby kwaternion był alternatywą `IkTarget`, ten wizytor **musiałby** się tu
zmienić i przestałby się kompilować bez decyzji — a nie zmienia się, bo
kwaternion nie jest celem.

---

## 3. `src/ik_equations/model/TargetValidation.cpp` — `validate(Quaternion)`

### 3.1 Helper w anonimowej przestrzeni nazw

Obok istniejącego `describeAxis`:

```cpp
const char* describeQuaternionComponent(std::size_t index) noexcept
{
    switch (index)
    {
    case 0: return "w";
    case 1: return "x";
    case 2: return "y";
    default: return "z";
    }
}
```

Kształt skopiowany z `describeAxis` świadomie, razem z `default` zamiast
`case 3`: przy indeksie z pętli `0..3` trzeci `case` i tak byłby martwy, a
`-Wswitch` nie ma tu czego pilnować, bo to `std::size_t`, nie enum.

### 3.2 Funkcja

Wstawiana za `validate(const RotationMatrix3&)`:

```cpp
std::expected<void, TargetValidationError> validate(const Quaternion& quaternion)
{
    const double components[4] = {quaternion.w, quaternion.x, quaternion.y, quaternion.z};
    for (std::size_t index = 0; index < 4; ++index)
        if (!std::isfinite(components[index]))
            return makeError(TargetValidationErrorCode::NonFiniteQuaternion,
                             std::string("quaternion ") + describeQuaternionComponent(index)
                                 + " is not finite");

    // Kontrola skończoności jest WYŻEJ z dwoch niezaleznych powodow, i warto je
    // rozroznic, bo chronia przed czym innym:
    //
    //   brak kroku 1 w ogole  -> NaN daje norme NaN, "NaN > tol" jest falszem,
    //                            wiec NaN przechodzi jako kwaternion jednostkowy;
    //   krok 2 przed krokiem 1 -> inf daje norme inf, "inf > tol" jest prawda,
    //                            wiec inf wychodzi jako NonNormalizedQuaternion
    //                            zamiast NonFiniteQuaternion -- kod bledu klamie.
    //
    // Samo przesuniecie tego sqrt wyzej, przy zachowanej petli finite przed
    // zwrotem, nie zmienia niczego dla NaN. Kolejnosc pilnuje przypadek inf.
    const double norm = std::sqrt(quaternion.w * quaternion.w + quaternion.x * quaternion.x
                                  + quaternion.y * quaternion.y + quaternion.z * quaternion.z);

    if (std::abs(norm - 1.0) > kQuaternionNormTolerance)
        return makeError(TargetValidationErrorCode::NonNormalizedQuaternion,
                         "quaternion norm differs from 1: ||q|| = " + format(norm));

    return {};
}
```

### 3.3 Dwie rzeczy, które w tym kodzie nie są oczywiste

**Dlaczego `sqrt`, a nie porównanie `‖q‖²` z jedynką.** Wersja bez pierwiastka
jest kusząca (`|n² − 1| ≤ 2·tol`), ale zmienia znaczenie progu: dla `n = 1 + ε`
jest `n² − 1 = 2ε + ε²`, więc tolerancja przestaje być tym, co deklaruje
komentarz i nazwa stałej, a zaczyna być „w przybliżeniu dwa razy tym". Przy
progu, którego cała wartość polega na tym, że jest **zmierzony i porównywalny z
populacją wejścia**, taka zamiana jest droga za koszt jednego pierwiastka na
cel. `format(norm)` w komunikacie pokazuje przy okazji tę samą wielkość, o
której mówi stała — nie jej kwadrat.

**Przepełnienie: `isfinite` na składowych nie gwarantuje skończonej sumy
kwadratów.** Dla `w = 1e200` każda składowa jest skończona, a `w * w` to już
`inf`, więc `norm` wychodzi `inf`. To **nie jest** dziura: `|inf − 1| > tol` jest
prawdą, więc taki kwaternion zostaje odrzucony jako nienormalizowany — czyli
dokładnie tak, jak powinien. Odnotowane, bo to jedyne miejsce w tej funkcji,
gdzie porównanie z `inf` daje poprawną odpowiedź przypadkiem, a nie z konstrukcji
— i ktoś, kto kiedyś zamieni `sqrt` na `std::hypot`, powinien wiedzieć, że
zmienia też ten przypadek (`hypot` skaluje i **nie** przepełni, więc `1e200`
przeszłoby przez normę jako skończone `1e200` i dalej też odpadło, tylko inną
drogą).

---

## 4. `src/ik_equations/model/QuaternionTarget.hpp` — nowy

```cpp
#pragma once

#include "ik_equations/model/IkTarget.hpp"
#include "ik_equations/model/Quaternion.hpp"
#include "ik_equations/model/TargetValidation.hpp"
#include "ik_equations/model/Vector3.hpp"

#include <expected>

namespace kinemaforge::ik {

// Jedyne drzwi, którymi kwaternion wchodzi do tego projektu.
//
// Konwersja siedzi na granicy modelu, a nie w jego środku: po zbudowaniu celu
// istnieje dokładnie jedna reprezentacja orientacji i dokładnie jedna ścieżka do
// IkEquationSystem. Dlatego "ten sam układ równań dla kwaternionu i dla
// równoważnej macierzy" jest tu prawdą konstrukcyjną, a nie wynikiem testu.
//
// Zwrócony PoseTarget przechodzi validate(PoseTarget) ZAWSZE. Nie jest to
// grzeczność wobec ConstraintBuildera, tylko warunek spójności modelu: typ,
// którego własny walidator odrzuca jego własny wytwór, jest zepsuty. Gwarancję
// daje normalizacja przed wzorem -- kwaternion przyjęty na krawędzi tolerancji
// bez niej dałby macierz odchyloną o 8e-8, czyli osiem razy poza
// kOrientationTolerance.
//
// Pozycja i orientacja są walidowane w tej kolejności, tej samej co w
// validate(PoseTarget); pierwszy błąd wygrywa.
//
// q i -q to ten sam obrót i dają ten sam PoseTarget, komórka po komórce.
//
// Nazwa mówi, KTÓRE drzwi są użyte, zamiast polegać na przeciążeniu po typie:
// zapowiedziane formy wejścia -- RPY i wektor obrotu -- mają identyczną
// sygnaturę (Vector3, Vector3), więc rodzina przeciążeń rozpadłaby się dokładnie
// wtedy, gdy zaczęłaby być potrzebna.
[[nodiscard]] std::expected<PoseTarget, TargetValidationError>
makePoseTargetFromQuaternion(const Vector3& position, const Quaternion& orientation);

} // namespace kinemaforge::ik
```

---

## 5. `src/ik_equations/model/QuaternionTarget.cpp` — nowy

```cpp
#include "ik_equations/model/QuaternionTarget.hpp"

#include <cassert>
#include <cmath>
#include <utility>

namespace kinemaforge::ik {
namespace {

// Warunek wstępny: validate(quaternion) się powiodło, więc każda składowa jest
// skończona, a norma leży w [1 - 1e-8, 1 + 1e-8]. W szczególności nie jest
// bliska zeru i to dzielenie nie ma jak wybuchnąć.
//
// Niepubliczna właśnie dlatego: funkcja z warunkiem wstępnym, którego caller nie
// może sprawdzić, byłaby drugimi drzwiami z gorszym kontraktem -- {0,0,0,0}
// dałby przez nią dzielenie przez zero i macierz pełną NaN.
//
// Normalizacja NIE jest naprawą danych. Obrót jest zakodowany w kierunku w R^4,
// a długość jest artefaktem zapisu, więc dzielenie kanonizuje reprezentację i
// niczego nie wybiera za caller'a. Bez niego kwaternion przyjęty na krawędzi
// pasma dałby macierz z |R^T R - I| = 8e-8 (zmierzone), czyli osiem razy poza
// kOrientationTolerance; po nim -- 2.22e-15.
RotationMatrix3 toRotationMatrix(const Quaternion& quaternion)
{
    const double norm = std::sqrt(quaternion.w * quaternion.w + quaternion.x * quaternion.x
                                  + quaternion.y * quaternion.y + quaternion.z * quaternion.z);

    const double w = quaternion.w / norm;
    const double x = quaternion.x / norm;
    const double y = quaternion.y / norm;
    const double z = quaternion.z / norm;

    // Wzór jest kwadratowy w składowych, więc R(-q) = R(q) bit w bit: każdy
    // iloczyn ma parzystą liczbę czynników ze zmienionym znakiem.
    return RotationMatrix3{{{{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),       2.0 * (x * z + w * y)},
                             {2.0 * (x * y + w * z),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)},
                             {2.0 * (x * z - w * y),       2.0 * (y * z + w * x),       1.0 - 2.0 * (x * x + y * y)}}}};
}

} // namespace

std::expected<PoseTarget, TargetValidationError>
makePoseTargetFromQuaternion(const Vector3& position, const Quaternion& orientation)
{
    if (auto checked = validate(PositionTarget{position}); !checked)
        return std::unexpected(std::move(checked.error()));

    if (auto checked = validate(orientation); !checked)
        return std::unexpected(std::move(checked.error()));

    PoseTarget target{position, toRotationMatrix(orientation)};

    // Nie jest to walidacja wyniku, tylko zapisany w kodzie wniosek z 3.3
    // architektury: po normalizacji ta macierz MUSI przechodzić walidację, którą
    // zaraz wykona ConstraintBuilder. W Release znika, i to jest w porządku --
    // własność pilnuje ProducedOrientationAlwaysPassesTargetValidation, a
    // build-werror/ ma asercje żywe, więc cały zestaw testów ją wykonuje.
    //
    // To NIE jest kształt usunięty w R4: tam assert poprzedzał wyłuskanie, więc
    // w Release złamany warunek zamieniał się w UB. Tu nic po nim nie następuje.
    assert(validate(target.orientation).has_value());

    return target;
}

} // namespace kinemaforge::ik
```

Jedna rzecz warta uwagi w kolejności: pozycja jest walidowana **pierwsza**, tak
samo jak w `validate(PoseTarget)`. Gdyby było odwrotnie, cel z `NaN` w pozycji i
złym kwaternionem raportowałby raz jeden błąd, raz drugi, zależnie od tego, którą
drogą powstał — a `RejectsNonFinitePositionInQuaternionTarget` pilnuje, że tak
nie jest.

---

## 6. Budowa

`CMakeLists.txt`, w liście źródeł `kinemaforge_ik`, za `TargetValidation.cpp`:

```cmake
    src/ik_equations/model/QuaternionTarget.cpp
```

`tests/CMakeLists.txt`, w liście źródeł `kinemaforge_tests`, za
`test_target_validation.cpp`:

```cmake
    test_quaternion_target.cpp
```

`Quaternion.hpp` i `QuaternionTarget.hpp` nie wymagają wpisu — nagłówki idą przez
`target_include_directories(kinemaforge_ik PUBLIC src)`.

---

## 7. `tests/test_target_validation.cpp` — dziesięć testów

### 7.1 Dopiski w anonimowej przestrzeni nazw

```cpp
Quaternion axisAngleQuaternion(double x, double y, double z, double angle)
{
    const double length = std::sqrt(x * x + y * y + z * z);
    const double half = 0.5 * angle;
    const double sine = std::sin(half) / length;
    return Quaternion{std::cos(half), x * sine, y * sine, z * sine};
}

// Oś skośna: żadna składowa nie jest zerem ani równa innej, więc zgubiona lub
// przestawiona składowa nie ma się gdzie schować.
Quaternion skewQuaternion()
{
    return axisAngleQuaternion(1.0, -2.0, 3.0, 0.7);
}

Quaternion scaled(const Quaternion& quaternion, double factor)
{
    return Quaternion{quaternion.w * factor, quaternion.x * factor,
                      quaternion.y * factor, quaternion.z * factor};
}

Quaternion withComponent(std::size_t index, double value)
{
    Quaternion quaternion{};   // identyczność
    double* const components[4] = {&quaternion.w, &quaternion.x,
                                   &quaternion.y, &quaternion.z};
    *components[index] = value;
    return quaternion;
}

Quaternion throughFloat(const Quaternion& quaternion)
{
    return Quaternion{static_cast<float>(quaternion.w), static_cast<float>(quaternion.x),
                      static_cast<float>(quaternion.y), static_cast<float>(quaternion.z)};
}
```

Plus `using kinemaforge::ik::Quaternion;` i
`using kinemaforge::ik::kQuaternionNormTolerance;` w nagłówku pliku, oraz dwa
includy: `#include "ik_equations/model/Quaternion.hpp"` (formalnie zbędny —
wchodzi przez `TargetValidation.hpp` — ale plik testowy ma deklarować, czego
używa) i `#include <string>` dla `std::stod` oraz `std::string::npos`, które w
tym pliku dotąd nie występowały.

### 7.2 Testy

```cpp
// --- quaternion ------------------------------------------------------

TEST(TargetValidationTest, AcceptsUnitQuaternion)
{
    EXPECT_TRUE(validate(Quaternion{}).has_value());              // identyczność
    EXPECT_TRUE(validate(skewQuaternion()).has_value());
}

TEST(TargetValidationTest, AcceptsNegatedUnitQuaternion)
{
    const Quaternion quaternion = skewQuaternion();
    EXPECT_TRUE(validate(scaled(quaternion, -1.0)).has_value());
}

TEST(TargetValidationTest, RejectsNonFiniteQuaternion)
{
    // To jest test, który pilnuje KOLEJNOŚCI kroków, a nie ten z NaN niżej.
    // Dla inf norma też jest inf, a "inf > tolerancja" jest PRAWDĄ -- więc
    // walidacja sprawdzająca normę jako pierwsza zwróciłaby tu
    // NonNormalizedQuaternion i ten EXPECT_EQ pęka. Kod błędu mówiłby wtedy
    // "kwaternion nie jest znormalizowany" o danych, które nie są liczbami.
    for (const double value : {kInfinity, -kInfinity})
        for (std::size_t index = 0; index < 4; ++index)
        {
            SCOPED_TRACE(testing::Message() << "component " << index << " = " << value);
            const auto result = validate(withComponent(index, value));
            ASSERT_FALSE(result.has_value());
            EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFiniteQuaternion);
            EXPECT_FALSE(result.error().message.empty());
        }
}

TEST(TargetValidationTest, RejectsNaNQuaternionInsteadOfAcceptingIt)
{
    // Ten test pilnuje, że NaN jest wykrywany JAWNIE. Walidacja bez osobnego
    // kroku skończoności policzyłaby normę NaN, a "NaN > tolerancja" jest
    // FAłSZEM -- czyli przyjęłaby NaN jako kwaternion jednostkowy i test
    // przeszedłby do sukcesu. Ten sam kształt błędu, który review z 2026-08-17
    // znalazło w within_limits().
    //
    // Czego ten test NIE pilnuje: kolejności kroków. Implementacja licząca normę
    // najpierw, ale wykonująca kontrolę skończoności przed zwrotem, nadal
    // zwróci tu NonFiniteQuaternion -- bo porównanie z NaN jest fałszywe i po
    // prostu przechodzi dalej. Kolejność pilnuje przypadek inf w teście wyżej.
    for (std::size_t index = 0; index < 4; ++index)
    {
        SCOPED_TRACE(testing::Message() << "component " << index << " = NaN");
        const auto result = validate(withComponent(index, kNaN));
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonFiniteQuaternion);
    }
}

TEST(TargetValidationTest, RejectsZeroQuaternion)
{
    const auto result = validate(Quaternion{0.0, 0.0, 0.0, 0.0});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonNormalizedQuaternion);
}

TEST(TargetValidationTest, RejectsNonNormalizedQuaternion)
{
    const auto result = validate(scaled(skewQuaternion(), 1.5));
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonNormalizedQuaternion);
    EXPECT_NE(result.error().message.find("1.5"), std::string::npos);
}

TEST(TargetValidationTest, AcceptsQuaternionAtNormToleranceEdge)
{
    EXPECT_TRUE(validate(scaled(skewQuaternion(), 1.0 + 0.9 * kQuaternionNormTolerance))
                    .has_value());
}

TEST(TargetValidationTest, RejectsQuaternionJustOutsideNormTolerance)
{
    const auto result = validate(scaled(skewQuaternion(), 1.0 + 1.1 * kQuaternionNormTolerance));
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, TargetValidationErrorCode::NonNormalizedQuaternion);
}

TEST(TargetValidationTest, QuaternionErrorMessageCarriesFullPrecision)
{
    // std::to_string drukuje sześć miejsc, więc norma 1 + 1e-7 -- czyli sama
    // przyczyna odrzucenia -- wyrenderowałaby się jako "1.000000". Precedens: ten
    // sam problem przy wyznaczniku.
    //
    // Test sprawdza to przez ROUND TRIP, a nie przez podciąg. Asercja na zapis
    // dziesiętny byłaby testem cudzego formatowania: dla przeskalowanego
    // kwaternionu skośnego komunikat brzmi "||q|| = 1.0000000999999998" i żaden
    // oczekiwany podciąg w rodzaju "1.0000001" się w nim nie znajdzie -- mimo że
    // kontrakt jest spełniony idealnie. Kontraktem max_digits10 jest bowiem to, że
    // wartość da się ODCZYTAĆ z powrotem bez straty, a nie że wygląda ładnie.
    //
    // Kwaternion osiowy, bo wtedy wartość oczekiwana to sqrt(w*w) -- jeden
    // składnik, więc test nie przepisuje całego wzoru na normę z produkcji.
    const Quaternion quaternion{1.0000001, 0.0, 0.0, 0.0};

    const auto result = validate(quaternion);
    ASSERT_FALSE(result.has_value());
    ASSERT_EQ(result.error().code, TargetValidationErrorCode::NonNormalizedQuaternion);

    const std::string marker = "||q|| = ";
    const auto position = result.error().message.find(marker);
    ASSERT_NE(position, std::string::npos) << result.error().message;

    const double reported = std::stod(result.error().message.substr(position + marker.size()));

    EXPECT_DOUBLE_EQ(reported, std::sqrt(quaternion.w * quaternion.w));

    // I to jest właściwy powód istnienia tego testu: komunikat ma odróżniać
    // odrzuconą normę od jedynki. Wydruk z to_string ("1.000000") odczytałby się
    // z powrotem jako dokładnie 1.0 i ta asercja by go złapała.
    EXPECT_NE(reported, 1.0);
}

TEST(TargetValidationTest, AcceptsQuaternionThatCameThroughFloat)
{
    // Pinuje kontrakt, a nie zachowanie liczbowe: próg normy NIE jest detektorem
    // pochodzenia danych i nie ma jak nim być. Wartości dokładnie
    // reprezentowalne w float mają po konwersji błąd normy równy ZERU, więc
    // przechodzą zawsze -- gdyby ktoś kiedyś próbował "naprawić" walidację tak,
    // żeby odrzucała dane po float, ten test pęka pierwszy.
    //
    // Ogólny kwaternion po float celowo NIE jest tu asercją: przechodzi w 41.65%
    // przypadków (1 000 000 prób), więc oczekiwanie w którąkolwiek stronę byłoby
    // testem migoczącym z definicji.
    EXPECT_TRUE(validate(throughFloat(Quaternion{})).has_value());
    EXPECT_TRUE(validate(throughFloat(Quaternion{0.5, 0.5, 0.5, 0.5})).has_value());
}
```

---

## 8. `tests/test_quaternion_target.cpp` — nowy, jedenaście testów

### 8.1 Nagłówek i pomocnicze

```cpp
#include <gtest/gtest.h>

#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/ConstraintBuilder.hpp"
#include "ik_equations/builders/ForwardKinematicsBuilder.hpp"
#include "ik_equations/builders/JointTransformBuilder.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/model/Quaternion.hpp"
#include "ik_equations/model/QuaternionTarget.hpp"
#include "ik_equations/model/TargetValidation.hpp"
#include "ik_equations/symbolic/Expression.hpp"
#include "ik_equations/symbolic/SymbolicTransform.hpp"
#include "support/NumericForwardKinematics.hpp"
#include "support/TransformComparison.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace ik = kinemaforge::ik;
namespace support = kinemaforge::testsupport;

using ik::ConstraintBuilder;
using ik::IkTaskKind;
using ik::PoseTarget;
using ik::Quaternion;
using ik::RotationMatrix3;
using ik::TargetValidationErrorCode;
using ik::Vector3;
using ik::makePoseTargetFromQuaternion;
using ik::sameNode;
using ik::sameSource;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Próg tych porównań: 1e-12, ten sam, którym związana jest krzyżowa walidacja
// FK. Zmierzone odchylenia są cztery rzędy niżej -- 1.11e-15 dla porównania z
// Rodriguesem i 1.78e-15 dla obrotu wektora -- więc test nie ma jak migotać.
constexpr double kTolerance = 1e-12;

Quaternion axisAngleQuaternion(double x, double y, double z, double angle)
{
    const double length = std::sqrt(x * x + y * y + z * z);
    const double half = 0.5 * angle;
    const double sine = std::sin(half) / length;
    return Quaternion{std::cos(half), x * sine, y * sine, z * sine};
}

// Rodrigues: R = I + sin(t) K + (1 - cos(t)) K^2, K = [k]_x, k jednostkowe.
//
// To jest DRUGA droga, i jej wartość polega na tym, że nie przechodzi przez
// kwaterniony w ogóle. Pomylona konwencja (w,x,y,z) / (x,y,z,w) albo sprzężenie
// zamiast kwaternionu są w świecie kwaternionowym spójne same ze sobą -- dopiero
// porównanie z drogą trygonometryczną je pokazuje.
RotationMatrix3 rodrigues(double x, double y, double z, double angle)
{
    const double length = std::sqrt(x * x + y * y + z * z);
    const double kx = x / length;
    const double ky = y / length;
    const double kz = z / length;

    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double t = 1.0 - c;

    return RotationMatrix3{{{{c + kx * kx * t,      kx * ky * t - kz * s, kx * kz * t + ky * s},
                             {ky * kx * t + kz * s, c + ky * ky * t,      ky * kz * t - kx * s},
                             {kz * kx * t - ky * s, kz * ky * t + kx * s, c + kz * kz * t}}}};
}

void expectRotationsMatch(const RotationMatrix3& actual, const RotationMatrix3& expected,
                          double tolerance)
{
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            EXPECT_NEAR(actual.values[row][column], expected.values[row][column], tolerance)
                << "cell (" << row << ", " << column << ")";
}

struct LoadedRobot
{
    ik::KinematicChain chain;
    ik::SymbolicTransform fk;
};

// Kopia helpera z test_constraint_builder.cpp, świadomie: dwie jednostki
// translacji, żaden wspólny nagłówek testowy poza support/, i żadna z nich nie
// ma prawa zmienić drugiej przy okazji.
std::optional<LoadedRobot> loadRobot(const char* fileName)
{
    ik::UrdfModelLoader loader;
    const auto robot = loader.load(std::filesystem::path{KINEMAFORGE_URDF_DATA_DIR} / fileName);

    ik::KinematicChainBuilder chainBuilder;
    auto chain = chainBuilder.build(robot, "base_link", "tool0");
    if (!chain)
    {
        ADD_FAILURE() << "chain build failed for " << fileName;
        return std::nullopt;
    }

    const ik::JointTransformBuilder transformBuilder;
    const ik::ForwardKinematicsBuilder fkBuilder;
    auto fk = fkBuilder.build(*chain, transformBuilder);

    return LoadedRobot{std::move(*chain), std::move(fk)};
}

} // namespace
```

### 8.2 Kształt i wartości

```cpp
TEST(QuaternionTargetTest, AcceptsValidQuaternionTarget)
{
    const auto target = makePoseTargetFromQuaternion(
        Vector3{1.6, -0.4, 2.1}, axisAngleQuaternion(1.0, -2.0, 3.0, 0.7));

    ASSERT_TRUE(target.has_value());
    EXPECT_DOUBLE_EQ(target->position.x, 1.6);
    EXPECT_DOUBLE_EQ(target->position.y, -0.4);
    EXPECT_DOUBLE_EQ(target->position.z, 2.1);
    EXPECT_TRUE(ik::validate(*target).has_value());
}

TEST(QuaternionTargetTest, IdentityQuaternionProducesIdentityRotation)
{
    const auto target = makePoseTargetFromQuaternion(Vector3{}, Quaternion{});
    ASSERT_TRUE(target.has_value());

    // Bez tolerancji: dla q = (1,0,0,0) każdy iloczyn we wzorze jest zerem, a
    // 1 - 2*0 to dokładnie 1. Test na EXPECT_NEAR przepuściłby tu implementację,
    // która liczy coś prawie-identycznościowego.
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            EXPECT_DOUBLE_EQ(target->orientation.values[row][column], row == column ? 1.0 : 0.0);
}

TEST(QuaternionTargetTest, ProducesEquivalentTargetsForQuaternionAndMatrix)
{
    // "Równoważna macierz" NIE może być wynikiem tej samej konwersji, bo wtedy
    // test jest tautologią. Macierz powstaje więc niezależnie, ze wzoru
    // Rodriguesa, z tej samej pary (oś, kąt).
    const double axis[3] = {1.0, -2.0, 3.0};
    const double angle = 0.7;

    const auto fromQuaternion = makePoseTargetFromQuaternion(
        Vector3{0.2, 0.3, 0.4}, axisAngleQuaternion(axis[0], axis[1], axis[2], angle));
    ASSERT_TRUE(fromQuaternion.has_value());

    const PoseTarget fromMatrix{Vector3{0.2, 0.3, 0.4},
                                rodrigues(axis[0], axis[1], axis[2], angle)};

    expectRotationsMatch(fromQuaternion->orientation, fromMatrix.orientation, kTolerance);
    EXPECT_DOUBLE_EQ(fromQuaternion->position.x, fromMatrix.position.x);
}

TEST(QuaternionTargetTest, AgreesWithRodriguesRotationFormula)
{
    struct Case { double x, y, z, angle; const char* label; };
    constexpr double kPi = 3.14159265358979323846;

    for (const Case& testCase : {Case{0.0, 0.0, 1.0, 0.5 * kPi,      "quarter turn about z"},
                                 Case{1.0, 0.0, 0.0, -0.25 * kPi,    "negative, about x"},
                                 Case{1.0, -2.0, 3.0, 0.7,           "skew axis"},
                                 Case{1.0, 1.0, 1.0, kPi - 1e-6,     "just short of pi"},
                                 Case{0.0, 1.0, 0.0, 1e-9,           "near identity"}})
    {
        SCOPED_TRACE(testCase.label);
        const auto target = makePoseTargetFromQuaternion(
            Vector3{}, axisAngleQuaternion(testCase.x, testCase.y, testCase.z, testCase.angle));
        ASSERT_TRUE(target.has_value());

        expectRotationsMatch(target->orientation,
                             rodrigues(testCase.x, testCase.y, testCase.z, testCase.angle),
                             kTolerance);
    }
}

TEST(QuaternionTargetTest, MatchesQuaternionRotationWithoutFormingAMatrix)
{
    // Trzecia droga: oracle obraca wektor kanapką kwaternionową i NIGDY nie
    // buduje macierzy. Porównujemy zachowanie, nie zapis -- R*v kontra rotate().
    const Quaternion quaternion = axisAngleQuaternion(1.0, -2.0, 3.0, 0.7);

    const auto target = makePoseTargetFromQuaternion(Vector3{}, quaternion);
    ASSERT_TRUE(target.has_value());

    const support::Quaternion oracleQuaternion{quaternion.w, quaternion.x,
                                               quaternion.y, quaternion.z};

    for (const support::Vector3d& vector : {support::Vector3d{1.0, 0.0, 0.0},
                                            support::Vector3d{0.0, 1.0, 0.0},
                                            support::Vector3d{0.0, 0.0, 1.0},
                                            support::Vector3d{0.37, -1.4, 0.9}})
    {
        const support::Vector3d expected = support::rotate(oracleQuaternion, vector);
        const auto& m = target->orientation.values;

        EXPECT_NEAR(m[0][0] * vector.x + m[0][1] * vector.y + m[0][2] * vector.z,
                    expected.x, kTolerance);
        EXPECT_NEAR(m[1][0] * vector.x + m[1][1] * vector.y + m[1][2] * vector.z,
                    expected.y, kTolerance);
        EXPECT_NEAR(m[2][0] * vector.x + m[2][1] * vector.y + m[2][2] * vector.z,
                    expected.z, kTolerance);
    }
}

TEST(QuaternionTargetTest, NegatedQuaternionProducesSameTarget)
{
    const Quaternion quaternion = axisAngleQuaternion(1.0, -2.0, 3.0, 0.7);
    const Quaternion negated{-quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z};

    const auto first = makePoseTargetFromQuaternion(Vector3{}, quaternion);
    const auto second = makePoseTargetFromQuaternion(Vector3{}, negated);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());

    // EXPECT_EQ, nie EXPECT_NEAR: wzór jest kwadratowy, więc zmiana znaku znosi
    // się w każdym iloczynie, a norma i dzielenie są dla -q takie same. Wartości
    // są DOKŁADNIE równe, nie tylko bliskie -- słabsza asercja przepuściłaby
    // implementację, która gubi czynnik w.
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            EXPECT_EQ(first->orientation.values[row][column],
                      second->orientation.values[row][column])
                << "cell (" << row << ", " << column << ")";
}
```

### 8.3 Odrzucenia i własność

```cpp
TEST(QuaternionTargetTest, RejectsNonNormalizedQuaternionTarget)
{
    const Quaternion quaternion = axisAngleQuaternion(1.0, -2.0, 3.0, 0.7);
    const auto target = makePoseTargetFromQuaternion(
        Vector3{}, Quaternion{quaternion.w * 1.5, quaternion.x * 1.5,
                              quaternion.y * 1.5, quaternion.z * 1.5});

    ASSERT_FALSE(target.has_value());
    EXPECT_EQ(target.error().code, TargetValidationErrorCode::NonNormalizedQuaternion);
    EXPECT_FALSE(target.error().message.empty());
}

TEST(QuaternionTargetTest, RejectsNonFinitePositionInQuaternionTarget)
{
    // Pozycja jest walidowana pierwsza i to jest obserwowalne: kwaternion też
    // jest tu zły, a mimo to raportowany błąd dotyczy pozycji.
    const auto target = makePoseTargetFromQuaternion(Vector3{kNaN, 0.0, 0.0},
                                                     Quaternion{0.0, 0.0, 0.0, 0.0});

    ASSERT_FALSE(target.has_value());
    EXPECT_EQ(target.error().code, TargetValidationErrorCode::NonFinitePosition);
}

TEST(QuaternionTargetTest, ProducedOrientationAlwaysPassesTargetValidation)
{
    // Własność, na której stoi cały wariant C: cel wychodzący stąd musi
    // przechodzić walidację, którą zaraz wykona ConstraintBuilder. Bez
    // normalizacji kwaternion na krawędzi pasma daje |R^T R - I| = 8e-8, czyli
    // osiem razy poza kOrientationTolerance -- i ten test pęka.
    //
    // Ziarno stałe, więc przebieg jest powtarzalny; wejście zaokrąglone do
    // dziewięciu miejsc po przecinku, czyli do kontraktu wejściowego, którego
    // najgorszy zmierzony błąd normy to 9.03e-10.
    std::mt19937_64 generator{20260821};
    std::normal_distribution<double> gaussian{0.0, 1.0};

    for (int sample = 0; sample < 200; ++sample)
    {
        SCOPED_TRACE(testing::Message() << "sample " << sample);

        double raw[4] = {gaussian(generator), gaussian(generator),
                         gaussian(generator), gaussian(generator)};
        const double norm = std::sqrt(raw[0] * raw[0] + raw[1] * raw[1]
                                      + raw[2] * raw[2] + raw[3] * raw[3]);
        ASSERT_GT(norm, 1e-6);

        for (double& component : raw)
            component = std::round(component / norm * 1e9) / 1e9;

        const auto target = makePoseTargetFromQuaternion(
            Vector3{}, Quaternion{raw[0], raw[1], raw[2], raw[3]});

        ASSERT_TRUE(target.has_value()) << target.error().message;
        EXPECT_TRUE(ik::validate(target->orientation).has_value());
    }
}
```

### 8.4 End-to-end: dwie niezależne ścieżki

```cpp
namespace {

// Obie ścieżki startują z tej samej konfiguracji i nigdzie się nie spotykają aż
// do ConstraintBuildera:
//
//                          konfiguracja q
//                        /                \
//   symboliczne FK -> evaluate -> R      oracle numeryczny -> q_oracle
//            |                                   |
//   PoseTarget{position, R}          makePoseTargetFromQuaternion(position, q_oracle)
//            \                                   /
//              ConstraintBuilder -> dwa IkEquationSystem -> porównanie
//
// Lewa strona idzie macierzami przez całą warstwę symboliczną, prawa
// kwaternionami przez niezależną implementację FK. Zgodność dwunastu prawych
// stron znaczy więc, że nowa konwersja zgadza się z drugą implementacją, a nie
// sama ze sobą.
//
// Oracle nie normalizuje swojego kwaternionu po drodze i nie musi: złożenie
// czternastu jednostkowych kwaternionów (siedem jointów, origin + ruch)
// odchyla normę najwyżej o 1.55e-15 przy 200 000 prób, czyli sześć milionów razy
// wewnątrz pasma akceptacji. Gdyby kiedyś wyszło poza, ten test zgłosi
// NonNormalizedQuaternion -- i będzie to prawdziwy sygnał, nie migotanie.
void expectSystemsMatch(const char* fileName, const support::JointConfiguration& configuration)
{
    const auto robot = loadRobot(fileName);
    ASSERT_TRUE(robot.has_value());

    const auto values = support::makeSymbolValues(robot->chain, configuration);

    const auto evaluated = support::evaluateSymbolic(robot->fk, values);
    ASSERT_TRUE(evaluated.has_value());

    RotationMatrix3 rotation{};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            rotation.values[row][column] = (*evaluated)[row][column];

    const Vector3 position{(*evaluated)[0][3], (*evaluated)[1][3], (*evaluated)[2][3]};

    const support::RigidTransform reference =
        support::numericForwardKinematics(robot->chain, configuration);
    const Quaternion quaternion{reference.rotation.w, reference.rotation.x,
                                reference.rotation.y, reference.rotation.z};

    const auto fromQuaternion = makePoseTargetFromQuaternion(position, quaternion);
    ASSERT_TRUE(fromQuaternion.has_value()) << fromQuaternion.error().message;

    const ConstraintBuilder builder;
    const auto matrixSystem = builder.build(robot->fk, robot->chain,
                                            PoseTarget{position, rotation});
    const auto quaternionSystem = builder.build(robot->fk, robot->chain, *fromQuaternion);

    ASSERT_TRUE(matrixSystem.has_value());
    ASSERT_TRUE(quaternionSystem.has_value());

    EXPECT_EQ(quaternionSystem->taskKind(), IkTaskKind::Pose);
    ASSERT_EQ(quaternionSystem->equations().size(), 12u);
    ASSERT_EQ(matrixSystem->equations().size(), 12u);
    EXPECT_EQ(quaternionSystem->unknowns().size(), matrixSystem->unknowns().size());

    for (std::size_t index = 0; index < 12; ++index)
    {
        SCOPED_TRACE(testing::Message() << "equation " << index);
        const auto& fromMatrix = matrixSystem->equations()[index];
        const auto& fromQuat = quaternionSystem->equations()[index];

        EXPECT_TRUE(sameNode(fromQuat.lhs, fromMatrix.lhs));
        EXPECT_TRUE(sameSource(fromQuat.source, fromMatrix.source));
        EXPECT_NEAR(ik::constantValue(fromQuat.rhs), ik::constantValue(fromMatrix.rhs),
                    kTolerance);
    }
}

} // namespace

TEST(QuaternionTargetTest, BuildsKr640FullPoseSystemFromQuaternion)
{
    expectSystemsMatch("kr640.urdf", support::JointConfiguration{0.3, -0.7, 1.1, 0.2, -0.5, 0.9});
}

TEST(QuaternionTargetTest, BuildsKr4FullPoseSystemFromQuaternion)
{
    expectSystemsMatch("kr4_r600.urdf", support::JointConfiguration{-0.2, 0.4, -0.9, 1.2, 0.6, -1.1});
}
```

Dwa roboty, tak jak w F2.4 i F2.5. Wartość drugiego nie jest kosmetyczna:
`kr4_r600` ma inną geometrię `rpy` w originach, więc jego macierz FK ma
niezerowe komórki tam, gdzie `kr640` ma zera — a zero jest jedyną wartością,
którą pomylona komórka może udawać niezauważona.

---

## 9. `STATUS.md`

```text
Pipeline progress   -> quaternion target ✅ F2.5Q -- implemented + tested
Tests               344 -> 365
Done                nowa sekcja: makePoseTargetFromQuaternion, jedne drzwi,
                    normalizacja przed konwersją, konwersja niepubliczna
Not done            F2.5Q znika z listy "specified, not built"
Known gaps          nowy wiersz: próg normy nie mówi nic o pochodzeniu danych
                    (41.65% kwaternionów po float przechodzi); prowenancja
                    wymagałaby osobnego typu wejściowego
Working process     wiersz dla proposala implementacyjnego F2.5Q
Next step           -> F2.6
```

Wiersz w „Known gaps" jest tu najważniejszy i celowo nie jest schowany w
proposalu: to jedyna rzecz w tym etapie, która **wygląda** na gwarancję, a nią
nie jest.

---

## 10. Gate

```text
build (Release)        0 ostrzeżeń
build-werror           0 ostrzeżeń, asercje żywe -- w tym assert z 5
                       (review v1 skompilowało kod produkcyjny na snapshocie
                        repo w tej konfiguracji: czysto)
ctest                  365/365 w obu (344 + 21)
istniejące testy       zero zmienionych asercji
zakres                 trzy nowe pliki produkcyjne, dwa rozszerzone,
                       jeden nowy plik testowy, dwa CMakeLists, STATUS.md
diff kontrolny         git diff --stat nie może pokazać IkTarget.hpp,
                       IkEquationSystem.*, Equation.hpp ani ConstraintBuilder.*
```

Ostatni wiersz jest sprawdzianem architektury, nie porządku: gdyby którykolwiek
z tych plików musiał się zmienić, znaczyłoby to, że kod realizuje wariant B, a
nie zatwierdzony C.

---

## 11. Ryzyka implementacyjne

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Brak jawnej kontroli skończoności | **realne** | `RejectsNaNQuaternionInsteadOfAcceptingIt`: bez niej NaN przechodzi jako jednostkowy i test zmienia się w sukces (7.2) |
| Kroki walidacji zamienione miejscami | **realne** | `RejectsNonFiniteQuaternion` z `±inf`: norma `inf` **przekracza** próg, więc walidacja licząca normę pierwsza zwróci `NonNormalizedQuaternion` i `EXPECT_EQ` na kodzie pęka. Przypadek NaN tego **nie** pilnuje — porównanie z NaN jest fałszywe i po prostu przechodzi dalej (7.2) |
| Normalizacja pominięta „bo i tak jest w paśmie" | **realne** | `ProducedOrientationAlwaysPassesTargetValidation` + `assert` w Debug; bez normalizacji krawędź pasma daje 8e-8 |
| Transpozycja w tabeli 3×3 | **realne** | `AgreesWithRodriguesRotationFormula` na osi skośnej: `R(0,1)` i `R(1,0)` różnią się znakiem, więc transpozycja nie przejdzie; dodatkowo `MatchesQuaternionRotationWithoutFormingAMatrix` na czterech wektorach |
| Wzór przepisany z pomyloną konwencją `(x,y,z,w)` | realne | Wyłapuje to wyłącznie droga trygonometryczna (Rodrigues) — dlatego jest osobnym testem, a nie wariantem poprzedniego |
| Test end-to-end migocze przez normę z oracle'a | niskie | Zmierzone: 1.55e-15 po czternastu mnożeniach, sześć milionów razy w paśmie (8.4) |
| `EXPECT_EQ` na doublach w teście `-q` | świadome | Asercja o dokładnej równości **wartości**, nie o reprezentacji bitowej (`EXPECT_EQ` nie odróżnia `+0.0` od `-0.0`, i nie musi — równość bitowa nie jest tu kontraktem domenowym). `EXPECT_NEAR` osłabiałby test bez powodu |
| Asercja na treść komunikatu | **złapane w review v1** | Podciąg `"1.0000001"` nie występuje w `"1.0000000999999998"`. Zamienione na round trip przez `std::stod` plus `reported != 1.0` (7.2). Pozostałe asercje na komunikat sprawdzone: `find("1.5")` renderuje się dokładnie |
| Duplikat `loadRobot` w dwóch plikach testowych | świadome | Wspólny nagłówek testowy poza `support/` nie istnieje i nie zakładamy go dla jednej funkcji; oba pliki mają się dać czytać osobno |

---

## 12. Rekomendacja

Zaimplementować po zatwierdzeniu. Po F2.5Q Faza 2 ma zamknięte wszystkie etapy
dotyczące **modelu celu** — `PositionTarget`, `PoseTarget` i obie drogi wejścia
do orientacji — i zostają dwa etapy o ograniczeniach i integracji: **F2.6**
(constrainty kierunkowe, cztery rodziny) oraz **F2.7** (fasada, DoD Fazy 2,
release `v0.2-ik-equations`).

Otwarte pytania: brak.
