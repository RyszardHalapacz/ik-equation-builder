# Proposal: ścisłe parsowanie `<limit>` — architektura (R3, v3)

## Prompt

> Review kodu, uwaga 4: „Walidacja `<limit>` jest zdecydowanie słabsza od
> walidacji geometrii. To kontrastuje z bardzo dobrym parserem Vector3. […] Nie
> ma rozróżnienia `lower="0"` od brakującego / niepoprawnie skonwertowanego
> atrybutu. Do tego `velocity` i `effort` w ogóle nie przechodzą `isfinite()`.
> A potem wartości są kopiowane do domenowego `JointLimits`, czyli wyglądają już
> jak dane zweryfikowane. […] Przed solverem IK zdecydowanie bym to zamknął."

Etap `F2.3.R`, dokument **R3**. Waga MEDIUM/HIGH.

## Historia wersji

**v1 → v2** po review (verdykt: *approve with one required change*):

| # | Poprawka z review | Gdzie |
|---|---|---|
| 1 | **wymagana:** jawnie zdefiniować profil wejściowy, przestać opisywać `lower`/`upper` jako uniwersalne „odstępstwo od spec" — semantyka `<limit>` zależy od generacji URDF | **4, 4.1** |
| 2 | `invalid_limit_range` → `invalid_limit_constraint` (przy ujemnym `velocity` nie ma „range") | 5.1 |
| 3 | uzasadnienie dla `velocity == 0` oparte na walidacji, nie na „bywa zastępnikiem nieokreślonego" | 4.3 |
| 4 | jawnie rozstrzygnąć `<limit>` przy `Fixed` — w v1 przemilczane | **4.4** (nowa sekcja) |
| 5 | rozdzielić trzy warstwy parsera, żeby `lower="1 2"` nie przeszło jako `1` | **3.1** (nowa sekcja) |

Potwierdzone bez zmian: wariant B parsera, `lower == upper` legalne,
`velocity`/`effort >= 0`, brak `negative_limit_value`, `hasPositionLimits`
nietknięte.

**v2 → v3** po review (verdykt: *approve with one required change*):

| # | Poprawka z review | Gdzie |
|---|---|---|
| 6 | **wymagana:** profil musi określać **dokładnie jeden** `<limit>`; wielokrotna deklaracja odrzucana, nie „pierwszy wygrywa" | **4.2** (nowa sekcja), 5.1 |
| 7 | podwójna numeracja `4.4` | 4.4–4.7 przenumerowane |
| 8 | §6.2 deklarowało 4 pozycje przy 5 wypisanych, w tym jedna niebędąca testem pozytywnym | 6.2, **6.3** (wydzielona), **6.4** (bilans) |

Liczby testów spięte w §6.4: **20 nowych** (15 negatywnych, 4 pozytywne,
1 diagnostyczny). Kodów błędu: **+5, −1**.

Przy okazji punktu 6 sprawdzone i **nazwane** rodzeństwo problemu: `<origin>`,
`<axis>`, `<parent>`, `<child>` mają ten sam wzorzec „pierwszy wygrywa" i są
świadomie poza R3 (4.2).

## Status weryfikacji

Odtworzone uruchomieniem na `main` przed R2, potwierdzone ponownie po R2:

```text
<limit velocity="1" effort="1"/>          bez lower/upper
    → accepted, limits = [0, 0]     joint aktuowany ZAMROŻONY po cichu
<limit lower="banana" upper="3" .../>
    → accepted, lower = 0           tekst niebędący liczbą → cicha zamiana na 0
<limit lower="-3" upper="3" velocity="nan" effort="inf"/>
    → accepted                      velocity/effort nigdy nie przechodzą isfinite
```

**Zweryfikowane przed napisaniem tego dokumentu** (ta sama dyscyplina co w R2 —
nowa walidacja nie może wywalić własnych danych):

```text
kr4_r600.urdf   6 bloków <limit> / 6 jointów aktuowanych
                6× lower, 6× upper, 6× velocity, 6× effort
kr640.urdf      to samo, 6/6/6/6
```

Wszystkie cztery atrybuty są obecne wszędzie, więc dowolny wariant ścisłości
przechodzi na obu robotach.

## 1. Cel i zakres

**W zakresie:** doprowadzenie walidacji `<limit>` do rygoru, jaki mają już
`origin/xyz` i `axis/xyz`.

**Poza zakresem:** topologia (R2, zrobione), `RobotModel`/`within_limits` (R4),
`continuous` jako funkcjonalność, egzekwowanie limitów podczas rozwiązywania IK
(to należy do solvera — R3 dba tylko o to, żeby dane wejściowe były prawdziwe).

## 2. Stan obecny — zweryfikowany w plikach

`src/kinematics/robot_model_loader.cpp:328–338`:

```cpp
if (j.type != JointType::fixed) {
    auto lim = node.child("limit");
    if (!lim)
        return std::unexpected(makeError(LoadErrorCode::invalid_limits, j.name, "limit"));
    j.limits.lower    = lim.attribute("lower").as_double();
    j.limits.upper    = lim.attribute("upper").as_double();
    j.limits.velocity = lim.attribute("velocity").as_double();
    j.limits.effort   = lim.attribute("effort").as_double();
    if (!std::isfinite(j.limits.lower) || !std::isfinite(j.limits.upper) ||
        j.limits.lower > j.limits.upper)
        return std::unexpected(makeError(LoadErrorCode::invalid_limits, j.name, "limit"));
}
```

Cztery odrębne wady:

**(a) `as_double()` nie odróżnia braku od śmieci.** pugixml zwraca `0.0` zarówno
dla nieobecnego atrybutu, jak i dla `lower="banana"`. Ta sama funkcja jest źródłem
obu wyników, więc kod nie ma jak ich rozdzielić.

**(b) Brakujące `lower`/`upper` dają `[0, 0]`, czyli zamrażają joint.** Warunek
`lower > upper` przechodzi (`0 > 0` jest fałszem), więc model wygląda na
zwalidowany, a joint aktuowany ma zerową przestrzeń ruchu. To najgroźniejsza z
czterech: cicha, fizycznie bezsensowna odpowiedź.

**(c) `velocity` i `effort` nie są walidowane wcale.** `as_double()` używa
`strtod`, który **parsuje** `"nan"` i `"inf"`, a warunek `isfinite` obejmuje tylko
`lower` i `upper`. Nieskończona prędkość przechodzi do modelu domenowego.

**(d) Kontrast z geometrią jest jaskrawy.** `origin/xyz` i `axis/xyz` mają własny
parser `parse_vector3` (`robot_model_loader.cpp:61–89`) oparty na
`std::from_chars`, który odrzuca złą liczbę składowych, końcowe śmieci,
`nan`/`inf` i przepełnienie (`1e400`), a nawet obsługuje wiodący `+` z ochroną
przed `+-1`. Ten sam plik traktuje `<limit>` przez `as_double()`.

Efekt końcowy jest gorszy niż suma wad: `UrdfModelLoader.cpp` kopiuje te wartości
do domenowego `JointLimits`, gdzie **wyglądają jak dane zweryfikowane**, bo
sąsiadują z geometrią, która naprawdę jest.

## 3. Decyzja: wspólny rdzeń parsera, nie druga implementacja

`parse_vector3` już zawiera dokładnie tę logikę, której `<limit>` potrzebuje —
tylko powtórzoną trzy razy i opakowaną w reguły separatorów.

| Wariant | Ocena |
|---|---|
| **A.** Nowy, niezależny `parse_scalar` | Odrzucony. Dwie implementacje tej samej rzeczy rozjadą się; to właśnie ten rozjazd jest treścią uwagi 4. |
| **B.** Wyodrębnić rdzeń „jedna liczba z pozycji `p`", zbudować na nim oba | **Wybrany.** |
| **C.** Zawołać `parse_vector3` i wziąć pierwszą składową | Odrzucony. `"1 2 3"` w `lower` przeszłoby jako `1`. |

### 3.1 Trzy warstwy z rozdzielnymi kontraktami

Kontrakt dla proposala implementacyjnego, wypisany po uwadze z review — granice
muszą być jawne, inaczej `lower="1 2"` przejdzie jako `1`:

```text
parseFiniteNumber(p, end) -> optional<double>, nowe p
    jeden token liczbowy od pozycji p
    wiodący '+' dozwolony, ale nie '+-' ani '++'
    std::from_chars
    isfinite
    NIE konsumuje otaczających białych znaków
    NIE wymaga dojścia do końca wejścia

parseScalar(text) -> optional<double>            ← NOWE, dla <limit>
    whitespace*  parseFiniteNumber  whitespace*  END
    dokładnie jedna liczba, nic po niej

parseVector3(text) -> optional<Vec3>             ← ISTNIEJĄCE, zachowuje semantykę
    whitespace*  parseFiniteNumber
      ( whitespace+ parseFiniteNumber ) × 2
    whitespace*  END
    separator whitespace WYMAGANY między składowymi ("1-2 3" nie jest wektorem)
```

Rozdzielenie „parsuj liczbę" od „skonsumuj całe wejście" jest tu istotą: to
`parseScalar` odpowiada za `END`, więc `lower="1 2"` odpada na nadmiarowej treści,
a nie milcząco daje `1`. Odwrotnie, `parseVector3` **musi** móc zatrzymać się w
środku, żeby móc czytać dalej.

Ryzyko refaktoru pokrywa 56 istniejących testów loadera, w tym `ParsesValidVector3`,
`AcceptsLeadingAndTrailingWhitespace`, `AcceptsLeadingPlusSign`,
`AcceptsScientificNotation`, `AcceptsCharacterReferenceTabSeparators`.

## 4. Profil wejściowy KinemaForge dla `<limit>`

> **Zmiana wobec v1 — wymagana przez review.** v1 opisywało wymaganie
> `lower`/`upper` jako „świadome odstępstwo od specyfikacji URDF". To
> sformułowanie było błędne, bo traktowało „specyfikację URDF" jak jedną rzecz.
> Semantyka `<limit>` **różni się między generacjami URDF**: w starszych
> `lower`/`upper` są opcjonalne z domyślnym `0`, a `effort`/`velocity` wymagane;
> w URDF 1.2 dla `revolute`/`prismatic` jest odwrotnie — `lower`/`upper` stają się
> wymagane, a `effort`/`velocity` opcjonalne i domyślnie znaczą „brak
> ograniczenia". Tabela z v1 mieszała więc dwie generacje: wymagała
> `lower`/`upper` jak 1.2 i jednocześnie `effort`/`velocity` jak 1.0/1.1.

### 4.1 R3 nie implementuje wersjonowanej semantyki URDF — i tego nie udaje

Fakt zweryfikowany w repo, nie założony:

```text
data/urdf/kr4_r600.urdf   <robot name="kuka_kr4_r600">        — brak atrybutu version
data/urdf/kr640.urdf      <robot name="kuka_kr640_r2800_2">   — brak atrybutu version
robot_model_loader.cpp    atrybut `version` nie jest czytany ani razu
```

Loader nie ma żadnej negocjacji wersji. Nie może więc twierdzić zgodności z
jakimkolwiek konkretnym wydaniem URDF — ani jej łamać. Jedyne uczciwe
sformułowanie to **własny, nazwany profil wejściowy**:

```text
KinemaForge strict joint-limit profile (R3)

Revolute / Prismatic:
    <limit>   DOKŁADNIE JEDEN
    lower     WYMAGANY
    upper     WYMAGANY
    velocity  WYMAGANY
    effort    WYMAGANY

liczby:
    dokładnie jedna skończona wartość double na atrybut

relacje:
    lower <= upper
    velocity >= 0
    effort   >= 0

Fixed:
    <limit> IGNOROWANY (patrz 4.4)

Continuous:
    poza zakresem R3
```

Jest to **celowo węższy kontrakt** niż starsze URDF (gdzie `lower`/`upper` mają
domyślne `0`) i **nie jest** implementacją semantyki URDF 1.2 (gdzie
`effort`/`velocity` byłyby opcjonalne). Ani jedno, ani drugie — trzecia rzecz,
nazwana wprost.

Pełna obsługa `<robot version="1.0|1.1|1.2">` z semantyką zależną od wersji
zasługuje na osobny proposal, jeśli interoperacyjność z nowym `urdfdom` okaże się
potrzebna. R3 świadomie tego nie dotyka: dodałoby negocjację wersji do etapu,
którego celem jest przestać cicho zamieniać śmieci na `0`.

### 4.2 Liczność `<limit>` — dokładnie jeden, nie „co najmniej jeden"

> Dodane w v3 jako **wymagana zmiana z review**. v2 mówiło tylko „`<limit>`
> wymagany", co implementacyjnie znaczyłoby `node.child("limit")` — czyli *weź
> pierwszy, resztę zignoruj*.

```xml
<joint name="j1" type="revolute">
    <limit lower="-1" upper="1" velocity="2" effort="10"/>
    <limit lower="-3" upper="3" velocity="5" effort="50"/>
</joint>
```

Bez reguły liczności loader wziąłby pierwszy i przemilczał drugi. To **dokładnie
ta klasa błędu, którą R3 usuwa**: wejście jest niejednoznaczne, parser po cichu
wybiera jedno znaczenie, a model wygląda na zwalidowany. Ta sama dyscyplina, którą
R2 zastosował do `<link>` (dwie deklaracje jednej nazwy nie mogą zapaść się w
jedną).

```text
0 × <limit>   → missing_limit_element
1 × <limit>   → parsowanie
2+ × <limit>  → duplicate_limit_element
```

Osobny kod, nie ogólny `invalid_limit_declaration`: naprawa dla autora pliku jest
oczywista i nazwa ją niesie.

**Zakres — rodzeństwo tego problemu, świadomie poza R3.** Sprawdzone w kodzie:
ten sam wzorzec „pierwszy wygrywa" dotyczy również innych elementów:

```text
robot_model_loader.cpp:283   node.child("origin")
robot_model_loader.cpp:307   node.child("axis")
robot_model_loader.cpp:325   node.child("parent")
robot_model_loader.cpp:326   node.child("child")
robot_model_loader.cpp:259   doc.child("robot")
```

`<limit>` nie jest więc wyjątkiem — jest tym elementem, którego R3 akurat dotyka.
Rozszerzenie reguły liczności na `origin`/`axis`/`parent`/`child` jest osobną,
niewielką zmianą i **nie wchodzi do R3**: poszerzałaby etap o elementy, których
walidacji ten dokument nie analizował, a każdy z nich ma własne pytanie o to, czy
brak jest błędem, czy domyślną wartością (dla `origin` i `axis` brak jest legalny
i ma default, dla `parent`/`child` nie). Zapisane tutaj, żeby nie wyglądało, że
liczność `<limit>` została uznana za jedyny taki przypadek.

> **Rekomendacja:** dopisać do listy prac jako drobny, samodzielny proposal po
> zamknięciu F2.3.R. Nie blokuje niczego — dziś żaden realny plik nie ma
> zdublowanych elementów, a oba fixture'y sprawdzono.

### 4.3 Dlaczego profil jest ścisły w obie strony

Domyślne `0` dla `lower` i `upper` daje przedział `[0, 0]` — joint aktuowany,
który nie może się ruszyć. **Niemożliwy do odróżnienia od pomyłki.**

1. **Ten projekt wyprowadza równania IK.** Joint o zerowej przestrzeni ruchu jest
   albo błędem modelu, albo powinien być `fixed`. Wybór między tymi znaczeniami
   należy do autora pliku, nie do parsera.
2. **Odrzucenie jest głośne, przyjęcie ciche.** Ta sama zasada, którą repo już
   stosuje przy walidacji targetów: *validation rejects, never repairs*.
   Podstawienie `0` jest naprawianiem.
3. **Koszt zerowy dla realnych plików.** Zmierzone: 6/6 jointów w obu robotach ma
   wszystkie cztery atrybuty.

**Koszt, który uznaję:** węższy profil oznacza, że plik akceptowany przez
KDL/`tf2`/Pinocchio może zostać przez nas odrzucony. To realna cena
interoperacyjności — płacona świadomie i nazwana, a nie ukryta pod słowem
„zgodność".

### 4.4 `velocity` i `effort` — skończone i nieujemne

Poza `isfinite` dokładamy `>= 0`. Ujemna prędkość maksymalna czy ujemny moment nie
mają odczytu, przy którym byłyby sensowne; to nie „kierunek", to ograniczenie na
wartość bezwzględną.

**Zero jest legalne**, i uzasadnienie jest wąskie na miejscu:

> `0` jest skończone i nieujemne. R3 waliduje dane, a **nie interpretuje
> biznesowo**, czy zerowa maksymalna prędkość ma sens dla konkretnego robota.

(v1 argumentowało tu, że „`velocity="0"` bywa zastępnikiem »nieokreślone«" —
review słusznie wskazało, że ten argument jest niepotrzebny i słabszy od
powyższego. Wycofane.)

### 4.5 `<limit>` przy `Fixed` — ignorowany

> Dodane w v2 na wniosek review: decyzja była w v1 przemilczana, a implementator
> musiałby ją odgadnąć.

```xml
<joint type="fixed">
    <limit lower="banana"/>
</joint>
```

**Wariant A: całe `<limit>` jest ignorowane dla `Fixed`** — nie parsowane, nie
walidowane.

Uzasadnienie i precedens: dokładnie tak samo repo traktuje już `<axis>` dla
jointu ustalonego. Kontrakt loadera mówi „a fixed joint's axis carries no
meaning", a `RobotDescription` dokumentuje to samo. Ograniczenie ruchu jointu,
który się nie rusza, nie ma znaczenia semantycznego i domenowy model go nie
konsumuje (`hasPositionLimits` jest dla `Fixed` fałszywe).

Odrzucony wariant B (walidować składnię, choć nie używamy wartości) rozszerzałby
ten etap na element, którego nikt nie czyta — a koszt jest niesymetryczny:
odrzucalibyśmy pliki za treść bez znaczenia.

Uwaga o zakresie: `<axis>` dla `Fixed` jest dziś walidowany **składniowo** (parser
przechodzi dla każdego typu, pomijana jest tylko semantyka niezdegenerowania).
`<limit>` dla `Fixed` **nie będzie** walidowany nawet składniowo. Ta asymetria jest
świadoma: dla `<axis>` składnia jest tania, bo parser i tak jest wołany dla
wszystkich; dla `<limit>` wymagałaby uruchomienia całej ścieżki tylko po to, żeby
wynik wyrzucić.

### 4.6 Przedział zerowej szerokości pozostaje legalny

`lower="1.5" upper="1.5"` przechodzi. Skoro oba atrybuty są wymagane, taki zapis
jest **jawny** — autor napisał tę wartość dwa razy. Odrzucamy `[0,0]` powstałe z
milczenia, nie `[1.5,1.5]` powstałe z decyzji.

### 4.7 `hasPositionLimits` zostaje wyprowadzane — świadomie

`kinemaforge::ik::JointLimits` ma pole `hasPositionLimits`, którego
`mt::kinematics::JointLimits` nie ma. Dziś `UrdfModelLoader.cpp:69` wyprowadza je
jako `type != Fixed`, z komentarzem, że jest to 1:1 z „jest aktuowany", bo loader
wymaga `<limit>` dla każdego jointu nieustalonego.

Po R3 to nadal jest prawdą i **R3 tego nie zmienia**. Zmieni się dopiero z
`continuous`: URDF nie wymaga `lower`/`upper` dla jointu ciągłego, więc wtedy
„aktuowany" przestanie być równoważne „ma limity pozycji" i flaga będzie musiała
pochodzić z parsowania, nie z typu. Odnotowane jako styk z przyszłym proposalem
`continuous`, nie jako zadanie R3 — poszerzanie zakresu o pole, którego dzisiaj
nikt nie czyta inaczej niż przez ten wzór, byłoby pracą na zapas.

## 5. Model błędów

### 5.1 `invalid_limits` rozbite na pięć kodów

Dziś jeden kod pokrywa brak elementu, złą wartość i zły przedział. Precedens z R2
(sześć kodów topologii) i z `IkEquationSystem::create` (dziewięć) mówi jasno:
**osobny kod tam, gdzie inna jest naprawa.**

```text
missing_limit_element        brak <limit> przy revolute/prismatic
duplicate_limit_element      więcej niż jeden <limit> w jednym joincie   (4.2)
missing_limit_attribute      brak wymaganego atrybutu (lower/upper/velocity/effort)
malformed_limit_value        atrybut obecny, ale nie jest jedną skończoną liczbą
invalid_limit_constraint     lower > upper, albo velocity/effort < 0
```

Bilans: **+5 kodów, −1** (`invalid_limits`).

> **Zmiana wobec v1:** czwarty kod nazywał się `invalid_limit_range`, co review
> słusznie uznało za niefortunne — przy `velocity="-1"` nie ma żadnego „range".
> `invalid_limit_constraint` obejmuje naturalnie oba przypadki: liczby są
> skończone, ale ich wzajemna relacja lub znak jest niedopuszczalny.

`invalid_limits` **zostaje usunięty** — po rozbiciu nie ma producenta. Każdy nowy
kod wymaga wpisu w `describeCode`; po R1 pominięcie wywali build, co potwierdzono
mutation checkiem.

Trzy z pięciu niosą `attribute` (`"limit/lower"` itd.) i `rawValue` (surowy
tekst, ucięty), więc komunikat wskazuje konkretny atrybut konkretnego jointu.

Osobnego `negative_limit_value` **nie tworzymy** — pięć kodów wystarcza, a
`attribute` usuwa niejednoznaczność. Potwierdzone w review.

### 5.2 Kolejność sprawdzania

```text
element → lower → upper → velocity → effort → relacje
```

W obrębie jednego atrybutu: obecność przed sparsowaniem, sparsowanie przed
znakiem. Relacje **na końcu**, bo `lower > upper` ma sens dopiero, gdy oba są
liczbami.

Ten sam zakres kontraktu co w R2 i z tego samego powodu: kolejność obowiązuje
**w obrębie walidacji limitów**. `<limit>` jest parsowany po `origin` i `axis` w
tej samej pętli po jointach, więc plik zepsuty geometrycznie **i** w limitach
zgłosi błąd geometrii. To stan faktyczny, nie postulat — R3 nie przestawia
kolejności.

### 5.3 Determinizm wyboru raportowanego atrybutu

Ta sama zasada, którą R2 zapisał dla linków (§5.4 tam), obowiązuje tutaj
trywialnie: atrybuty sprawdzamy w **ustalonej, zapisanej w kodzie kolejności**
(`lower`, `upper`, `velocity`, `effort`), nigdy przez iterację po
`lim.attributes()`, której kolejność zależy od pliku i biblioteki. Plik z dwoma
zepsutymi atrybutami zawsze zgłasza ten sam.

## 6. Plan testów

### 6.1 Negatywne (15)

```text
RejectsMissingLimitElement            revolute bez <limit>            (ścieżka istnieje, test nie — §6.4)
RejectsDuplicateLimitElement          dwa <limit> w jednym joincie    ← §4.2
RejectsMissingLowerLimit              <limit velocity effort/>        ← BUG (b)
RejectsMissingUpperLimit
RejectsMissingVelocity
RejectsMissingEffort
RejectsMalformedLowerLimit            lower="banana"                  ← BUG (a)
RejectsMalformedUpperLimit            upper="1 2"
RejectsEmptyLimitAttribute            lower=""
RejectsNonFiniteVelocity              velocity="nan"                  ← BUG (c)
RejectsNonFiniteEffort                effort="inf"
RejectsOverflowingLimitValue          upper="1e400"
RejectsNegativeVelocity
RejectsNegativeEffort
RejectsInvertedLimitRange             lower="1" upper="-1"            (ścieżka istnieje, test nie — §6.4)
```

### 6.2 Pozytywne — granica ścisłości (4)

```text
AcceptsZeroWidthLimitRange            lower="1.5" upper="1.5"   → §4.6
AcceptsZeroVelocityAndEffort          velocity="0" effort="0"   → §4.4
AcceptsLimitValuesInScientificNotation
IgnoresLimitForFixedJoint             fixed + lower="banana"    → §4.5
```

Wszystkie cztery istnieją po to, żeby ścisłość nie przerodziła się w nadmierną —
pinują dokładnie to, czego R3 celowo **nie** odrzuca. `IgnoresLimitForFixedJoint`
jest najważniejszy, bo jest jedynym dowodem na decyzję z 4.5; bez niego wariant A
istnieje tylko w komentarzu.

### 6.3 Diagnostyka (1)

```text
ReportsMalformedLimitWithAttributeContext   attribute="limit/lower", rawValue="banana"
```

Wydzielone z §6.2, bo nie jest to test pozytywny — sprawdza treść błędu, nie
przyjęcie pliku.

### 6.4 Bilans

```text
§6.1 negatywne     15
§6.2 pozytywne      4
§6.3 diagnostyka    1
                   --
razem nowych       20
```

### 6.5 Stan pokrycia dzisiaj — zero testów negatywnych

Sprawdzone gremplem po `tests/`, nie założone:

```text
testy asertujące na LoadErrorCode::invalid_limits        0
testy negatywne dla <limit> w jakiejkolwiek formie      0
testy pozytywne dla <limit>                             2
    MapsKr4VelocityAndEffortLimits
    MapsFixedJointWithoutPositionLimits
```

Jest to ostrzejsze, niż zakładałem pisząc §6.1: `invalid_limits` ma dziś **dwóch
producentów w kodzie i ani jednego testu**. Nie ma więc czego „przenosić na nowy
kod" — obie istniejące ścieżki błędu (brak elementu, odwrócony przedział) są
nietestowane, a odpowiadające im pozycje w §6.1 są nowymi testami, nie migracją.

To domyka obserwację z review — „testy loadera są bardzo mocne w xyz/rpy/axis, ale
ta część została praktycznie nietknięta" — liczbą: **z 56 testów loadera 0 dotyczy
poprawności `<limit>`.** Po R3 będzie **20 testów dotyczących `<limit>`** (§6.4);
całkowita liczba testów loadera wzrośnie odpowiednio.

Dwa testy pozytywne muszą przechodzić bez zmian i są dowodem, że ścisłość nie
zepsuła danych produkcyjnych.

## 7. Plan zmian w plikach

```text
src/kinematics/robot_model_loader.hpp   LoadErrorCode: +5, -1; profil w komentarzu
src/kinematics/robot_model_loader.cpp   rdzeń parsera skalara, parse_vector3 na nim,
                                        blok <limit> przepisany (~45 linii)
src/ik_equations/UrdfModelLoader.cpp    describeCode: +5 wpisów, -1
tests/test_urdf_model_loader.cpp        +20 (§6.4), zero przeniesień (§6.5)
```

Bez zmian w: FK, warstwie symbolicznej, TCP, modelu równań, `robot_model.hpp`,
`KinematicChainBuilder`.

**R3 jest samodzielnie releasowalne** — inaczej niż R2. Nie usuwa niczego, na czym
opiera się inne API; zawęża tylko zbiór akceptowanych plików. Nie ma zależności
merge z R4.

## 8. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Ścisły profil odrzuci plik, który KDL/Pinocchio/nowy `urdfdom` przyjmą | **realne** | Świadome i **nazwane** jako własny profil (4.1), nie ukryte pod słowem „zgodność". Wersjonowana semantyka URDF to osobny proposal, gdy interoperacyjność stanie się potrzebna |
| Brak negocjacji `version` zostanie odczytany jako zgodność z 1.0 | średnie | Zapisane wprost, że loader **nie czyta** `version` i nie twierdzi zgodności z żadnym wydaniem (4.1) |
| Refaktor `parse_vector3` zepsuje sprawdzoną geometrię | średnie | Wspólny rdzeń zmienia tylko wnętrze; 56 testów loadera, w tym 5 celujących wprost w składnię liczb. Granice warstw wypisane w 3.1, żeby `END` nie wyciekło do złej warstwy |
| Ścisłość odrzuci własne fixture'y | **zamknięte** | Zmierzone przed napisaniem: 6/6 jointów, 4/4 atrybuty w obu plikach |
| `velocity="0"` bywa zastępnikiem „nieokreślone" i zostanie odrzucone | niskie | Celowo **nie** odrzucamy zera (4.2), tylko wartości ujemne; pinowane testem |
| Pięć nowych kodów zamiast jednego to więcej powierzchni | niskie | Zgodne z dyscypliną repo; `-Wswitch` z R1 pilnuje kompletności `describeCode` |

## 9. Do rozstrzygnięcia w review (v2)

Zamknięte w v2: profil wejściowy nazwany wprost zamiast „odstępstwa od spec",
`invalid_limit_constraint`, uzasadnienie dla zera, `<limit>` przy `Fixed`
(wariant A), warstwy parsera. Potwierdzone bez zmian: wariant B, `lower == upper`
legalne, `>= 0`, `hasPositionLimits` nietknięte.

Pozostaje otwarte — obie pozycje drobne:

1. Czy nazwa profilu ma trafić do komentarza w `robot_model_loader.hpp` jako
   kontrakt publiczny (rekomenduję tak — kontrakt, którego nie widać w nagłówku,
   nie istnieje dla konsumenta), czy wystarczy ten dokument.
2. Czy asymetria z 4.4 — `<axis>` walidowany składniowo dla `Fixed`, `<limit>` nie
   walidowany wcale — jest akceptowalna, czy wolisz spójność w jedną ze stron.
   Rekomenduję zostawić: koszt składniowej walidacji jest w obu przypadkach
   różny, a uzasadnienie jest zapisane.

## 10. Rekomendacja końcowa

Wykonać po R2, przed F2.4. Uwaga 4 jest jedyną z dziesięciu, która dotyczy danych
wchodzących wprost do przyszłego solvera: limity złączowe są tym, co solver
wykorzysta do odrzucania rozwiązań poza zakresem, a dziś mogą do niego dotrzeć
jako `[0, 0]` z milczenia albo jako `inf`.

Gate:

```text
dwa <limit> w jednym joincie            → odrzucone (nie „pierwszy wygrywa")
brak wymaganego atrybutu <limit>        → odrzucone, z nazwą atrybutu
atrybut niebędący skończoną liczbą      → odrzucone, z surowym tekstem
lower="1 2"                             → odrzucone (nie przyjęte jako 1)
velocity/effort = nan/inf/ujemne        → odrzucone
lower > upper                           → odrzucone
lower == upper (jawne)                  → PRZECHODZI
velocity == effort == 0                 → PRZECHODZI
<limit> przy Fixed, dowolna treść       → IGNOROWANE
oba pliki w data/urdf                   → ładują się z niezmienionymi wartościami
profil wejściowy zapisany w nagłówku jako kontrakt
pełny zestaw testów przechodzi w build i build-werror
```
