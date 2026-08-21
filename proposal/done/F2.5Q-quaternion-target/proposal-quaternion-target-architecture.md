# Proposal: target kwaternionowy (F2.5Q), architektura

**Etap:** F2.5Q · **v2 — APPROVED, ZAIMPLEMENTOWANE** · 2026-08-21 · część 1 z 2

> **v2, po review.** Rdzeń zatwierdzony: wariant C (konwersja na granicy), N2
> (walidacja w wąskim paśmie + normalizacja), `kQuaternionNormTolerance = 1e-8`,
> `makePoseTargetFromQuaternion` jako jedyne drzwi, zero zmian w `IkTarget` i
> `ConstraintBuilder`, brak konwersji odwrotnej. Jeden blocker i cztery korekty:
>
> 1. **Norma nie jest detektorem `float`** — v1 wyciągała z najgorszego przypadku
>    wniosek o całej populacji. Odtworzone niezależnie: **41.65%** kwaternionów po
>    `float` przechodzi próg `1e-8`. Przepisane 3.2 i 3.4, komentarz przy stałej w
>    5, wycofane pytanie otwarte (9).
> 2. Z listy rzeczy łapanych przez normę wypadają **„dane w innej konwencji"** —
>    przestawienie składowych normę zachowuje, więc v1 przeczyła w 3.3 własnej
>    tabeli ryzyk (8).
> 3. **„`q` i `2q` to ten sam obrót"** było za szerokie: wzór z 4.1 zakłada wejście
>    unitarne i dla `2q` daje inną macierz. Prawdziwe zdanie dotyczy kierunku w
>    `R⁴`, i to ono uzasadnia normalizację (3.3).
> 4. **Test end-to-end opisany odwrotnie do tego, co oracle potrafi** — nie ma w
>    nim konwersji macierz → kwaternion. Poprawione na dwie równoległe ścieżki z
>    jednej konfiguracji (6), co jest zresztą mocniejszym testem.
> 5. Wspólny `TargetValidationError` zostaje, ale **nie z powodu**
>    `ConstraintBuilderError` — builder tych dwóch kodów nigdy nie zobaczy, bo
>    `makePoseTargetFromQuaternion` zwraca błąd wcześniej (7).
>
> Plus poprawka rachunkowa w 3.3: `2.44e-15` wobec `1e-8` to ~4 **miliony** razy,
> nie cztery tysiące.
>
> **Zatwierdzone w review v2 (2026-08-21)**, bez v3. Naniesione po zatwierdzeniu
> trzy rzeczy redakcyjne: licznik `+9 → +10` w 7 (pozostałość po v1), synchronizacja
> nazw testów niezależności z roadmapą (6 — **roadmapa zmieniona**, jeden wpis
> ogólny zastąpiony dwoma konkretnymi) oraz przyjęta preferencja nazewnicza z
> review: `makePoseTargetFromQuaternion` → **`makePoseTargetFromQuaternion`** (9a A — argument
> review wzmocniony o kolizję przeciążeń, która nadchodzi razem z RPY). Oba
> otwarte pytania zamknięte.

Poprzednik: [`../F2.5-constraint-builder-fullpose/proposal-constraint-builder-fullpose-architecture.md`](../F2.5-constraint-builder-fullpose/proposal-constraint-builder-fullpose-architecture.md)
— sekcja 5 tamtego dokumentu wydzieliła ten etap i zapisała trzy powody. Ten
proposal rozstrzyga **którymi drzwiami kwaternion wchodzi do pipeline'u** i
**czym dokładnie jest tolerancja normalizacji** — dwa pytania, których F2.5
świadomie nie tknęło.

## Prompt

> Roadmapa, F2.5Q — przyjmować orientację celu również jako kwaternion, bez
> utraty niezależności kontroli numerycznej FK.
>
> ```text
> + typ kwaternionu w modelu (albo alternatywa w PoseTarget, albo osobne wejście)
> + własna tolerancja normalizacji, niezależna od kOrientationTolerance
> + konwersja kwaternion -> RotationMatrix3 NAPISANA W PRODUKCIE OD ZERA
> ```
>
> Ostatni punkt jest warunkiem, nie preferencją: `tests/support/NumericForwardKinematics`
> jest drugą implementacją FK i jego wartość znika, jeśli produkt zacznie wołać
> ten sam kod.
>
> Minimalne testy: `AcceptsValidQuaternionTarget`,
> `ProducesEquivalentTargetsForQuaternionAndMatrix`, `RejectsNonNormalizedQuaternion`,
> `QuaternionConversionIsIndependentOfTestOracle`.
>
> Gate: ten sam `IkEquationSystem` dla kwaternionu i dla równoważnej macierzy;
> konwersja produkcyjna nie dzieli kodu z `tests/support`.

Prompt jest cytatem roadmapy **sprzed** review v2 i zostaje w tym brzmieniu.
Jedyna zmiana kontraktu: wpis `QuaternionConversionIsIndependentOfTestOracle`
został w roadmapie zastąpiony dwoma konkretnymi testami — uzasadnienie w 4.2,
nazwy w 6.

---

## 0. Stan wyjściowy

Co **jest** w repo dzisiaj (344/344 w obu konfiguracjach, `build/` i `build-werror/`):

```text
PoseTarget            { Vector3 position; RotationMatrix3 orientation; }   F2.3
RotationMatrix3       values[row][column], row-major, zwykły agregat       F2.3
validate(RotationMatrix3)  skończoność -> ortogonalność -> wyznacznik      F2.3
kOrientationTolerance = 1e-8, zmierzone na 500 000 losowych obrotach       F2.3
IkTarget              variant<PositionTarget, PoseTarget> — zbiór zamknięty F2.3
ConstraintBuilder     dwa przeciążenia, 3 i 12 równań                      F2.4/F2.5
```

Co **nie** istnieje w produkcie: kwaternion. W repo jest dokładnie jeden i leży
w `tests/support/NumericForwardKinematics.hpp`, z komentarzem mówiącym wprost,
że jego jedyną wartością jest bycie drugą implementacją.

Trzy fakty z tamtego pliku, które kształtują cały ten etap:

```cpp
struct Quaternion { double w{1.0}; double x{}; double y{}; double z{}; };  // (w,x,y,z)

Matrix3 toRotationMatrix(const Quaternion& q);            // wzór algebraiczny, 9 komórek
Vector3d rotate(const Quaternion& q, const Vector3d& v);  // v + 2w(u×v) + 2u×(u×v)
```

`rotate()` **nigdy nie buduje macierzy** — i to jest ta część oracle'a, która ma
dla nas wartość w tym etapie (4.2).

---

## 1. Cel i zakres

**Cel:** caller, który ma orientację celu jako kwaternion, ma móc zbudować
`PoseTarget` bez pisania własnej konwersji i bez ryzyka, że jego dane wejdą do
układu równań jako coś, co nie jest obrotem.

**W zakresie:**

```text
typ Quaternion w modelu produkcyjnym
walidacja kwaternionu (skończoność + norma), z własną tolerancją
konwersja kwaternion -> RotationMatrix3, napisana od zera
jedna funkcja wytwórcza zwracająca std::expected<PoseTarget, TargetValidationError>
```

**Poza zakresem, świadomie:**

```text
konwersja odwrotna RotationMatrix3 -> Quaternion   brak konsumenta (9a B)
wejście osi-kąt, RPY, kąty Eulera                  inny etap, inne pułapki
kwaternion jako niewiadoma / zmienna symboliczna   to nie jest cel zadania
zmiana IkTarget, IkTaskKind, IkEquationSystem      zero (2.3)
zmiana ConstraintBuilder                           zero (2.3)
integracja z fasadą                                F2.7
```

Etap dodaje **zero równań i zero rodzajów zadania**. Dodaje jedne drzwi do
zbudowania celu, który już istnieje.

---

## 2. Którymi drzwiami wchodzi kwaternion

### 2.1 Wariant A — trzecia alternatywa w `IkTarget`

```cpp
struct QuaternionPoseTarget { Vector3 position; Quaternion orientation; };
using IkTarget = std::variant<PositionTarget, PoseTarget, QuaternionPoseTarget>;
```

**Odrzucony.** Konsekwencje sięgają daleko poza ten etap:

- `IkTaskKind` musiałby dostać trzecią wartość albo dwie alternatywy mapowałyby
  się na `Pose` — a `IkEquationSystem::create` waliduje liczbę i rodzaj równań
  właśnie po `IkTaskKind`;
- `TargetVisitor` w `TargetValidation.cpp` jest **jawnym zestawem przeciążeń
  właśnie po to**, żeby trzecia alternatywa nie skompilowała się bez decyzji —
  ta decyzja zapadłaby wtedy tutaj, przy okazji;
- `ConstraintBuilder` dostałby trzecie przeciążenie, które w środku i tak
  konwertuje i woła to samo, co drugie.

Cena jest realna, a zysk zerowy: **kwaternion i macierz opisują to samo
ograniczenie**. Rozgałęzianie typu zadania po reprezentacji wejścia oznacza, że
solver z Fazy 3 dostałby dwie ścieżki tam, gdzie problem jest jeden.

### 2.2 Wariant B — alternatywa wewnątrz `PoseTarget`

```cpp
struct PoseTarget { Vector3 position; std::variant<RotationMatrix3, Quaternion> orientation; };
```

**Odrzucony, i to mocniej niż A.** `PoseTarget` jest modelem zatwierdzonym w
F2.3, czytanym dziś przez dwa miejsca (`validate`, `ConstraintBuilder`), a
docelowo przez solver. Wariant B przenosi konwersję w głąb — do każdego
konsumenta z osobna — czyli dokładnie odwrotnie, niż powinien: **konwersja należy
do granicy, nie do środka.** Dodatkowo `ConstraintBuilder::appendOrientationEquations`
musiałby wołać konwersję w trakcie budowania równań, więc etap dopisujący
„tylko wejście" zmieniałby builder zatwierdzony tydzień wcześniej.

### 2.3 Wariant C — konwersja na granicy modelu ✅ **rekomendowany**

Jedna funkcja wytwórcza, po niej wszystko jest jak dotąd:

```cpp
std::expected<PoseTarget, TargetValidationError>
makePoseTargetFromQuaternion(const Vector3& position, const Quaternion& orientation);
```

```text
Quaternion ─────────┐
                    ├─> makePoseTargetFromQuaternion ─> PoseTarget ─> ConstraintBuilder ─> IkEquationSystem
RotationMatrix3 ────┘   (agregat, jak dziś — wchodzi wprost)
```

Co to daje:

| | |
|---|---|
| Zmiany w `IkTarget`, `IkTaskKind`, `IkEquationSystem`, `ConstraintBuilder` | **zero** |
| Gate „ten sam `IkEquationSystem` dla kwaternionu i macierzy" | spełniony **konstrukcyjnie**, nie zbiegiem okoliczności — po konwersji istnieje jedna ścieżka |
| Liczba miejsc, które muszą znać kwaterniony | jedno |
| Odwracalność decyzji | pełna: gdyby wariant A kiedyś okazał się potrzebny, `makePoseTargetFromQuaternion` zostaje jako jego implementacja |

### 2.4 Co wariant C traci — uczciwie

1. **`PoseTarget` nie pamięta, że przyszedł z kwaternionu.** Nie da się później
   zaraportować „cel podano jako kwaternion". Nikt tego dziś nie potrzebuje: to,
   co konsumuje cel — równania i solver — pracuje na liczbach po prawej stronie,
   a nie na historii wejścia. Gdyby stało się potrzebne (komunikat
   diagnostyczny), wraca wariant A.
2. **Nie ma drogi powrotnej macierz → kwaternion.** Świadomie: napisanie jej „na
   wszelki wypadek" oznacza wybór gałęzi (`w` blisko zera to klasyczna pułapka
   `sqrt` z ujemnego argumentu) bez konsumenta, który by ten wybór zweryfikował.
   Otwarte pytanie 9a B.
3. **Konwersja liczy się dwa razy przy walidacji.** `makePoseTargetFromQuaternion` produkuje
   macierz, a `ConstraintBuilder` i tak zawoła na niej `validate(PoseTarget)`.
   To ~50 flopów na cel — i to nie jest marnotrawstwo, tylko granica: builder nie
   ma prawa zakładać, kto zbudował jego argument. Ale **wymusza** warunek z 3.3:
   macierz wyprodukowana tutaj musi przechodzić tamtą walidację zawsze.

---

## 3. Walidacja kwaternionu

### 3.1 Kolejność kroków — i pułapka `NaN`

```text
1. wszystkie cztery składowe skończone       -> NonFiniteQuaternion
2. | ||q|| - 1 | <= kQuaternionNormTolerance -> NonNormalizedQuaternion
```

Kolejność nie jest kosmetyczna, jest **warunkiem poprawności**. Napisane
odwrotnie — albo bez kroku 1 — sprawdzenie normy wygląda tak:

```cpp
const double n = std::sqrt(w*w + x*x + y*y + z*z);  // NaN, jeśli którakolwiek składowa to NaN
if (std::abs(n - 1.0) > tolerance)                  // NaN > tolerance  ==  FAŁSZ
    return reject;                                  // ...więc NIE odrzuca
```

`NaN` **przechodzi walidację**, konwersja produkuje macierz pełną `NaN`, a
`validate(RotationMatrix3)` łapie to dopiero w `ConstraintBuilder` — jeden
poziom za późno i pod cudzym kodem błędu. To dokładnie ten sam kształt błędu,
który review z 2026-08-17 znalazło w `RobotModel::within_limits()`
(`v < lower || v > upper` jest fałszywe po obu stronach dla `NaN`), usuniętym w
R4 i zapisanym w `STATUS.md` jako pułapka do niepowtórzenia. Powtórzenie go w
etapie, który sam siebie nazywa walidacją wejścia, byłoby wpadką z gatunku
„wiedzieliśmy".

Kwaternion zerowy `{0,0,0,0}` odpada na kroku 2 (`|0-1| = 1`), więc **do
dzielenia przez normę nigdy nie dochodzi z normą bliską zeru** — patrz 3.3.

### 3.2 Tolerancja normalizacji — zmierzona, nie wybrana

`kOrientationTolerance = 1e-8` zostało zmierzone dla **macierzy** i dotyczy innej
wielkości: odchylenia `RᵀR` od `I`. Norma czterowektora to co innego i dostaje
własną stałą.

Pomiar (MinGW g++, `-O2`, ziarna stałe; 200 000 obrotów dla wiersza pierwszego,
1 000 000 dla drugiego):

| Populacja wejścia | najgorsze | mediana | ile mieści się w `1e-8` |
|---|---|---|---|
| kwaternion jednostkowy zapisany tekstem z 9 miejscami po przecinku | **9.03e-10** | — | 100% |
| kwaternion jednostkowy po przejściu przez `float` | 5.02e-8 | 1.22e-8 | **41.65%** |

Propozycja:

```cpp
inline constexpr double kQuaternionNormTolerance = 1e-8;
```

Uzasadnienie jest jednostronne i tylko takie być może (3.4): próg przyjmuje z
**11-krotnym zapasem** całą populację, którą kontrakt wejściowy obiecuje
przyjmować, i odrzuca wszystko, co nie wygląda na kwaternion jednostkowy zapisany
z rozsądną precyzją. **Nie** mówi natomiast nic o tym, skąd dane pochodzą — drugi
wiersz tabeli jest tego dowodem, nie potwierdzeniem.

Warto też powiedzieć, czego ta stała przy N2 **nie** robi: nie jest granicą
poprawności. Po normalizacji (3.3) wyprodukowana macierz jest ortogonalna do
~2e-15 niezależnie od tego, gdzie w paśmie leżało wejście, więc wartość progu
decyduje wyłącznie o tym, **jak wcześnie krzyczymy** na dane, które nie wyglądają
na to, za co się podają. W N1 byłoby odwrotnie — tam próg byłby klifem
numerycznym, i to jest kolejny argument za N2.

Wartość jest liczbowo **równa** `kOrientationTolerance` i jest to zbieg
okoliczności dwóch niezależnych pomiarów, a nie współdzielenie progu. Dlatego
**osobna stała, a nie alias**: gdyby kiedyś zaostrzyć próg macierzowy,
kwaternionowy nie ma iść za nim automatycznie, bo mierzy co innego. Roadmapa
wymagała tolerancji „niezależnej od `kOrientationTolerance`" — i to jest ta
niezależność: w derywacji, nie w cyfrach.

### 3.3 Normalizować po przyjęciu — tak. I dlaczego to nie jest „naprawa"

Projekt ma twardą zasadę: **walidacja odrzuca, nigdy nie naprawia** (README,
`TargetValidation.cpp`). Pytanie brzmi więc, czy podzielenie przyjętego
kwaternionu przez jego normę tej zasady nie łamie.

Najpierw liczby, bo one przesądzają. Dla `q = (1+ε)·q̂` wzór algebraiczny daje

```text
R(q) = R̂ + δ(R̂ − I),   δ = ‖q‖² − 1 ≈ 2ε
```

czyli odchylenie liniowe w `ε`, z czynnikiem do 4 od `2I − (R̂ + R̂ᵀ)`. Zmierzone
(te same 200 000 obrotów plus obrót o 180° jako przypadek najgorszy analitycznie):

| `ε = \| ‖q‖−1 \|` | `max \|RᵀR − I\|` | `max \|det − 1\|` |
|---|---|---|
| 1e-9 | 8.0e-9 | 8.0e-9 |
| **1.25e-9** | **1.0e-8** | **1.0e-8** |
| 1e-8 | 8.0e-8 | 8.0e-8 |
| 1e-6 | 8.0e-6 | 8.0e-6 |

Wiersz trzeci to wyrok. **Bez normalizacji** kwaternion przyjęty przy progu
`1e-8` daje macierz odchyloną o `8e-8`, czyli **osiem razy poza**
`kOrientationTolerance`: `makePoseTargetFromQuaternion` zwracałby `PoseTarget`, który
`validate()` tego samego modelu odrzuca. Model produkujący wartość, której własny
walidator nie przyjmuje, to nie kompromis, tylko błąd.

Wyjścia są dwa:

- **N1 — nie normalizować, zacisnąć próg.** Z tabeli: próg musiałby wynosić
  `1.25e-9`, żeby dopiero *dotknąć* `1e-8`. Odrzucone z dwóch powodów. Po
  pierwsze, zapas nad tekstem z dziewięcioma miejscami (9.03e-10 z 3.2) byłby
  1.1× — praktycznie żaden, a wejście z ośmioma miejscami już by nie
  przechodziło. Po drugie, stała przestałaby być własna: byłaby ilorazem cudzej
  stałej przez zmierzony czynnik, więc każda zmiana `kOrientationTolerance` po
  cichu przestawiałaby próg kwaternionowy. Dokładnie to, czego roadmapa zakazała.
- **N2 — przyjąć w wąskim paśmie, potem podzielić przez normę.** ✅

Po normalizacji, na tych samych 200 000 obrotów podanych z 9 miejscami po przecinku:

```text
max |RᵀR − I| = 2.22e-15        max |det − 1| = 2.44e-15
```

— około **cztery miliony** razy wewnątrz `kOrientationTolerance`, i niezależnie
od tego, gdzie w paśmie leżało wejście. Sprzężenie między dwiema tolerancjami **znika**,
zamiast być strojone.

**Dlaczego dzielenie przez normę nie jest naprawą danych.** Bo długość
czterowektora nie niesie żadnej informacji o obrocie: obrót jest zakodowany w
**kierunku** w `R⁴`, a norma jest artefaktem zapisu. Precyzyjnie — i ważne, bo v1
miała to zdanie za szerokie:

```text
q i 2q wyznaczają ten sam kierunek, więc po normalizacji dają ten sam
kwaternion jednostkowy i ten sam obrót.

q i 2q NIE dają tej samej macierzy, jeśli włożyć je wprost do wzoru z 4.1 —
ten wzór zakłada wejście unitarne i poza nim nie liczy obrotu w ogóle
(dla 2q wychodzi R = R̂ + 3(R̂ - I), δ = 3).
```

Czyli: nieunitarny kwaternion **nie jest** sam w sobie poprawną reprezentacją
obrotu — staje się nią dopiero po kanonizacji. I dokładnie to robi
`makePoseTargetFromQuaternion`: kanonizuje reprezentację, nie poprawia danych.

Porównanie z macierzą, gdzie zasada „odrzucaj, nie naprawiaj" powstała, jest
rozstrzygające:

| | macierz nie-ortogonalna | kwaternion nieznormalizowany |
|---|---|---|
| Czy „zamierzony obrót" jest jednoznaczny? | **Nie** — najbliższy obrót zależy od wybranej normy i algorytmu (SVD, Gram–Schmidt, iteracja), dwa rozsądne wybory dają różne wyniki | **Tak** — `q/‖q‖`, jedno dzielenie, zero wyborów |
| Czy naprawa zmienia zadanie caller'a? | Tak, o wielkość, której caller nie zna | Nie — obrót jest ten sam z dokładnością do zaokrągleń |
| Czy caller zrobiłby to sam tak samo? | Nie ma jednej „tej samej" metody | Tak, i wyjdzie mu bit w bit to samo |

Kontrola normy zostaje mimo to — jej zadaniem nie jest chronić semantykę, tylko
**łapać śmieci**: strukturę wypełnioną w połowie, `{0,0,0,0}`, wartość
przemnożoną przez skalę, wektor, który kwaternionem nigdy nie był. `q` o normie
1.7 to dowód czyjegoś błędu i ma się zatrzymać tutaj, a nie zostać po cichu
przeskalowany.

Czego norma **nie** łapie, i co v1 błędnie tu wpisywała: **danych w innej
konwencji**. Przestawienie `(w,x,y,z)` ↔ `(x,y,z,w)` jest permutacją składowych,
więc norma zostaje identyczna, a obrót wychodzi inny — poprawny, tylko nie ten.
Ten błąd jest niewykrywalny na tym poziomie i figuruje jako taki w tabeli ryzyk
(8). Drugi z tej samej rodziny — pochodzenie danych — ma własną sekcję (3.4).

### 3.4 Czego norma nie mówi: pochodzenia danych

v1 twierdziła, że próg `1e-8` odrzuca dane, które przeszły przez `float`. **To jest
nieprawda i nie może być kontraktem.** Wniosek był wyciągnięty z jednej liczby —
najgorszego przypadku `4.98e-8` — podczas gdy liczy się rozkład. Odtworzone
niezależnie, 1 000 000 kwaternionów jednostkowych przepuszczonych przez
`double → float → double`:

```text
najgorsze  |‖q‖-1| = 5.02e-8
mediana    |‖q‖-1| = 1.22e-8
<= 1e-8                41.65%     <- przechodzi walidację
<= 1e-9                 4.29%
identyczność          błąd 0     <- (1,0,0,0) jest dokładna w float
(.5,.5,.5,.5)          błąd 0     <- 120° wokół (1,1,1) też
```

Prawie połowa przechodzi, a wartości dokładnie reprezentowalne w `float` —
identyczność, obroty o wielokrotność 90° wokół osi głównych, `(.5,.5,.5,.5)` —
przechodzą **zawsze**, bo ich norma po konwersji jest dokładnie 1. Żadna
tolerancja tego nie zmieni: informacja o precyzji pośrednich obliczeń **nie jest
zawarta w wartości kwaternionu**. Żeby ją mieć, trzeba by prowenancji — osobnego
typu wejściowego albo znacznika źródła — i tego świadomie **nie** dokładamy: to
kontrakt o pochodzeniu danych, a nie o geometrii, i należałby do warstwy, która
dane pobiera.

Kontrakt tego etapu brzmi więc dokładnie tyle, ile da się dotrzymać:

```text
q skończony
+ | ‖q‖ - 1 | <= kQuaternionNormTolerance
                      -> przyjęty, znormalizowany, skonwertowany

w przeciwnym razie
                      -> odrzucony
```

To jest **miara jakości reprezentacji, którą faktycznie dostaliśmy**, a nie
detektor tego, jak ta reprezentacja powstała.

Praktyczna konsekwencja jest łagodna i warto ją podać, bo zdejmuje presję z tej
decyzji. Dla kwaternionów po `float`, które **przechodzą** próg, zmierzone
odchylenie obrotu od dokładnego (po normalizacji, kąt między obrotami) wynosi
najwyżej:

```text
9.88e-8 rad  =  5.7e-6 stopnia   =  0.2 µm na ramieniu 2 m
```

Czyli dane, które prześlizgnęłyby się mimo `float`, są fizycznie nieodróżnialne od
poprawnych, a te odrzucone są odrzucone za jakość zapisu, nie za błąd geometrii.
Symetria z drzwiami macierzowymi też pozostaje nienaruszona, tylko brzmi inaczej,
niż v1 chciała: w obu drzwiach sprawdzamy, **czy to, co dostaliśmy, jest tym, za
co się podaje**, i w żadnych nie zgadujemy, skąd przyszło.

Dla macierzy różnica jest jednak realna i zostaje: macierz po `float` odchyla się
od ortogonalności o ~1e-6, czyli **poza** `kOrientationTolerance` niemal zawsze —
i tam odrzucenie jest konieczne, bo naprawa (ortonormalizacja) jest
niejednoznaczna (3.3). Przy kwaternionie naprawa jest jednoznaczna, więc
przyjęcie części takich danych nie tworzy żadnego długu.

---

## 4. Konwersja

### 4.1 Wzór

Dla znormalizowanego `q = (w, x, y, z)`:

```text
        | 1−2(y²+z²)   2(xy−wz)    2(xz+wy) |
R(q) =  |  2(xy+wz)   1−2(x²+z²)   2(yz−wx) |
        |  2(xz−wy)    2(yz+wx)   1−2(x²+y²) |
```

Wybrany, bo dla wejścia znormalizowanego jest formą **najkrótszą i najlepiej
uwarunkowaną** z trzech rozważanych: wariant „jednorodny"
`R = ((w²−‖v‖²)I + 2vvᵀ + 2w[v]×)/‖q‖²` liczy to samo drożej, a wariant „trzy
kanapki `q⊗eᵢ⊗q*`" trzykrotnie powtarza tę samą algebrę. Do celów **testowych**
oba pozostają cenne właśnie dlatego, że idą inną drogą (4.2).

Normalizacja dzieje się **w tej samej funkcji**, tuż przed wzorem, i nigdy nie
dostaje normy bliskiej zeru: walidacja z 3.1 przepuszcza wyłącznie
`‖q‖ ∈ [1−1e-8, 1+1e-8]`.

### 4.2 Niezależność od `tests/support` — co dokładnie znaczy

Warunek roadmapy ma dwie warstwy i tylko jedną z nich da się sprawdzić testem.

**Warstwa strukturalna — brak współdzielenia kodu.** Wymuszona przez budowę, nie
przez dyscyplinę: `tests/support/NumericForwardKinematics.cpp` jest źródłem celu
`kinemaforge_tests`, a nie `kinemaforge_ik`. Produkt nie ma jak zawołać tamtej
funkcji, bo jej u siebie nie widzi. Żaden test tego nie „sprawdza" — sprawdza to
linker, przy każdym buildzie.

**Warstwa merytoryczna — brak wspólnego błędu.** Tu ryzyko jest realne: dwie
implementacje tego samego wzoru algebraicznego mogą być błędne **identycznie**
(przepisana literówka, ta sama pomylona konwencja znaku). Dlatego test
równoważności nie porównuje wzoru ze wzorem, tylko produkcyjną macierz z dwiema
**innymi drogami**:

```text
1. R·v   kontra   rotate(q, v) z oracle'a
   rotate() nigdy nie buduje macierzy — to kanapka kwaternionowa
   v + 2w(u×v) + 2u×(u×v). Zmierzona zgodność: 1.78e-15 dla |v| ~ 1.7.

2. R(q)  kontra   wzór Rodriguesa R = I + sinθ [k]× + (1−cosθ) [k]×²
   zbudowany z tej samej pary (oś, kąt), z której powstał kwaternion.
   Droga trygonometryczna, nie algebraiczna. Zmierzona zgodność: 1.11e-15.
```

Droga 2 jest ważniejsza, niż wygląda: jako jedyna nie przechodzi przez
kwaterniony w ogóle, więc jako jedyna wychwyci pomyloną konwencję
`(w,x,y,z)`/`(x,y,z,w)` albo `q*` zamiast `q` — czyli błędy, które w świecie
kwaternionowym są spójne same ze sobą.

Zmierzone `1.1e-15` przy progu testowym `1e-12` daje ~900× zapasu, więc test nie
będzie migotał.

### 4.3 Podwójne pokrycie: `q` i `−q`

Wzór jest kwadratowy w składowych, więc `R(−q) = R(q)` **dokładnie** — co do
wartości każdej komórki, nie tylko z dokładnością do tolerancji;
każdy iloczyn zawiera parzystą liczbę czynników ze zmienionym znakiem. To nie
jest przybliżenie i zasługuje na test, bo jest to jedyna własność odróżniająca
poprawnie przepisany wzór od takiego, w którym ktoś zgubił jeden czynnik `w`.
Konsekwencja dla caller'a: dwa różne kwaterniony dają **ten sam** `PoseTarget` —
i to jest poprawne, reprezentują ten sam obrót.

---

## 5. API

### `src/ik_equations/model/Quaternion.hpp` (nowy)

```cpp
#pragma once

namespace kinemaforge::ik {

// (w, x, y, z) -- skalar PIERWSZY. ROS/tf2 i Eigen::Quaterniond::coeffs()
// serializują (x, y, z, w); przepisanie czwórki bez zmiany kolejności daje
// obrót, który jest poprawnym obrotem, tylko innym. Typ tego nie wykryje i nie
// udaje, że wykryje -- dlatego przy wypełnianiu z zewnętrznego źródła używać
// inicjalizatorów desygnowanych:
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

Osobny nagłówek, precedens: `model/Vector3.hpp`. Nie trafia do `IkTarget.hpp`,
bo tamten trzyma **zamknięty zbiór celów**, a kwaternion celem nie jest.

### `src/ik_equations/model/TargetValidation.hpp` (rozszerzenie)

```cpp
enum class TargetValidationErrorCode
{
    NonFinitePosition,
    NonFiniteOrientation,
    NonOrthogonalOrientation,
    InvalidOrientationDeterminant,
    NonFiniteQuaternion,        // nowe
    NonNormalizedQuaternion     // nowe
};

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

// Sprawdzane w tej kolejności, i kolejność jest obserwowalna:
//   1. cztery składowe skończone     -> NonFiniteQuaternion
//   2. | ||q|| - 1 | <= tolerancja   -> NonNormalizedQuaternion
//
// Krok 1 jest osobny, bo NaN przechodzi przez KAŻDE porównanie w kroku 2:
// "NaN > tolerancja" jest fałszem, więc test napisany bez kroku 1 przyjmuje NaN
// jako kwaternion jednostkowy. Ten sam kształt błędu, który review z 2026-08-17
// znalazło w RobotModel::within_limits() (usuniętym w R4).
[[nodiscard]] std::expected<void, TargetValidationError> validate(const Quaternion& quaternion);
```

### `src/ik_equations/model/QuaternionTarget.hpp` (nowy)

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
// Pozycja i orientacja są walidowane w tej samej kolejności, co w
// validate(PoseTarget).
//
// q i -q to ten sam obrót i dają ten sam PoseTarget, komórka po komórce.
[[nodiscard]] std::expected<PoseTarget, TargetValidationError>
makePoseTargetFromQuaternion(const Vector3& position, const Quaternion& orientation);

} // namespace kinemaforge::ik
```

**Czego tu celowo nie ma:** publicznego `toRotationMatrix(const Quaternion&)`.
Taka funkcja musiałaby albo mieć niesprawdzalny warunek wstępny („kwaternion już
zwalidowany" — a `{0,0,0,0}` daje wtedy dzielenie przez zero i macierz `NaN`),
albo zwracać `expected` i być drugimi drzwiami z własnym kontraktem. Konwersja
zostaje w anonimowej przestrzeni nazw w `.cpp`, a testy sięgają do niej przez
`makePoseTargetFromQuaternion(...)->orientation`, czyli dokładnie tak, jak zrobi to każdy
konsument.

---

## 6. Testy

**`tests/test_target_validation.cpp`** — walidacja, bo tam mieszka
`TargetValidation` (konwencja: plik testowy ↔ plik produkcyjny):

```text
AcceptsUnitQuaternion                       identyczność i obrót 90° wokół osi skośnej
AcceptsNegatedUnitQuaternion                -q przechodzi tak samo
RejectsNonFiniteQuaternion                  inf, każda z czterech składowych osobno
RejectsNaNQuaternionInsteadOfAcceptingIt    jawny test pułapki z 3.1: NaN w w,
                                            oczekiwany kod NonFiniteQuaternion,
                                            NIE NonNormalizedQuaternion
RejectsZeroQuaternion                       {0,0,0,0} -> NonNormalizedQuaternion
RejectsNonNormalizedQuaternion              norma 1.5  [roadmapa]
AcceptsQuaternionAtNormToleranceEdge        |n-1| = 9e-9 przechodzi
RejectsQuaternionJustOutsideNormTolerance   |n-1| = 1.1e-8 odpada
QuaternionErrorMessageCarriesFullPrecision  format() z max_digits10, precedens z
                                            wyznacznika: "1.000000" byłoby gorsze
                                            niż brak komunikatu
AcceptsQuaternionThatCameThroughFloat       identyczność i (.5,.5,.5,.5) po
                                            double->float->double przechodzą —
                                            pinuje kontrakt z 3.4, żeby nikt później
                                            nie "naprawił" progu na detektor
                                            precyzji, którym nie może być
```

**`tests/test_quaternion_target.cpp`** (nowy) — konwersja i cel:

```text
AcceptsValidQuaternionTarget                [roadmapa]
ProducesEquivalentTargetsForQuaternionAndMatrix   [roadmapa] — patrz niżej
RejectsNonNormalizedQuaternionTarget        [roadmapa] przez funkcję wytwórczą;
                                            błąd przechodzi bez spłaszczenia
IdentityQuaternionProducesIdentityRotation  dokładnie I, bez tolerancji
NegatedQuaternionProducesSameTarget         wartości dokładnie równe (4.3)
MatchesQuaternionRotationWithoutFormingAMatrix    [roadmapa, wpis zmieniony
                                            2026-08-21] R·v kontra oracle
                                            rotate(), próg 1e-12
AgreesWithRodriguesRotationFormula          [roadmapa, wpis zmieniony
                                            2026-08-21] droga trygonometryczna,
                                            nieprzechodząca przez kwaterniony,
                                            próg 1e-12
ProducedOrientationAlwaysPassesTargetValidation   200 pseudolosowych obrotów ze
                                            stałym ziarnem, zaokrąglonych do 9
                                            miejsc; każdy wynik przechodzi validate()
RejectsNonFinitePositionInQuaternionTarget  pozycja walidowana jak w PoseTarget
BuildsKr640FullPoseSystemFromQuaternion     end-to-end, gate etapu — patrz niżej
BuildsKr4FullPoseSystemFromQuaternion       to samo na drugim robocie; kr4_r600
                                            ma niezerowe rpy w originach, więc
                                            jego macierz FK nie ma zer, którymi
                                            pomylona komórka mogłaby się zasłonić
```

Trzy testy wymagają komentarza.

**`ProducesEquivalentTargetsForQuaternionAndMatrix`** nie może porównywać przez
`==`, i to nie z ostrożności, tylko z rachunku. „Równoważna macierz" znaczy jedno
z dwojga: albo jest **wynikiem tej samej konwersji** — wtedy równość jest bitowa,
a test tautologią — albo caller wpisał ją ręcznie, a wtedy różni się o rząd
własnego zaokrąglenia (~1e-9 dla dziewięciu miejsc po przecinku), co jest
własnością jego danych, nie naszej konwersji. Test porównuje więc z macierzą
zbudowaną **niezależnie** (Rodrigues z tej samej osi i kąta) przy progu `1e-12` —
i to samo robi na poziomie całego układu równań.

**`BuildsKr640FullPoseSystemFromQuaternion`** jest gate'em etapu. v1 opisywała go
odwrotnie do tego, co oracle potrafi: **nie ma tam konwersji macierz →
kwaternion** i nie ma jej mieć (`tests/support` idzie w drugą stronę — niesie
kwaternion przez całe FK i macierz składa dopiero na końcu). Test ma więc dwie
**równoległe ścieżki z jednej konfiguracji**, każda w swojej reprezentacji od
początku do końca:

```text
                          konfiguracja q
                        /                \
   symboliczne FK → evaluate → R        numeryczny oracle → q_oracle
            |                                      |
   PoseTarget{position, R}              makePoseTargetFromQuaternion(position, q_oracle)
            \                                      /
              ConstraintBuilder → dwa IkEquationSystem
                          porównanie
```

Porównanie: identyczna struktura (`taskKind`, `source`, kolejność równań, `lhs`
przez `sameNode` — obie ścieżki dostają ten sam `SymbolicTransform`) i prawe
strony w promieniu `1e-12`, czyli tego samego, którym związana jest istniejąca
krzyżowa walidacja FK.

Ta wersja jest **mocniejsza** niż opisana w v1, i to jest właściwy powód zmiany,
nie tylko wykonalność: żadna ze stron nie jest przeliczeniem drugiej. Lewa idzie
macierzami przez całą warstwę symboliczną, prawa kwaternionami przez oracle i
wchodzi do produktu dopiero w `makePoseTargetFromQuaternion`. Zgodność dwunastu prawych stron
oznacza wtedy, że nowa konwersja zgadza się z niezależną implementacją FK, a nie
sama ze sobą. Test jest przy tym niewrażliwy na wybór znaku przez oracle: `q` i
`−q` dają tę samą macierz (4.3).

**`ProducedOrientationAlwaysPassesTargetValidation`** jest testem własności, nie
przypadku, bo broni warunku, na którym stoi cały wariant C (2.4, punkt 3): każdy
`PoseTarget` wychodzący z `makePoseTargetFromQuaternion` musi przechodzić walidację, którą
zaraz wykona `ConstraintBuilder`.

Razem **+10** do `test_target_validation.cpp` i **+11** w nowym pliku, czyli gate
`365/365` (344 + 21). Dokładna liczba jest wiążąca dopiero w części 2.

---

## 7. Plan zmian w plikach

```text
src/ik_equations/model/Quaternion.hpp          NOWY, sam typ (~20 linii)
src/ik_equations/model/QuaternionTarget.hpp    NOWY, makePoseTargetFromQuaternion + kontrakt
src/ik_equations/model/QuaternionTarget.cpp    NOWY, konwersja w anon. namespace
src/ik_equations/model/TargetValidation.hpp    +2 kody błędu, +stała, +1 przeciążenie
src/ik_equations/model/TargetValidation.cpp    +validate(Quaternion)
CMakeLists.txt                                 +1 źródło w kinemaforge_ik
tests/test_target_validation.cpp               +10 testów
tests/test_quaternion_target.cpp               NOWY, +11 testów
tests/CMakeLists.txt                           +1 plik testowy
STATUS.md                                      F2.5Q, licznik testów, wiersz w
                                               "Known gaps": próg normy nie mówi
                                               nic o pochodzeniu danych (3.4)
proposal/KinemaForge_Phase2_Phase3_Roadmap.md  ZROBIONE 2026-08-21: minimalne testy
                                               F2.5Q, jeden wpis ogólny -> dwa
                                               konkretne (6). Po implementacji:
                                               F2.5Q -> zaimplementowane, następny etap
```

**Zero zmian** w: `IkTarget.hpp`, `IkEquationSystem.{hpp,cpp}`, `Equation.hpp`,
`ConstraintBuilder.{hpp,cpp}`, warstwie symbolicznej, loaderze, budowniczych
transformacji, fasadzie. Żaden istniejący test nie zmienia asercji — i to jest
sprawdzian, że drzwi są rzeczywiście na granicy: gdyby trzeba było ruszyć
`ConstraintBuilder`, znaczyłoby, że wybrano wariant B, tylko nie wprost.

`TargetValidationError` **nie zyskuje nowego typu**, tylko dwa kody. Uzasadnienie
z v1 było przy tym błędne i zostaje wycofane: `ConstraintBuilder` tych dwóch
kodów **nigdy nie zobaczy**, bo `makePoseTargetFromQuaternion` zwraca błąd, zanim jakikolwiek
`PoseTarget` powstanie — to nie jest ta ścieżka wykonania. Powodem jest co innego,
i wystarczającym: caller budujący cel z kwaternionu dostaje **jedno `expected` i
jeden słownik błędów** na obie połowy tego samego celu — pozycję i orientację —
zamiast składać wynik z dwóch niekompatybilnych typów. `TargetValidationError`
jest już „błędem w danych celu", a nie „błędem macierzy".

Niezależnie od tego ścieżka z F2.4 (`ConstraintBuilderError::targetError`) działa
bez zmiany — po prostu dlatego, że nic w niej nie ruszamy.

---

## 8. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| `NaN` przechodzi walidację normy | **realne, znane z historii repo** | Osobny krok skończoności przed normą + `RejectsNaNQuaternionInsteadOfAcceptingIt`, który pilnuje także **kodu** błędu (3.1) |
| Konwersja odchyla macierz poza `kOrientationTolerance` | **realne, policzone** | Normalizacja przed wzorem; 2.2e-15 zamiast 8e-8; `ProducedOrientationAlwaysPassesTargetValidation` jako test własności (3.3) |
| Dwie implementacje tego samego wzoru mylą się identycznie | realne | Sprawdzenie dwiema **innymi drogami**: kanapka kwaternionowa z oracle'a i Rodrigues z trygonometrii (4.2) |
| Caller wpisuje `(x,y,z,w)` | **realne, nie do wykrycia typem ani normą** | Permutacja składowych zachowuje normę, więc żaden próg tego nie złapie. Komentarz przy typie + inicjalizatory desygnowane w każdym przykładzie — i proposal tego nie ukrywa (3.3) |
| Dane po `float` przechodzą w części przypadków | **realne, zmierzone: 41.65%** | Świadome i niedomykalne bez prowenancji (3.4). Przy N2 nie jest to problem poprawności: przyjęty kwaternion po `float` niesie błąd obrotu ≤ 9.9e-8 rad, a wyprodukowana macierz jest ortogonalna do ~2e-15 |
| Normalizacja maskuje błąd skalowania u caller'a | niskie | Pasmo jest wąskie — przechodzi tylko to, co już wygląda na kwaternion jednostkowy; skala `1.7` czy `{0,0,0,0}` odpada (3.3) |
| Ktoś w przyszłości zawoła konwersję z pominięciem walidacji | niskie | Nie da się: konwersja nie jest publiczna (5) |
| Rozrost `TargetValidationErrorCode` | świadome | Dwa kody zamiast nowego typu błędu; alternatywa psuła ścieżkę zachowania błędu z F2.4 (7) |

---

## 9. Rozstrzygnięte w review (v2)

| # | Pytanie / twierdzenie z v1 | Rozstrzygnięcie |
|---|---|---|
| 1 | „Czy odrzucać kwaterniony po `float`?" (9.1 w v1) | **Pytanie wycofane, bo było źle postawione.** Z wartości kwaternionu nie da się odczytać jego pochodzenia: 41.65% danych po `float` przechodzi próg, a wartości dokładnie reprezentowalne przechodzą zawsze. Walidujemy jakość otrzymanej reprezentacji, nie historię obliczeń (3.4) |
| 2 | Czy norma łapie „dane w innej konwencji" | **Nie.** Permutacja `(w,x,y,z)` ↔ `(x,y,z,w)` zachowuje normę. Usunięte z 3.3, zostaje w tabeli ryzyk jako jawnie niewykrywalne (8) |
| 3 | „`q` i `2q` to ten sam obrót — dosłownie" | **Za szerokie.** Ten sam jest **kierunek** w `R⁴`; wzór z 4.1 zakłada wejście unitarne i dla `2q` daje inną macierz. Stąd normalizacja jest kanonizacją reprezentacji, a nie naprawą danych (3.3) |
| 4 | Kształt testu end-to-end | **Przepisany na dwie równoległe ścieżki** z jednej konfiguracji; oracle nie ma konwersji macierz → kwaternion i mieć jej nie będzie (6) |
| 5 | Dlaczego wspólny `TargetValidationError` | **Nie z powodu `ConstraintBuilderError`** — builder tych kodów nie zobaczy. Powód: jedno `expected` i jeden słownik błędów na cały cel (7) |
| 6 | `kQuaternionNormTolerance = 1e-8` | **Zostaje**, z nową argumentacją: jednostronną (przyjmuje kontrakt wejściowy z 11× zapasem) i bez roli detektora. Przy N2 nie jest granicą poprawności (3.2) |
| 7 | `2.44e-15` wobec `1e-8` | Poprawka rachunkowa: **~4 miliony razy**, nie cztery tysiące (3.3) |

## 9a. Zamknięte w review v2

**A. Nazwa funkcji wytwórczej — `makePoseTargetFromQuaternion`.** Review zgłosiło
preferencję do `poseTargetFromQuaternion` zamiast `makePoseTarget`, jako miękką —
i argument jest słuszny: nazwa ma mówić na call-site, które **drzwi wejściowe** są
użyte, i ma się skalować na kolejne formy wejścia. Przyjmuję argument, zachowując
prefiks `make`, który jest w tym repo konwencją dla wolnych funkcji wytwórczych
(`makePrincipalRotation`, `makeRpyRotation`, `makeSymbolValues`) — stąd
**`makePoseTargetFromQuaternion`**.

Do argumentu review dochodzi twardsza przesłanka, która przesądza sprawę na
korzyść nazywania drzwi zamiast przeciążania po typie. Kolejne formy wejścia z
listy „poza zakresem" (1) **kolidują w systemie typów**:

```text
poza zakresem dziś, ale zapowiedziane:
    RPY               (Vector3 position, Vector3 rpy)
    wektor obrotu     (Vector3 position, Vector3 rotationVector)   <- ta sama sygnatura
    oś-kąt           (Vector3 position, Vector3 axis, double angle)
```

Dwie pierwsze są nierozróżnialne dla kompilatora, więc rodzina przeciążeń
`makePoseTarget(...)` rozpada się dokładnie w momencie, w którym miałaby zacząć
być przydatna — i rozwiązaniem byłoby wtedy i tak nazwanie drzwi, tylko że z
przemianowaniem istniejącego API. Teraz kosztuje to zero, bo kodu jeszcze nie ma.

**B. Konwersja odwrotna `RotationMatrix3 → Quaternion` — nie dodajemy.** Zgodnie z
rekomendacją review: brak konsumenta, a wejściowo trudna część to wybór stabilnej
gałęzi dla obrotów bliskich π (`w ≈ 0`, `sqrt` z wartości schodzącej do zera przez
błąd numeryczny). Napisana bez konsumenta byłaby kodem, którego nie da się
sensownie zweryfikować — przetestowalibyśmy własny wybór gałęzi przeciwko sobie
samemu. Naturalny moment: Faza 3, jeśli solver ma raportować rozwiązanie jako pozę.

Otwarte pytania: **brak**.

## 10. Rekomendacja

Wariant **C** (2.3), **N2** (walidacja w wąskim paśmie + normalizacja, 3.3),
`kQuaternionNormTolerance = 1e-8` i API z sekcji 5 zostały w review zatwierdzone;
v2 nanosi blocker i cztery korekty, nie rusza żadnej z tych decyzji. Jedyna
zmiana o skutku zewnętrznym to **zawężenie obietnicy**: etap nie twierdzi już, że
odrzuca dane po `float` — twierdzi to, co jest prawdą i sprawdzalne:

```text
q skończony i | ‖q‖ - 1 | <= 1e-8   ->   przyjęty, znormalizowany, skonwertowany
w przeciwnym razie                    ->   odrzucony
```

**Status: APPROVED w review v2 (2026-08-21)** — bez v3, z trzema poprawkami
naniesionymi wyżej. Następny krok to proposal implementacyjny z pełnym kodem
trzech nowych plików, rozszerzeniem `TargetValidation` i dwudziestoma jeden
testami, gate `365/365` w obu konfiguracjach:

```text
proposal/F2.5Q-quaternion-target/proposal-quaternion-target-implementation.md
```

Po F2.5Q w Fazie 2 zostają **F2.6** (constrainty kierunkowe — największy etap tej
fazy, cztery rodziny ograniczeń) i **F2.7** (integracja z fasadą, DoD Fazy 2,
release `v0.2-ik-equations`).
