# Proposal: postać kanoniczna — F3.2, architektura

**Etap:** F3.2 · **v8 — ZATWIERDZONY 2026-08-22** · część 1 z 2

> **v8, po siódmym review.** Jeden blocker i dwie pozostałości tekstowe:
>
> 1. **Predykat dokładności mnożenia nie może być zachowawczy.** v7 przyjęła
>    fałszywe negatywy jako „koszt jednego węzła" — a po 3.2.4 wiadomo, że
>    **psują postać normalną tak samo, jak fałszywe pozytywy psuły dziedzinę**:
>    `(1e200*3)*x` i `1e200*(3*x)` rozjechałyby się. Predykat jest teraz
>    **sound i complete**, zweryfikowany wobec arytmetyki wymiernej na 8000
>    parach (3.2.1).
> 2. Usunięte wzmianki o „`fma` + strażnik zakresu" i o zaokrągleniu przy
>    redukcji stałych — pozostałości po v5/v6, już nieprawdziwe.

> **v7, po szóstym review.** Trzy blockery — wszystkie w zwijaniu stałych i w
> domknięciu znaku — oraz **odpowiedź na pytanie architektoniczne**, które
> recenzent postawił jako ważniejsze od nich:
>
> 1. **`isnormal(p) && fma(...) == 0` nadal dawało fałszywy pozytyw.** Zmierzone:
>    `nextafter(2^-511)^2` ma wynik normalny i zerową resztę, a iloczyn jest
>    zaokrąglony — błąd `2^-1126` sam wpada w underflow. Kryterium jest teraz
>    **bitowe**, na mantysach (3.2.1).
> 2. **Reducer był deterministyczny, ale nie idempotentny.** Zmierzone dla
>    `2^52, 2^52+1, 2^52+3`: pierwszy przebieg zostawia dwie stałe, drugi łączy
>    je w jedną. Reducer ma teraz własny punkt stały (3.2.2).
> 3. **`(-1) * (a+b)` łamało własny postcondition** — dawało `Negate` nad sumą.
>    Końcowe nałożenie znaku idzie przez `canonicalNegate` (3.3).
> 4. **Czy zwijanie stałych jest warte tej złożoności?** Odpowiedź: **jest
>    konieczne**, i to nie dla oszczędności węzłów — bez niego dwa AC-równoważne
>    wejścia dają dwie różne postacie normalne (3.2.4).

> **v6, po piątym review.** Dwa blockery, oba w mechanizmie wprowadzonym w v5 —
> czyli w tym, który miał uratować gwarancję dokładności:
>
> 1. **`fma(a,b,-p) == 0` nie dowodzi dokładności mnożenia.** Zmierzone:
>    `denorm_min * 0.5` daje `p = 0` i resztę `fma = 0`, choć wynik matematyczny
>    to `2^-1075` — underflow przechodzi jako „dokładny". Test dostaje strażnika
>    na zakres normalny (3.2.1).
> 2. **Brak harmonogramu redukcji — dwa dopuszczalne parowania, dwie postacie.**
>    Zmierzone dla `1, 2, 2^53`: `[3, 2^53]` albo `[1, 2^53+2]`, oba złożone
>    wyłącznie z połączeń dokładnych. Harmonogram jest teraz częścią kontraktu
>    (3.2.2).
> 3. Poprawiony test, który po v5 obiecywał „jedną stałą", oraz postcondition
>    z 3.4 po rozszerzeniu konwencji znaku na `Divide`.

> **v5, po czwartym review.** Dwa blockery, oba w **domknięciu transformacji** —
> struktura klas jest już dobra, brakowało precyzji w tym, co jeden przebieg
> naprawdę gwarantuje:
>
> 1. **Redukcja stałych łamała gwarancję dziedziny symbolicznej.** Zmierzone:
>    `q + 1e16 + 1 + (-1e16)` ma `zeroSet = {-1}`, a po zwinięciu stałych
>    `zeroSet = {0}`. Zwijanie jest teraz **wyłącznie dokładne**, sprawdzane
>    2Sum/`fma`, a kształt odbudowy nie pozwala fabryce zwinąć reszty (3.2).
> 2. **Kolektor nie spłaszczał operatora odsłoniętego przez kanonizację dziecka.**
>    `a + -(-(b+c))` dawało `a + (b+c)`, więc `IsIdempotent` by poległ.
>    Kolektor **splice'uje** (3.4).
> 3. W ramach tej samej poprawki domknięte `Negate(Add(...))` — `-(a+b)` i
>    `0-(a+b)` dają jedną postać (3.4).
> 4. Zakres normalizacji znaku **rozszerzony na `Divide`** i powiedziany wprost,
>    żeby `F3.5` nie zostało z „znak jednoznaczny, ale nie przy dzieleniu" (3.3).

> **Stan tego dokumentu: ZATWIERDZONY.** Następny krok to proposal
> implementacyjny. Dwie uwagi nieblokujące z approvala idą tam, nie tutaj:
> jawny kontrakt platformowy IEEE-754 binary64 wraz z `static_assert`ami
> i pilnowaniem FTZ/DAZ, oraz oracle-testy predykatu z 3.2.1.

> **v4, po trzecim review.** Trzy blockery, wszystkie z próby złamania definicji:
>
> 1. **§6 mieszało dziedzinę algebraiczną z określonością w `double`.** Zmierzone:
>    `1/((1e16 + q) + (-1e16))` przy `q = 1` daje `DivisionByZero`, a po
>    normalizacji — wartość. Dokument rozdziela teraz **dwa kontrakty** i wybiera
>    semantykę algebraiczną, z uzasadnieniem mocniejszym, niż się spodziewałem:
>    przy `q = 3` oryginał liczy się na **4** (6).
> 2. **Parzystość znaku nie widziała ujemnych stałych.** `(-2)*x` niesie
>    `Constant{-2}`, nie `Negate(Constant{2})`, więc wychodziło obok `-(2*x)`
>    jako druga postać (3.3).
> 3. **`Subtract` nie był domknięty dla zagnieżdzeń.** `a - (b - c)`
>    i `(a - b) + c` dawały różne postacie. Wprowadzony **kolektor ze znakiem**
>    (3.4).
> 4. Dopisany kontrakt implementacyjny: **zachowanie tożsamości węzła także
>    w świeżej sesji** (5.6), oraz „wszystkie pola niewyrażeniowe zachowane
>    dokładnie" dla `EquationSimplifier` (13).

> **v3, po drugim review.** Dwa blockery i jedno rozstrzygnięcie — wszystkie
> dotyczą tego samego: **v2 nazywała „postacią normalną" coś, co nie było
> jednoznaczne**.
>
> 1. **Redukcja stałych w kolejności wejściowej nie jest deterministyczna.**
>    Zmierzone: `1e16 + x + (-1e16) + y + 1` daje **1**, a to samo z przestawionymi
>    stałymi daje **0**. Stałe są teraz sortowane **przed** redukcją, składane
>    w ustalonym układzie, a obietnica ograniczona do tego, na co pozwala
>    fabryka (3.2).
> 2. **„Normalizacja znaku" nie definiowała jednej postaci.** `-(a*b)`, `(-a)*b`
>    i `a*(-b)` muszą wyjść jako jeden DAG; do tego transformacja znaku stała
>    **po** sortowaniu, więc mogła rozbić własny wynik. Wprowadzona konwencja
>    przez **parzystość** (3.3).
> 3. **`-(-x)` i `Subtract` wracają do kanonizatora**, bo inaczej jeden przebieg
>    nie daje postaci normalnej — z konsekwencją, którą trzeba powiedzieć wprost:
>    **F3.2 dostarcza zero reguł produkcyjnych dla silnika F3.1** (8).
> 4. Rozstrzygnięte: **`EquationSimplifier` powstaje już w F3.2**, cienki (13).
>
> Plus doprecyzowanie kontraktu równoważności wobec zwijania stałych (6.1).

> **v2, po review.** Kierunek zatwierdzony, pięć poprawek — trzy z nich
> powtarzają błędy, które F3.1 już raz naprawiło, i to jest najważniejsza rzecz
> do odnotowania o tej wersji:
>
> 1. **§6 robiło globalne porównanie `divisorsOf`** — ta sama klasa błędu, którą
>    F3.1 zamknęło w v1→v2. `a/(y+x)` → `a/(x+y)` zmienia strażnika
>    **strukturalnie**, nie zmieniając dziedziny, więc proponowany test odrzucałby
>    poprawną kanonizację. Zastąpione warunkiem behawioralnym (6).
> 2. **`-(a*b) → (-a)*b` łamie kontrakt miary** dokładnie tak, jak `x-y → x+(-y)`:
>    4 → 4. Dokument złapał drugi przypadek i **przeoczył pierwszy** (8, 9).
> 3. **Brakowało sesji** — układ równań to wiele korzeni jednego DAG-a, więc
>    kanonizacja bez wspólnego memo rozbija współdzielenie. F3.1 dostało
>    `RewriteSession` z tego samego powodu (5.4).
> 4. **§5.2 i §5.3 opisywały dwa różne porządki** — leksykograficzny i po
>    odcisku. Wybrany strukturalny, hash wycofany (5).
> 5. **Kształt odbudowanego drzewa nie był zdefiniowany** — sama kolejność nie
>    wyznacza struktury (5.5).
>
> Plus dwie nazewnicze: **postac AC-normalna**, nie „kanoniczna" (1.1), oraz
> `F3.5` zamiast `F3.4` przy pattern detektorze.

> **Ten dokument proponuje zmianę zakresu etapu wobec roadmapy.** Powód nie jest
> estetyczny — jest zmierzony. Cała lista reguł, którą roadmapa przypisuje F3.2,
> zadziałałaby na dzisiejszym materiale **zero razy**, bo `ExpressionFactory`
> robi to samo zachłannie, przy budowie.
>
> Proponuję F3.2 jako **kanoniczne uporządkowanie łańcuchów AC** — czyli to,
> co roadmapa wymienia na końcu listy jako „porządkowanie składników", a co
> F3.1 świadomie wypchnęło poza silnik jako mechanizm o innej naturze.
>
> Decyzja o zakresie należy do ciebie; dokument daje liczby, na których można ją
> oprzeć, i mówi wprost, co tracimy, jeśli zostaniemy przy wersji z roadmapy.

---

## Prompt

> potem pisz proposal kolejnej fazy

---

## 0. Stan wyjściowy

F3.1 **zaimplementowane i zielone**:

```text
build/         Release, NDEBUG        450 / 450
build-werror/  -Werror, asserty żywe  450 / 450
```

W repo są `RewriteRule.hpp`, `RewriteEngine.hpp`, `RewriteEngine.cpp` i
`test_rewrite_engine.cpp` — silnik, który potrafi bezpiecznie stosować reguły,
ale **nie ma ani jednej reguły**. F3.2 miało je dostarczyć.

---

## 1. Pomiar, od którego trzeba zacząć

Zanim napisałem choć jedną regułę, sprawdziłem, na czym miałyby pracować.
Materiał: pełny układ równań dla celu `Pose`, oba roboty w repo.

```text
kr640.urdf        12 równań    540 węzłów z krotnościami, 286 unikalnych
  add=109  sub=0  mul=140  div=0  neg=6  sin=6  cos=6

kr4_r600.urdf     12 równań    920 węzłów z krotnościami, 521 unikalnych
  add=188  sub=0  mul=251  div=0  neg=6  sin=6  cos=6
```

Teraz lista z roadmapy, wzorzec po wzorcu, liczona na tym materiale:

```text
x + 0 -> x            0 wystąpień
x - 0 -> x            0
x * 1 -> x            0
x / 1 -> x            0
constant folding      0 par do zwinięcia
-(-x) -> x            0
-(a*b) -> (-a)*b      0
c1*(c2*x)             0
x + x                 0
x - x                 0
```

**Zero. Każdy wzorzec, oba roboty.** Nie „mało" — dokładnie zero.

Sprawdziłem też szerzej, czy cokolwiek klasycznie kanonizującego ma co robić:

```text
kr640      łańcuchy iloczynowe 140, z >1 stałą:  0, najdłuższy: 2 czynniki
           łańcuchy sumowe      44, z >1 stałą:  0, najdłuższy: 11 składników
           sumy z powtórzonym składnikiem strukturalnie: 0

kr4_r600   łańcuchy iloczynowe 251, z >1 stałą:  0, najdłuższy: 2 czynniki
           łańcuchy sumowe      87, z >1 stałą:  1, najdłuższy: 19 składników
           sumy z powtórzonym składnikiem strukturalnie: 0
```

---

## 2. Dlaczego zero — i dlaczego to nie jest wada

Odpowiedź jest w komentarzu klasy `ExpressionFactory`, napisanym w Fazie 1:

> Applies two kinds of local, O(1) normalization while building:
> constant folding, neutral elements.

Fabryka jest **zamknięta nad tymi tożsamościami**. `add(x, 0)` nigdy nie zbuduje
węzła `Add` — zwróci `x`. `multiply(c1, c2)` nigdy nie zbuduje `Multiply` —
zwróci stałą. Wzorzec, który reguła miałaby dopasować, **nie istnieje w chwili,
gdy reguła mogłaby go zobaczyć**.

To była dobra decyzja i nie proponuję jej cofać. Ma jednak konsekwencję, której
roadmapa nie mogła przewidzieć, bo powstawała przed Fazą 1: **F3.2 w brzmieniu
z roadmapy jest w dużej mierze pusty**.

Uczciwie o dwóch zastrzeżeniach do tego wniosku:

**„Reguły przydadzą się później, gdy F3.3 zacznie podstawiać."** Prawda — i to
jest jedyny argument za napisaniem ich mimo wszystko. Ale wtedy nazwijmy rzecz
po imieniu: to nie jest etap upraszczania równań, tylko **przygotowanie
biblioteki reguł na materiał, którego jeszcze nie ma**, testowany wyłącznie
własnymi testami jednostkowymi. Etap wart może dnia, nie dziesięciu.

**„A jeśli reguła F3.3 zbuduje `x + 0`?"** Nie zbuduje, bo też chodzi przez
fabrykę. Jedyne, co może zbudować, to wzorce **złożone**, których fabryka nie
widzi lokalnie — a te wymagają reguł, których na liście z roadmapy nie ma.

---

## 3. Co naprawdę zostało do zrobienia

Popatrzmy na to, co równania faktycznie zawierają:

```text
suma 19 składników, każdy iloczynem dwóch czynników zbudowanych z sin/cos
kolejność składników: taka, w jakiej złożyła je kompozycja FK
```

Ta kolejność jest **przypadkiem konstrukcyjnym**. `T1*T2*T3...` mnoży macierze
w kolejności łańcucha, a suma w komórce wychodzi w kolejności pętli po indeksie.
Dwa algebraicznie identyczne wyrażenia zbudowane inną drogą dadzą inną kolejność
i **nie będą `structurallyEqual`**.

To blokuje wszystko, co przychodzi później:

```text
F3.3  tożsamości trygonometryczne   musi ROZPOZNAĆ sin^2+cos^2 w sumie 19 składników
F3.5  IkPatternDetector             musi dopasować kształt równania do wzorca
F3.8  solver pozycji 6R             musi porównać strukturę z katalogiem przypadków
```

(v1 pisała tu `F3.4` — to jest **geometryczny model osi i jointów**; detektor
wzorców to `F3.5`.)

Żaden z nich nie działa na materiale, w którym to samo wyrażenie ma wiele
zapisów. **Postać AC-normalna jest przesłanką tamtych etapów, a nie ozdobą
tego.**

Proponowany zakres F3.2 — **jeden przebieg, który kończy robotę**:

```text
canonicalize(E), dla węzła złożonego:

  1. ZBIERZ łańcuch AC kolektorem ze znakiem                  <-- 3.4
        schodzi przez Add / Subtract / Negate
        każdy atom kanonizuje rekurencyjnie
        jeśli kanonizacja atomu ODSŁONI operator AC -> splice
  2. dla iloczynu i dzielenia: zdejmij znaki, policz PARZYSTOŚĆ  <-- 3.3
        źródła znaku: Negate ORAZ Constant(c < 0)
  3. rozdziel na STAŁE i pozostałe
  4. posortuj stałe porządkiem z 5.2                          <-- najpierw kolejność
  5. redukuj stałe OD LEWEJ z akumulatorem, tylko połączenia DOKŁADNE
        dodawanie: 2Sum;  mnożenie: test na reprezentacji     <-- 3.2.1
        bez szukania partnera dla nieprzyłączonej stałej      <-- 3.2.2
  6. odrzuć stałą neutralną, jeśli taka wyszła
  7. posortuj pozostałe składniki porządkiem z 5.2
  8. odbuduj: balanced(niestałe), stałe doklejone NA ZEWNĄTRZ   <-- 5.5, 3.2.3
        żadne wywołanie add/multiply nie dostaje dwóch stałych
  9. parzystość nieparzysta: canonicalNegate(wynik)            <-- 3.3
        NIE „jeden Negate na zewnątrz": dla sumy znak wchodzi w składniki
 10. jeśli nic się nie zmieniło: zwróć ORYGINALNY Expression  <-- 5.6
```

Zestaw reguł dla silnika F3.1: **pusty** — uzasadnienie w 8.

### 3.1 Nazwa: **AC-normalna**, nie „kanoniczna"

Poprawka z review i ważniejsza, niż wygląda. Ten etap **nie** daje własności:

```text
E1 równe algebraicznie E2   =>   normalize(E1) structurallyEqual normalize(E2)
```

bo świadomie nie robi rozdzielności, zbierania wyrazów podobnych ani
trygonometrii (10). Po tym etapie `a*(b+c)` i `a*b + a*c` **nadal będą różne**.

Kontrakt nazywa się więc:

> **postać AC-normalna** — jednoznaczna względem łączności i przemienności
> `Add`/`Multiply`, redukcji stałych oraz normalizacji odejmowania i znaku;
> **względem niczego innego**.

Musi to stać w komentarzu klasy, bo `F3.5` jest o krok od założenia
`structurallyEqual == algebraically equal`, a to założenie będzie fałszywe jeszcze
przez kilka etapów — i będzie fałszywe **cicho**.

### 3.2 Stałe: kolejność, a potem **wyłącznie dokładna** redukcja

Ta sekcja była poprawiana dwa razy i za drugim razem okazało się, że sam pomysł
był za słaby, nie tylko jego kolejność.

**Runda pierwsza (v3):** redukcja w kolejności wejściowej nie jest
deterministyczna, bo dodawanie `double` nie jest łączne. Zmierzone:

```text
E1:  1e16 + x + (-1e16) + y + 1     ->  (1e16 + -1e16) + 1  =  1
E2:  1e16 + x + 1       + y + (-1e16) ->  (1e16 + 1) + -1e16  =  0
```

Stąd: **stałe sortowane przed redukcją**, składane w ustalonym układzie. To
zostaje.

**Runda druga (v4 → v5): to nie wystarczy, bo redukcja łamie gwarancję z 6.**
Review pokazało, że ustalona kolejność czyni wynik **powtarzalnym**, ale nadal
**innym wyrażeniem**:

```text
d = q + 1e16 + 1 + (-1e16)          algebraicznie:  q + 1     zeroSet = {-1}

stałe posortowane:  -1e16, 1, 1e16
redukcja:           (-1e16) + (1 + 1e16) = (-1e16) + 1e16 = 0      <-- zmierzone
stała neutralna odrzucona

d' = q                              zeroSet = {0}
```

To **nie jest** już różnica w evaluatorze `double`, którą 6.1 dopuszcza. Nowy
liść `Constant(0)` staje się częścią **drzewa symbolicznego**, które potem
interpretujemy nad `R` — więc zmieniła się funkcja, a nie tylko jej numeryczne
przybliżenie. Representability guard tego nie złapie, bo mianownikiem jest `q`,
a nie literalne zero.

Gwarancja „dziedzina symboliczna zachowana **dokładnie**" i „zwijamy rozproszone
stałe przez `double`" **nie mogą obowiązywać naraz**. Jedna z nich musi ustąpić.

**Rozstrzygnięcie: ustępuje zwijanie, nie gwarancja.**

```text
stałe są ŁĄCZONE TYLKO WTEDY, GDY WYNIK JEST DOKŁADNY
```

### 3.2.1 Kryterium dokładności — i dlaczego dodawanie jest łatwiejsze niż mnożenie

**Dodawanie: 2Sum, bez zastrzeżeń.**

```cpp
// Exact rounding error of a + b. err == 0  <=>  the sum was exact.
//
// Sound for every finite pair, subnormals included: addition whose result is
// subnormal is exact under gradual underflow, so there is no hidden case here.
// Overflow is the only breakage and ExpressionFactory declines to fold a
// non-finite result anyway.
double addError(double a, double b)
{
    const double s = a + b;
    const double bb = s - a;
    return (a - (s - bb)) + (b - bb);
}
```

**Mnożenie: `fma` nie wystarczy, i to na dwa sposoby.** Pierwszy złapało review
przy v5, drugi przy v6 — obydwa zmierzyłem, zanim je zapisałem.

```text
(1)  underflow do zera             a = denorm_min, b = 0,5
     p = 0,  fma = 0               „dokładne"    -- NIEPRAWDA
     isnormal(p) = fałsz            <-- złapane przez strażnika z v6

(2)  underflow SAMEJ RESZTY        a = b = nextafter(2^-511, +inf)
     p normalny, fma = 0           „dokładne"    -- NIEPRAWDA
     isnormal(p) = PRAWDA           <-- strażnik z v6 tego NIE łapie
```

Przypadek (2) jest tym, przez który `fma` nie nadaje się na **dowód**. Dla
`a = (1 + 2^-52) · 2^-511`:

```text
a·a = (1 + 2^-51 + 2^-104) · 2^-1022
błąd zaokrąglenia = 2^-1126
najmniejsza liczba subnormalna = 2^-1074
=> reszta też wpada w underflow i wychodzi z fma jako 0
```

Czyli reszta z `fma` jest wiarygodna tylko wtedy, gdy **ona sama** jest
reprezentowalna — a tego nie widać po `p`.

**Kryterium musi być sound *i* complete** — i to jest rozstrzygnięcie z v8,
które przewraca podejście z v7. v7 przyjęła kryterium świadomie zachowawcze,
uzasadniając to tym, że „nieodrzucone zwinięcie kosztuje jeden węzeł". Po
napisaniu 3.2.4 **to zdanie przestało być prawdziwe**, czego nie zauważyłem:

```text
A = (1e200 * 3) * x       fabryka widzi dwie stałe -> zwija do  3e200 * x
B = 1e200 * (3 * x)       stałe rozdzielone symbolem

1e200 * 3 JEST dokładnie reprezentowalne.

kryterium zachowawcze z v7 odrzuca tę parę, więc:
    normalize(A) = 3e200 * x
    normalize(B) = 1e200 * (3 * x)        DWIE POSTACIE NORMALNE
```

Przyczyna jest ta sama, co w 3.2.4, tylko od drugiej strony: skoro fabryka zwija
**zachłannie i dokładnie**, to każdy **fałszywy negatyw** predykatu psuje postać
normalną dokładnie tak samo, jak fałszywy pozytyw psuł dziedzinę. Kontrakt jest
więc dwustronny:

> `multipliesExactly(a, b)` jest prawdą **wtedy i tylko wtedy**, gdy matematyczny
> iloczyn `a·b` jest dokładnie reprezentowalnym skończonym `binary64`.

Drugi przypadek, który to samo psuł, a którego v7 też nie widziała: strażnik
`!isnormal(p) -> false` odrzucał iloczyny o wyniku **subnormalnym, ale dokładnym**
— na przykład `2^-1000 · 2^-70 = 2^-1070`.

**Pełna analiza bitowa**, bez `__int128`:

```cpp
// True exactly when the mathematical product of a and b is a representable
// finite binary64 -- sound AND complete, because a false negative breaks the
// normal form just as a false positive breaks the domain (3.2.4).
//
// frexp gives |f| in [0.5, 1), so ldexp(|f|, 53) is exactly a 53-bit integer.
// Their product needs at most 106 bits, assembled here from 32-bit limbs
// rather than __int128, which keeps the MSVC branch of the build viable.
bool multipliesExactly(double a, double b)
{
    const double product = a * b;

    if (!std::isfinite(product))
        return false;                       // overflow; the factory declines anyway
    if (a == 0.0 || b == 0.0)
        return true;

    int exponentA = 0;
    int exponentB = 0;
    const double fractionA = std::frexp(std::fabs(a), &exponentA);
    const double fractionB = std::frexp(std::fabs(b), &exponentB);

    const auto significandA = static_cast<std::uint64_t>(std::ldexp(fractionA, 53));
    const auto significandB = static_cast<std::uint64_t>(std::ldexp(fractionB, 53));

    // Exact 106-bit product as (high, low).
    const std::uint64_t lowA = significandA & 0xFFFFFFFFull, highA = significandA >> 32;
    const std::uint64_t lowB = significandB & 0xFFFFFFFFull, highB = significandB >> 32;
    const std::uint64_t middle = lowA * highB + highA * lowB;   // < 2^54, cannot overflow
    std::uint64_t high = highA * highB + (middle >> 32);
    const std::uint64_t middleLow = middle << 32;
    std::uint64_t low = lowA * lowB + middleLow;
    if (low < middleLow)
        ++high;

    const int trailingZeros = low ? std::countr_zero(low) : 64 + std::countr_zero(high);
    const int bitWidth      = high ? 64 + std::bit_width(high) : std::bit_width(low);

    const int significantBits = bitWidth - trailingZeros;
    const int lowestExponent  = exponentA + exponentB - 106 + trailingZeros;

    if (significantBits > 53)                    return false;   // needs more than 53 bits
    if (lowestExponent < -1074)                  return false;   // bits below the smallest subnormal
    if (lowestExponent + significantBits - 1 > 1023) return false;
    return true;
}
```

**Zweryfikowane wobec arytmetyki dokładnej**, bo w tej sekcji pomyliłem się dwa
razy i nie chciałem polegać na rozumowaniu. Predykat porównany z wyrocznia
liczoną na ułamkach wymiernych, bez zaokrągleń:

```text
15 przypadków brzegowych z tego dokumentu   ->  15/15 zgodnych
8000 par losowych, w tym subnormalne i skrajne wykładniki
      4523 dokładnych według wyroczni       ->  0 rozjazdów
```

Kontrolnie, wiersze, które w v7 były błędne:

```text
nextafter(2^-511)^2      dokładny? NIE   predykat: NIE      (v7: błędnie TAK)
denorm_min * 0,5         dokładny? NIE   predykat: NIE
1e200 * 3                dokładny? TAK   predykat: TAK      (v7: błędnie NIE)
2^-1000 * 2^-70          dokładny? TAK   predykat: TAK      (v7: błędnie NIE)
```

**Dodawanie zostaje na 2Sum**, i to nie jest niekonsekwencja: 2Sum jest
**sound i complete** dla każdej pary skończonych `double` bez przepełnienia —
zwraca dokładny błąd zaokrąglenia, więc `err == 0` zachodzi dokładnie wtedy, gdy
suma była dokładna. Nie ma tu odpowiednika problemu z 3.2.1, bo suma o wyniku
subnormalnym jest **dokładna** przy stopniowym niedomiarze, więc reszta nie ma
jak wpaść w underflow. Psuje ją wyłącznie przepełnienie, a tam fabryka i tak
odmawia zwinięcia.

Asymetria między dodawaniem a mnożeniem jest więc w **narzędziu**, nie
w kontrakcie: oba predykaty muszą być dwustronne, tylko dla sumy wystarczy
sześć działań, a dla iloczynu trzeba sięgnąć do reprezentacji.

### 3.2.2 Harmonogram redukcji — też musi być częścią kontraktu

Drugi blocker z review i ten sam rodzaj przeoczenia, co przy kształcie drzewa
w 5.5: „redukuj sąsiednie stałe, gdy dokładnie" **nie wyznacza jednego wyniku**,
bo nie mówi, **które** sąsiednie i w jakiej kolejności. Zmierzone na
posortowanych `1, 2, 2^53`:

```text
wariant A:   (1 + 2) = 3        dokładne
             3 + 2^53           NIEdokładne     ->  reszty  [3, 2^53]

wariant B:   (2 + 2^53)         dokładne, bo ulp przy 2^53 wynosi 2
             1 + (2^53 + 2)     NIEdokładne     ->  reszty  [1, 2^53+2]
```

Oba używają **wyłącznie połączeń dokładnych**, oba zachowują wartość
algebraiczną, oba startują z tego samego posortowanego wejścia — i dają **różne
postacie**. Bez ustalonego harmonogramu teza etapu nie zachodzi.

**Rozstrzygnięcie: przejście od lewej z akumulatorem, bez szukania partnera —
powtarzane aż do punktu stałego.**

```text
powtarzaj:
    posortuj stałe porządkiem z 5.2
    acc := c[0]
    dla kolejnych c[i]:
        jeśli połączenie acc z c[i] jest DOKŁADNE:  acc := acc (op) c[i]
        w przeciwnym razie:                          zapisz acc;  acc := c[i]
    zapisz acc
dopóki cokolwiek się połączyło
```

Zewnętrzna pętla to poprawka z v7 i jest **konieczna**, bo jedno przejście jest
deterministyczne, ale **nie idempotentne**. Zmierzone:

```text
wejście:  2^52,  2^52+1,  2^52+3        (wszystkie dokładnie reprezentowalne)

przebieg 1:  2^52 + (2^52+1) = 2^53+1   NIEdokładne  -> flush 2^52
             (2^52+1) + (2^52+3)        dokładne     -> 2^53+4
             wynik:  [2^52,  2^53+4]

przebieg 2:  2^52 + (2^53+4)            DOKŁADNE     -> jedna stała
```

Bez powtarzania `normalize(normalize(E))` różniłoby się od `normalize(E)`, czyli
poległby `IsIdempotent` — test pilnujący całej tezy etapu.

**Terminacja jest trywialna:** każde udane połączenie zmniejsza liczbę stałych
o jeden, więc pętla wykona najwyżej `n-1` obrotów. Reducer ma więc własny, mały
punkt stały, dowodzony bez odwołania do miary z F3.1 i bez mieszania go z
silnikiem przepisywania.

Dla przykładu z `1, 2, 2^53` daje to jednoznacznie wariant A. **Nie szukamy innego
partnera** dla stałej, która się nie złączyła — i to jest część kontraktu, nie
uproszczenie implementacji: „spróbujmy złożyć co się da" jest algorytmem
zachłannym o wyniku zależnym od kolejności prób, czyli dokładnie tym, czego ten
etap ma nie mieć.

Wybór „od lewej" zamiast harmonogramu zbalansowanego wymaga jednego zdania
uzasadnienia, bo 5.5 wybrało zbalansowany kształt **drzewa**: te dwie decyzje
dotyczą różnych rzeczy. Zbalansowane sumowanie wybrano tam dla **akumulacji
błędu** — a tutaj błędu **nie ma żadnego**, bo każde połączenie jest albo dokładne,
albo odrzucone. Zostaje więc kryterium prostoty i dowodliwości, a harmonogram
zbalansowany z odmówionymi węzłami nie ma naturalnej definicji: nie wiadomo, co
zrobić z parą, która się nie złączyła, gdy jej rodzic ma już trzy elementy po
jednej stronie.

### 3.2.3 Pułapka: sama rezygnacja ze zwijania **nie wystarczy**

Rzecz, której nie widać z opisu algorytmu: gdy zostają dwie stałe, których nie
wolno połączyć, **zrobi to za nas fabryka**. `ExpressionFactory::add(c1, c2)` zwija
bezwarunkowo (poza nieskończonością), więc wystarczy, że zbalansowana odbudowa
postawi je obok siebie — a posortowane stałe są obok siebie z definicji.

**Poprawka kształtu**: stałe nie wchodzą do zbalansowanego drzewa, tylko są
doklejane **na zewnątrz**, pojedynczo:

```text
wynik =  c1 + ( c2 + ( ... + balanced(składniki niestałe) ) )
```

Każde wywołanie `add` ma wtedy co najwyżej **jeden** operand będący stałą, więc
fabryka nie ma czego zwinąć. Kształt pozostaje deterministyczny, a koszt
głębokości żaden: liczba niepołączalnych stałych to w praktyce **zero albo jedna**
— na materiale repo zmierzono jeden łańcuch sumowy z dwiema stałymi w całym
`kr4_r600` (1), i one się łączą dokładnie.

Ten sam kształt obowiązuje dla iloczynu, z `multiply`.

**Granica obietnicy**, teraz spójna z 6:

> Stałe łańcucha są redukowane do jednej **wtedy i tylko wtedy, gdy każde
> połączenie jest dokładne**. Stałe, których nie można połączyć bez zaokrąglenia,
> zostają osobnymi składnikami w kolejności AC-normalnej. **Kanonizator nie
> wprowadza żadnego nowego zaokrąglenia.**

Ostatnie zdanie jest tym, co czyni dowód z 6.3 prawdziwym bez zastrzeżeń —
a wcześniejsze wersje miały w tym miejscu wyjątek, który unieważniał całą tezę.

### 3.2.4 Czy zwijanie stałych jest w ogóle warte tej złożoności?

Pytanie z review, postawione jako ważniejsze od trzech blockerów — i słusznie,
bo zwijanie zjada większość tego rozdziału: 2Sum, test bitowy, harmonogram,
powtarzanie do punktu stałego, ochrona reszt przed fabryką. Pięć mechanizmów po
to, żeby — według pomiaru z 1 — zwinąć stałe w **jednym łańcuchu sumowym w całym
korpusie dwóch robotów**.

Sprawdziłem więc wariant „bez zwijania międzyłańcuchowego": fabryka zachowuje
swoje lokalne zwijanie, kanonizator tylko sortuje rozproszone stałe i nie próbuje
ich łączyć. **Ten wariant przewraca tezę etapu**, i to na najprostszym możliwym
przykładzie:

```text
wejście A:   (3 + 4) + x        fabryka zwija przy budowie  ->  7 + x
wejście B:    3 + x  + 4        stałe rozdzielone symbolem   ->  Add(Add(3,x),4)

A i B są AC-równoważne.

bez zwijania międzyłańcuchowego:
    normalize(A) = 7 + x
    normalize(B) = 3 + (4 + x)          DWIE RÓŻNE POSTACIE NORMALNE
```

Powodem jest to, że **fabryka i tak zwija**, tylko lokalnie i zależnie od tego, co
akurat stało obok siebie w chwili budowy. Kanonizator, który nie zwija, nie jest
więc „bardziej zachowawczy" — jest **niespójny z materiałem, który dostaje**.
Żeby dwa AC-równoważne wejścia dały jedną postać, kanonizator musi dokończyć
to, co fabryka zaczęła przypadkowo.

**Odpowiedź: zwijanie jest konieczne** — nie dla oszczędności węzłów, tylko dla
samej własności postaci normalnej. Złożoność jest ceną tej własności, a nie
optymalizacją, którą można wyciąć.

Co **można** wyciąć, gdyby ten rozdział okazał się w praktyce kłopotliwy: zwijanie
**iloczynów**. Łańcuchy iloczynowe mają w korpusie zero przypadków z więcej niż
jedną stałą (1), a ten sam kontrprzykład dla mnożenia — `2 * x * 3` wobec
`(2*3) * x` — wymagałby świadomej rezygnacji z postaci normalnej dla iloczynów.
**Nie rekomenduję tego**, ale odnotowuję, bo to jedyne miejsce, w którym da się
kupić prostotę za jasno nazwaną cenę, i lepiej mieć to policzone niż odkrywać
przy implementacji.

### 3.3 Znak: konwencja przez **parzystość**, nie przez wybór czynnika

Drugi blocker. v2 zostawiała w kanonizatorze transformację
`Negate(Multiply(a,b)) → Multiply(Negate(a), b)` i nazywała to „normalizacją
znaku". To nie jest normalizacja — to **przesunięcie problemu**:

```text
-(a*b)      (-a)*b      a*(-b)        trzy zapisy jednej rzeczy
```

Reguła v2 sprowadza pierwszy do drugiego i **nie dotyka trzeciego**. Gorzej:
`Negate(a)` ma inną rangę niż `a` (5.2), więc wynik nie jest już posortowany —
a transformacja stała **po** sortowaniu, czyli jeden przebieg mógł zostawić
postać wymagającą kolejnego. `IsIdempotent` z 11 poległby na tym.

**Poprawka: znak wychodzi z iloczynu w całości, przez parzystość** — i liczy się
z **dwóch** źródeł, nie z jednego:

```text
dla każdego czynnika łańcucha:
    Negate(x)                 ->  parity ^= 1,  czynnik := x
    Constant(c), c < 0        ->  parity ^= 1,  czynnik := Constant(-c)
posortuj czynniki JUŻ BEZ znaków
zredukuj stałe (3.2), odrzuć neutralną 1
odbuduj iloczyn zbalansowany
parity parzyste    -> zostaw
parity nieparzyste -> canonicalNegate(wynik)     <-- NIE factory.negate
```

**Ostatni wiersz jest poprawką z v7** i bez niego konwencja łamie własny
postcondition z 3.4. Kontrprzykład z review:

```text
(-1) * (a + b)

   stała -1  ->  parity ^= 1,  czynnik := 1
   1 jest neutralne  ->  wypada
   zostaje jeden czynnik:  a + b
   odbudowa iloczynu z jednym czynnikiem  ->  a + b

   parity nieparzyste  ->  factory.negate(a + b)  =  Negate(Add(...))
                                                     ^^^ ZAKAZANE przez 3.4
```

A ponieważ `canonicalize(-(a+b))` daje `(-a) + (-b)`, dostajemy
`canonicalize(canonicalize(E)) ≠ canonicalize(E)` — znowu złamana idempotencja,
tym razem inną drogą niż w 3.2.2.

`canonicalNegate` rozwiązuje to **z definicji**, bo nadaje znak zgodnie
z konwencją zamiast owijać węzeł: sumie wpuszcza znak do składników przez kolektor
z 3.4, iloczynowi i ilorazowi przełącza parzystość, stałej zmienia wartość,
`Negate(x)` sprowadza do `x`. To ta sama funkcja, której kolektor używa do
nadawania biegunowości — więc nie dochodzi nowy mechanizm, tylko przestaje być
omijany w jednym miejscu.

Ta sama poprawka obowiązuje dla ilorazu: jeśli po odrzuceniu neutralnego
mianownika `x / 1` wynik zapadnie się do sumy, znak musi wejść przez
`canonicalNegate`, a nie owinąć ją w `Negate`.

Drugi wiersz jest poprawką z review i bez niego konwencja **nie działa**:
fabryka buduje `(-2) * x` jako `Multiply(Constant{-2}, x)`, a nie
`Multiply(Negate(Constant{2}), x)`, więc parzystość liczona wyłącznie po węzłach
`Negate` w ogóle go nie widzi:

```text
(-2) * x     ->  bez poprawki zostaje (-2)*x
2 * (-x)     ->  -(2*x)
                 dwie postacie jednego wyrażenia — dokładnie to, czego etap ma nie robić
```

Z poprawką obie dają `-(2*x)`. Przypadek skrajny wychodzi przy okazji i też jest
wart testu:

```text
(-1) * x   ->  parity nieparzyste, stała 1 odrzucona jako neutralna  ->  -x
```

Jedna subtelność do zapisania w kodzie: warunek to `c < 0`, więc **`-0.0` nie
przełącza parzystości** — `-0.0 < 0.0` jest fałszem w IEEE-754. To jest zachowanie
pożądane: `-0.0` i `0.0` są tą samą wartością, a `isZero` w tym projekcie i tak
traktuje je jednakowo.

#### Zakres konwencji znaku — powiedziany wprost

Review słusznie zauważyło, że kontrakt musi mówić, **dokąd** sięga normalizacja
znaku, inaczej `F3.5` założy globalną jednoznaczność i pomyli się przy pierwszym
dzieleniu.

```text
OBJĘTE:   biegunowość Add / Subtract           (3.4)
          parzystość Multiply, z ujemnymi stałymi
          parzystość Divide                      <-- rozszerzenie w v5
```

Review dopuszczało zostawienie `Divide` poza kontraktem, byle to napisać.
**Włączam je**, bo kosztuje tę samą maszynerię, a alternatywa zostawia
`F3.5` ze zdaniem „znak jest jednoznaczny, **z wyjątkiem** dzielenia" — czyli
z wyjątkiem, który trzeba pamiętać:

```text
-(a/b)   ->  -(a/b)
(-a)/b   ->  -(a/b)
a/(-b)   ->  -(a/b)         jeden DAG
(-a)/(-b) ->  a/b           parzystość parzysta
```

Bezpieczeństwo: `zeroSet(-b) == zeroSet(b)`, więc dziedzina jest zachowana
dokładnie (6.3), a negacja jest **dokładna zmiennoprzecinkowo**, więc nie dotyka
wyjątku z 3.2. Na materiale repo to i tak zero trafień — `div=0` w obu układach
równań — więc koszt jest żaden, a luka w kontrakcie znika.

```text
POZA KONTRAKTEM tego etapu, świadomie:
    rozdzielność znaku przez sumę w mianowniku:  a/(-(b+c))  vs  -(a/(b+c))
        -- wymagałoby wnoszenia znaku pod Add, czyli rozdzielności (10)
    znak wewnątrz sin/cos:  sin(-x) = -sin(x)
        -- to tożsamość trygonometryczna, czyli F3.3
```

Wynik:

```text
-(a*b)  ->  -(a*b)
(-a)*b  ->  -(a*b)
a*(-b)  ->  -(a*b)          jeden DAG, niezależnie od zapisu wejściowego
(-a)*(-b) ->  a*b           parzystość załatwia to sama, bez osobnej reguły
```

Sortowanie **bez znaków** jest tu istotne: gdyby minus został przy czynniku,
kolejność zależałaby od tego, który czynnik go akurat niósł na wejściu — czyli
znowu od przypadku konstrukcyjnego.

### 3.4 Kolektor: ze znakiem **i** ze spłaszczaniem tego, co odsłoni

v3 wprowadziła kolektor ze znakiem dla `Subtract`, v5 domyka go w dwóch
miejscach, które review nazwało precyzyjnie.

**Problem A — kanonizacja dziecka odsłania nowy operator AC.** Kolektor pyta
o kształt **przed** kanonizacją składnika, więc:

```text
E = a + -(-(b + c))

zebrane składniki:   a,  -(-(b+c))         drugi NIE jest Add, więc jest atomem
kanonizacja atomu:   -(-(b+c))  ->  b + c   ...i to jest Add, ale lista już zamknięta

wynik:  a + (b + c)          zamiast płaskiego  [a, b, c]
```

Skutek jest dokładnie taki, jak wskazało review: **`normalize(normalize(E))`
różni się od `normalize(E)`**, więc `IsIdempotent` — test, który ma pilnować
całej tezy etapu — by poległ.

**Problem B — `Negate(Add(...))`.** Kolektor schodził przez `Subtract`, ale nie
przez `Negate`, więc:

```text
-(a + b)        zostawało atomem
0 - (a + b)     schodziło przez Subtract  ->  -a, -b
```

Znowu dwie postacie jednego wyrażenia.

**Poprawka — jeden kolektor, trzy przypadki zejścia i splice:**

```text
collectAdd(expr, sign, out):

    Add(a, b)       ->  collectAdd(a,  sign, out);  collectAdd(b,  sign, out)
    Subtract(a, b)  ->  collectAdd(a,  sign, out);  collectAdd(b, -sign, out)
    Negate(x)       ->  collectAdd(x, -sign, out)              <-- problem B

    w przeciwnym razie:
        term = canonicalize(expr)
        jeśli korzeń term to Add | Subtract | Negate:
            collectAdd(term, sign, out)                        <-- problem A: SPLICE
        w przeciwnym razie:
            out.push( sign < 0 ? canonicalNegate(term) : term )
```

Analogicznie dla iloczynu, z `Negate` przechodzącym w parzystość zamiast w znak
składnika (3.3).

**Dlaczego splice się kończy**, bo to jedyne miejsce, gdzie można zrobić pętlę:
splice woła `collectAdd` na wyrażeniu **już kanonicznym**, którego korzeń należy
do zbioru zejścia — więc trafia w jedną z trzech pierwszych gałęzi i schodzi
**strukturalnie niżej**. Jego dzieci są kanoniczne, więc `canonicalize` zwraca je
bez zmian (5.6, tożsamość węzła) i trafiają do `push`. Bez wymogu z 5.6 ta
rekurencja **nie miałaby dowodu terminacji** — te dwie decyzje trzymają się
nawzajem.

W dzisiejszym języku wyrażeń splice jest zresztą osiągalny **wyłącznie** przez
`Negate`, bo żaden inny rodzaj węzła nie kanonizuje się do `Add` — nie ma
rozdzielności (10). Zostaje mimo to zapisany ogólnie, bo F3.3 doda przekształcenia,
które to zmienią, i lepiej, żeby kontrakt był gotowy niż łatany.

**Postcondition**, mocniejszy niż w v3 i wart wypowiedzenia osobno:

> Wyjście AC-normalne **nie zawiera węzła `Subtract`**, a `Negate` występuje
> wyłącznie jako:
>
> - znak pojedynczego składnika sumy,
> - jeden zewnętrzny znak iloczynu,
> - jeden zewnętrzny znak ilorazu   *(doszło z rozszerzeniem konwencji na
>   `Divide`, 3.3)*,
>
> i **nigdy bezpośrednio nad sumą**.

`F3.3` i `F3.5` dostają dzięki temu język węższy o jeden rodzaj węzła i o całą
klasę wariantów znaku. Zamiana `a - b` na `a + (-b)` jest przy tym **dokładna
bitowo**, więc nie dokłada się do dryfu z 7 ani do wyjątku z 3.2.

---

## 4. Dlaczego to **nie** może być regułą silnika F3.1

F3.1 §7.3 przewidziało ten moment i odmówiło:

> Reguły porządkujące — poza tym silnikiem.

Powód jest twardy, nie stylistyczny. Kontrakt reguły wymaga **ściśle malejącej
miary**:

```text
measure(a + b)  =  measure(b + a)
```

Przestawienie **nie zmienia liczby węzłów**. Reguła porządkująca łamie więc
kontrakt, `assert` w buildzie diagnostycznym zatrzyma proces, a jeśli ktoś ten
`assert` usunie — silnik zapętli się na `a+b → b+a → a+b`, dokładnie tak, jak
przewiduje 4.2 architektury F3.1.

Wniosek: **kanonizacja AC to osobny przebieg**, nie wpis w tablicy reguł.

Kolejność względem silnika była w v2 opisana jako:

```text
normalize(E)  ->  rewrite(E)  ->  normalize(E)
```

i review słusznie to podważyło. Skoro kanonizator sam normalizuje odejmowanie
i znak (3.3, 3.4), to **drugi przebieg kanonizujący może odtworzyć redex**, który
przed chwilą usunęła reguła — `a - (-b)` przechodzi w `a + (-(-b))`, więc
kanonizacja **produkuje** dokładnie ten wzorzec, który `-(-x) → x` miało usuwać.
Wynik nie byłby punktem stałym względem reguł, a jedynym wyjściem stałoby się
przeplatanie:

```text
normalize -> rewrite -> normalize -> rewrite -> ...
```

**Odrzucone, i to zdecydowanie.** Terminacja takiej pętli nie wynika z miary
żadnego z mechanizmów z osobna i wymagałaby własnego dowodu — tego rodzaju
konstrukcji F3.1 uniknęło świadomie i nie ma powodu wprowadzać jej tutaj.

Wyjście wychodzi samo z 3.3: **`-(-x)` należy do kanonizatora**, jako
konsekwencja parzystości znaku, a nie jako osobna reguła. Wówczas:

```text
F3.2:    normalize(E)                    jeden przebieg, pełna postać AC-normalna
F3.3+:   normalize(E) -> rewrite(E)      gdy pojawią się reguły trygonometryczne
```

i nie ma drugiego mechanizmu, który mógłby zaburzyć wynik pierwszego. Sesje są
tworzone **raz na układ równań**, nie raz na wyrażenie — z powodu opisanego
w 5.4.

Co z F3.3, gdy reguły **będą** psuć postać normalną? To pytanie tamtego etapu
i tam trzeba je rozstrzygnąć uczciwie — albo reguły będą pisane tak, żeby
zachowywać postać normalną, albo przeplatanie dostanie własny dowód terminacji.
Zapisuję to jako **znane wejście do F3.3**, żeby nie było odkryciem.

---

## 5. Porządek, kształt i sesja

Wymagania na porządek, wszystkie konieczne:

```text
totalny          dowolne dwa wyrażenia dają się porównać
deterministyczny ten sam wynik przy każdym uruchomieniu i na każdej platformie
strukturalny     zgodny ze structurallyEqual: equal <=> ani mniejsze, ani większe
```

v1 miała tu czwarte — „tani" — i to właśnie ono rozjechało dokument.

### 5.1 Pułapka, którą trzeba nazwać

**Porządek nie może dotykać adresów węzłów.** Kuszące, bo `Expression` to
`shared_ptr` i `<` na wskaźnikach jest darmowe — i całkowicie błędne: kolejność
zależałaby od alokatora, czyli od uruchomienia. Te same równania dałyby dwie
różne postacie w dwóch procesach.

To ta sama klasa błędu, którą F3.1 zamknęło kontraktem determinizmu (v3, 1.3) —
i oba etapy powinny odwoływać się do tej samej zasady.

### 5.2 Porządek: strukturalny leksykograficzny

```text
1. RANGA RODZAJU węzła, ustalona jawnie:
       Constant < Symbol < Sin < Cos < Negate < Multiply < Divide < Add < Subtract

2. przy równej randze — porównanie właściwe dla rodzaju:
       Constant     po wartości  (<, dokładnie)
       Symbol       po nazwie    (leksykograficznie)
       jednoargumentowe   rekurencyjnie po operandzie
       dwuargumentowe     rekurencyjnie: lhs, potem rhs
```

Stałe **przed** symbolami, żeby po uporządkowaniu stały obok siebie — choć po
poprawce z 3 redukcja stałych i tak nie zależy już od ich sąsiedztwa.

### 5.3 Odcisk (hash) — **wycofany z tego etapu**

v1 proponowała obok tego porządku klucz sortowania:

```text
key(Symbol s)  = (1, hash(s))
key(op(a,...)) = (rangaRodzaju, mieszanie(key dzieci))
porównanie po kluczu, fallback strukturalny przy kolizji
```

**To był inny porządek, nie przyspieszenie tego samego** — i review złapało to
precyzyjnie. Jeśli `hash("z") < hash("a")`, klucz mówi `z < a`, a 5.2 mówi
`a < z`. Fallback nic nie ratuje, bo włącza się **tylko przy równych kluczach**,
a tu klucze są różne. Oba porządki mogą dać poprawną postać normalną — ale
trzeba wybrać jeden i opisać ten wybrany.

Drugi powód wycofania jest twardszy: `std::hash<std::string>` **nie ma
kontraktu stabilności** między uruchomieniami ani platformami. Budowanie na nim
wymagania „deterministyczny na każdej platformie" byłoby opieraniem gwarancji
na czymś, czego standard nie obiecuje — dokładnie ten błąd, który 5.1 zakazuje
w innym przebraniu.

**Decyzja: prosty, dokładny komparator strukturalny z 5.2.** Materiał to
najwyżej 19 składników; wynik kanonizacji jest memoizowany po węźle (5.4), więc
każdy unikalny węzeł sortuje się raz. Odcisk wraca do rozmowy dopiero, gdy pomiar
pokaże, że boli — i wtedy jako **zamiana porządku**, świadoma i opisana, nie jako
optymalizacja pod istniejący.

### 5.4 `CanonicalizationSession` — z tego samego powodu, co `RewriteSession`

Poprawka z review; v1 mówiła o wolnej funkcji `canonicalise(Expression)` i to był
błąd, który F3.1 popełniło i naprawiło w swojej v2.

Układ równań to **wiele korzeni jednego współdzielonego DAG-a**. Niech
`shared = c + b + a` występuje w pięciu równaniach i wymaga przebudowy do
`a + b + c`. Bez wspólnego memo każde wywołanie zbuduje **własną kopię** wyniku:

```text
bez sesji   5 strukturalnie identycznych, ale ROZŁĄCZNYCH poddrzew
            współdzielenie rozbite dokładnie tam, gdzie jest największe
z sesją     jeden wynik, pięć trafień w memo
```

Rozbicie współdzielenia jest tu gorsze niż w F3.1, bo wchodzi **przed** całym
późniejszym przetwarzaniem: F3.3 dostałoby materiał wielokrotnie większy, a
każdy jego przebieg płaciłby za to jeszcze raz.

```cpp
class CanonicalizationSession
{
public:
    explicit CanonicalizationSession(ExpressionFactory factory);

    [[nodiscard]] Expression canonicalize(const Expression& expression);

private:
    detail::ExpressionMap<Expression> memo_;   // wynik na unikalny węzeł
    ExpressionFactory factory_;
};
```

Kształt jest celowo bliźniaczy do `RewriteSession`: ten sam klucz po tożsamości
węzła, ta sama zasada „jedna sesja to jedna konfiguracja", to samo ostrzeżenie
o monotonicznym wzroście memo. Różnica jest jedna i warto ją zapisać:
**kanonizacja nie ma punktu stałego do osiągnięcia** — to jeden przebieg po
drzewie, więc nie ma tu ani limitu kroków, ani `reachedLimit`, ani transakcyjności
z F3.1 5.5. Nie ma czego przerwać w połowie.

### 5.5 Kształt odbudowanego drzewa — też musi być jednoznaczny

Druga rzecz, której v1 nie zdefiniowała, a bez której słowo „normalna" jest
puste. Sama kolejność `[a, b, c, d]` **nie wyznacza struktury**:

```text
(((a+b)+c)+d)      a+(b+(c+d))      (a+b)+(c+d)
```

Trzy drzewa, ta sama kolejność, `structurallyEqual` = fałsz między każdą parą.

**Decyzja: drzewo zbalansowane, deterministyczne** — dziel listę na pół
(`mid = n/2`), rekurencyjnie złóż obie połowy. Powody, w kolejności wagi:

```text
głębokość      log n zamiast n  -- rekurencyjne przejścia nie rosną liniowo z długością sumy
numeryka       sumowanie parami ma lepszą akumulację błędu niż długi łańcuch w lewo
skalowalność   19 składników dziś, więcej po podstawieniach F3.3
```

Konsekwencja, którą trzeba przyjąć świadomie: przy zbalansowanym kształcie
**sąsiednie stałe nie muszą trafić do jednego węzła**, więc „fabryka zwinie je
przy okazji" przestaje działać. Dlatego redukcja stałych z 3 jest osobnym krokiem
na płaskiej liście — te dwie decyzje są ze sobą związane i trzeba je podjąć
razem.

---

### 5.6 Tożsamość węzła — wymóg implementacyjny, nie oczekiwanie testu

v3 miała to jako dwa testy (`KeepsNodeIdentityWhenAlreadyNormal`,
`IsIdempotent`). Review słusznie zauważyło, że to za mało: najprostsza
implementacja — `flatten → sort → rebuild` — buduje **za każdym razem nowy,
strukturalnie identyczny DAG**, a memo tego nie ratuje, bo świeża sesja ma puste
memo.

Wymóg, wprost:

> Jeśli wejście jest już AC-normalne, `canonicalize` zwraca **oryginalny
> `Expression`** — także w sesji, która widzi je pierwszy raz.

```cpp
CanonicalizationSession fresh{factory};
sameNode(fresh.canonicalize(alreadyNormal), alreadyNormal);   // MUSI być true
```

Implementacja jest tania i nie wymaga śledzenia „czy coś się zmieniło" przez
wszystkie dziewięć kroków z 3.1:

```cpp
Expression rebuilt = /* ... pełny przebieg ... */;
return structurallyEqual(rebuilt, input) ? input : rebuilt;
```

Porównanie jest tu tanie, bo dzieci wyniku to **te same węzły** co dzieci
wejścia (memo), a `structurallyEqual` skraca na `sameNode` w pierwszej linijce.

To ta sama zasada, którą F3.1 zapisało w 6.3 i z tego samego powodu: bez niej
współdzielenie ginie przy pierwszym przebiegu, który niczego nie zmienia — a dla
kanonizatora to przypadek **typowy**, nie brzegowy, bo drugie wywołanie na tym
samym układzie równań nie ma już czego porządkować.

---

## 6. Bezpieczeństwo dziedziny — i błąd, który v1 tu powtórzyła

v1 pisała:

```text
spłaszczenie + sortowanie + odbudowa nie dodaje ani nie usuwa Divide
=> divisorsOf przed == divisorsOf po
=> dziedzina zachowana
```

i proponowała test `PreservesDomainGuardsAsASet`. **Pierwsza linijka jest
prawdziwa, druga nie wynika z niej, a test odrzucałby poprawną kanonizację.**

```text
a / (y + x)

kanonizacja mianownika:   y + x  ->  x + y

divisorsOf przed = { y + x }
divisorsOf po    = { x + y }        RÓŻNE strukturalnie

zeroSet(y + x)  ==  zeroSet(x + y)  TA SAMA dziedzina
```

To jest **ta sama klasa błędu**, którą F3.1 usunęło między v1 a v2 — tam
przykładem było `a/-(-q)` wobec `a/q`. Warunek strażników z F3.1 jest **lokalny
i tylko lokalny**: obowiązuje między wejściem reguły a jej wyjściem, w punkcie
trafienia. Zastosowany globalnie — między całymi drzewami przed i po — odrzuca
poprawne przepisania, i to niezależnie od tego, czy przepisuje reguła, czy
kanonizator.

Fakt, że popełniłem to drugi raz, jednym etapem później, jest sam w sobie
argumentem, żeby to zdanie stało w obu dokumentach, a nie tylko w tamtym.

### 6.1 Dwie różne rzeczy nazywane „dziedziną"

v3 dowodziła zachowania dziedziny tak:

```text
d == canonicalize(d) jako funkcje  =>  zeroSet równe  =>  dziedzina zachowana
```

**To jest prawdziwe nad `R` i nieprawdziwe dla `ExpressionEvaluator`**, który
liczy w `double`. Review złapało to i zmierzyłem:

```text
d = (1e16 + q) + (-1e16)

q = 1      d w double:  (1e16 + 1) = 1e16,  potem 1e16 - 1e16 = 0
           1/d  ->  DivisionByZero
           po normalizacji d = q = 1,  1/d = 1     WARTOŚĆ

q = 0.001  to samo: przed DivisionByZero, po WARTOŚĆ
```

Kanonizacja **zmieniła więc określoność w evaluatorze**. To ten sam dryf, który
§7 mierzy jako `1e-14`, tylko w punkcie, gdzie nie objawia się jako mała różnica
wartości, lecz jako przejście `DivisionByZero ↔ wartość`.

### 6.2 Którą semantykę wybieramy — i dlaczego tę

**Wybór: dziedzina symboliczna jest definiowana w semantyce algebraicznej nad
`R`**, nie przez zaokrąglenia konkretnej kolejności działań w `double`.

Przy pomiarze wyszło coś, co czyni ten wybór mocniejszym, niż wyglądał — i wart
jest zapisania, bo odwraca intuicję „normalizacja psuje dziedzinę":

```text
q = 3      d przed normalizacją liczy się na  4
           d po normalizacji                   3
```

ulp przy `1e16` wynosi 2, więc `1e16 + 3` zaokrągla się do `1e16 + 4`.
W tym kontrprzykładzie **postać AC-normalna jest wierniejsza matematyce niż
wejście**, a `DivisionByZero` przy `q = 1` jest **artefaktem kolejności
sumowania**, nie osobliwością wyrażenia. Wybranie określoności evaluatora jako
kontraktu znaczyłoby zobowiązanie się do **zachowywania artefaktów numerycznych** —
i, w konsekwencji, do zamrożenia kolejności sumowania w mianownikach, czyli do
rezygnacji z celu całego etapu.

Dwa kontrakty, rozdzielone jawnie:

```text
DZIEDZINA SYMBOLICZNA   zachowana DOKŁADNIE przez AC-normalizację, konwencję
                        znaku i eliminację odejmowania
                        -- to jest kontrakt tego etapu

EWALUACJA W double      może zmienić wartość (§7: 1,07e-14 na materiale repo)
                        i w punktach patologicznych także klasyfikację
                        „mianownik dokładnie zerowy"
                        -- podlega walidacji numerycznej, NIE jest dowodem
                        dziedziny symbolicznej
```

Konsekwencja, którą lepiej zapisać teraz niż odkryć w F3.10: **wykrywanie
osobliwości nie może opierać się na `DivisionByZero` z evaluatora.** Ten błąd
mówi „ta kolejność działań dała dokładnie zero w `double`", a nie „mianownik ma
tu zero". Osobliwości są własnością symboliczną i F3.10 musi je liczyć
symbolicznie.

### 6.3 Rozumowanie, które jest poprawne

```text
1. AC-normalizacja zachowuje ZNACZENIE ALGEBRAICZNE nad R
       przestawienie i przenawiasowanie sumy/iloczynu     -- dokładnie
       Subtract(a,b) -> Add(a, Negate(b))                 -- dokładnie, też bitowo
       konwencja parzystości znaku                        -- dokładnie
       redukcja stałych                                   -- WYŁĄCZNIE gdy dokładna
2. zatem dla każdego mianownika d:   d  ==  canonicalize(d)  jako funkcje NAD R
3. zatem                              zeroSet(d) == zeroSet(canonicalize(d))
4. zatem dziedzina SYMBOLICZNA całego wyrażenia jest zachowana
```

Czwarty wiersz kroku 1 był w v3 i v4 **wyjątkiem, który unieważniał całą tezę**:
skoro redukcja stałych zaokrągla, to `d == canonicalize(d)` nad `R` po prostu nie
zachodzi, a bez tego kroki 2–4 nie mają przesłanki. Review pokazało to na
`q + 1e16 + 1 + (-1e16)`, gdzie `zeroSet` przechodzi z `{-1}` na `{0}`.

W v5 wyjątek **znika**, bo redukcja jest wyłącznie dokładna (3.2):

> **Kanonizator nie wprowadza żadnego nowego zaokrąglenia.** Przekształcenia
> strukturalne zachowują znaczenie algebraiczne dokładnie, a stałe łączone są
> tylko wtedy, gdy połączenie jest dokładne — **sprawdzane**, nie zakładane:
> 2Sum dla sumy, test na reprezentacji `binary64` dla iloczynu (3.2.1).

Dzięki temu krok 1 jest prawdziwy **bez zastrzeżeń**, a cały dowód 1–4 stoi.

Uwaga, żeby nie było nieporozumienia: fabryka nadal zwija stałe zachłannie przy
budowie i **to się nie zmienia** — to jest semantyka wejściowego DAG-a, ustalona
w Fazie 1. Kontrakt mówi o tym, czego **kanonizator** nie dokłada od siebie.

### 6.4 Test: behawioralny, ale **na kontrolowanym materiale**

```text
PreservesDefinednessAcrossCanonicalization

    weź wyrażenie z dzieleniem, którego mianownik SIĘ PRZESTAWI:
        1 / (q + r)          po kanonizacji:  1 / (r + q)  albo odwrotnie
    punkty deterministyczne, w tym DOKŁADNIE na osobliwości:
        q = 1,  r = -1       <-- mianownik jest dokładnie zerem
        q = 1,  r = -1 +/- 1e-9
        oraz kilka zwykłych
    wymagaj ZGODNOŚCI CO DO OKREŚLONOŚCI, nie równości dzielników
```

Ten sam kształt, co `AgreesOnDefinednessAndValue` z F3.1 — mierzymy **dziedzinę**,
a nie jej składniowy zapis, a punkt osobliwy jest w liście z nazwiska.

**Czego ten test nie dowodzi, i musi to stać w jego komentarzu:** nie jest
dowodem zachowania dziedziny w ogólności. Dla materiału dobranego jak w 6.1
**przeszedłby na czerwono, choć implementacja byłaby poprawna**. Działa jako
sprawdzian regresji na wyrażeniach o umiarkowanych stałych — czyli na wszystkim,
co ten projekt buduje — a nie jako twierdzenie.

### 6.5 Representability guard obowiązuje też tutaj

Jeśli kanonizacja mianownika zwinie go do literalnego zera,
`ExpressionFactory::divide` przewróci `assert`. Dziś nie może się to zdarzyć —
redukcja stałych zwija tylko stałe, a suma symboliczna nie znika — ale to jest
własność dzisiejszego zestawu kroków, nie twierdzenie o wszystkich przyszłych.
**Guard z F3.1 2.7 obowiązuje bez zmian**: mianownik, który po kanonizacji stał
się zerem, zostaje w postaci oryginalnej.

Przykład, w którym to przestanie być hipotetyczne: gdy F3.3 doda zbieranie
wyrazów podobnych, `1/(x - x)` **będzie** kandydatem na zero w mianowniku.

---

## 7. Zmiennoprzecinkowość — i dlaczego to nie blokuje

Przestawienie sumy **zmienia wynik numeryczny**: dodawanie `double` nie jest
łączne. To jedyne miejsce, w którym F3.2 różni się jakościowo od F3.1 — reguły
tamtego etapu zachowywały wartość dokładnie, kanonizacja nie.

Zmierzone na tym materiale, 200 losowych konfiguracji ze stałym ziarnem, każde
z 12 równań obu robotów, porównanie oryginalnej kolejności z odwróconą:

```text
kr640.urdf       2400 porównań   worst |abs| = 4,44e-16   worst rel = 5,62e-15
kr4_r600.urdf    2400 porównań   worst |abs| = 3,33e-16   worst rel = 1,07e-14
```

Odniesienie: tolerancja w `test_numeric_fk_validation.cpp` to **`1e-12`**.
Najgorszy zmierzony dryf jest od niej **około sto razy mniejszy**.

Uczciwe zastrzeżenia:

- to pomiar na **odwróconej** kolejności, czyli na jednej konkretnej permutacji;
  porządek kanoniczny da inną, ale rząd wielkości bierze się z długości sumy
  (11 i 19 składników), a nie z tego, która permutacja;
- to **nie jest** gwarancja, tylko obserwacja na dwóch robotach — dłuższy łańcuch
  kinematyczny da dłuższe sumy i większy dryf;
- kontrakt (E) mówi o równości **algebraicznej** i ta jest zachowana **dokładnie**.
  Numeryka jest osobnym pytaniem i tak trzeba ją w dokumencie trzymać.

**Rekomendacja: przyjąć.** Sto razy poniżej istniejącej tolerancji, przy zysku,
bez którego trzy późniejsze etapy nie mają na czym stanąć.

---

## 8. Co zostaje z listy roadmapy — i gdzie to trafia

```text
ODPADA całkiem — fabryka robi to przy budowie, reguła nie ma czego dopasować:
    x + 0,  x - 0,  x * 1,  x / 1,  zwijanie stałych

DO KANONIZATORA — poprawne, ale NIE zmniejszają miary:
    Subtract(a, b)        ->  Add(a, Negate(b))          3 -> 4   ROŚNIE
    Negate(Multiply(a,b)) ->  konwencja parzystości      4 -> 4   BEZ ZMIANY

DO KANONIZATORA — mimo że miarę ZMNIEJSZA:
    -(-x) -> x                                           3 -> 1   MALEJE

REGUŁY SILNIKA F3.1 dostarczone przez ten etap:   ŻADNE
```

Dwa środkowe wiersze to poprawka z v2, trzeci — z v3, i to on wymaga
wyjaśnienia, bo `-(-x)` **spełnia** kontrakt miary i mógłby być regułą.

Powodem jest 4: kanonizator sam produkuje `-(-x)` przy normalizacji odejmowania,
więc zostawienie tej reguły silnikowi znaczyłoby, że wynik kanonizacji nie jest
punktem stałym względem reguł — albo że trzeba przeplatać oba mechanizmy bez
dowodu terminacji. Jako część konwencji parzystości (3.3) `-(-x)` znika **za
darmo**, bo dwa minusy to parzystość parzysta.

Kryterium podziału jest więc **dwuczłonowe** i v2 miała je za wąskie:

```text
reguła silnika F3.1 wymaga:   spadku miary
                        ORAZ:   żeby kanonizator nie odtwarzał jej wzorca
```

### 8.1 Konsekwencja, którą trzeba powiedzieć wprost

**F3.2 nie dostarcza żadnej reguły produkcyjnej dla silnika z F3.1.** Silnik
zbudowany w poprzednim etapie zostaje bez wołającego jeszcze jeden etap.

Nie jest to powód do niepokoju, ale **jest** powód, żeby to zapisać zamiast
znaleźć jakąś regułę na siłę. F3.1 było fundamentem — mechanizmem bezpiecznego
przepisywania — i jego pierwsze poważne zastosowanie to **F3.3**: tożsamości
trygonometryczne są dokładnie tym rodzajem przekształcenia, które zmniejsza
drzewo, wymaga (E) i nie da się zapisać jako normalizacja kształtu.

Zapisuję to również dlatego, że **wzmacnia argument z 13**: skoro F3.2 nie daje
użytkownika silnikowi, tym bardziej powinien dać użytkownika kanonizatorowi —
inaczej dwa etapy z rzędu kończą się mechanizmem bez wołającego.

Przy każdej transformacji dokument implementacyjny ma podać **zmierzoną liczbę
trafień na materiale repo**. Dziś wynosi ona zero dla wszystkich, w tym dla
`Subtract` — `sub=0` w obu układach równań — i tak ma być napisane, zamiast
sugerować, że etap coś upraszcza. Etap **porządkuje**, i to jest jego cała
wartość.

---

## 9. Warstwy: gdzie mieszka mechanizm, a gdzie orkiestracja

v1 zostawiała to jako pytanie otwarte; review odpowiedziało i **zgadzam się**,
bo odpowiedź rozwiązuje też problem z 5.4:

```text
symbolic/
    ExpressionCanonicalizer          transformacja jednego wyrażenia
    CanonicalizationSession          memo dzielone między korzeniami

model/  albo  builders/
    EquationSimplifier               orkiestrator układu równań
        tworzy JEDNĄ CanonicalizationSession
        kanonizuje lhs i rhs wszystkich równań przez tę samą sesję
        później: RewriteSession, trygonometria, operacje na poziomie równania
```

To **nie jest** rezygnacja z `EquationSimplifier` z roadmapy — przeciwnie,
nadaje mu właściwą rolę. Wszystko, co ten etap robi, jest własnością
pojedynczego wyrażenia, więc mechanizm należy do warstwy wyrażeń. Ale to
**orkiestrator** wie, że dwadzieścia cztery strony równań to jeden DAG i że mają
iść przez jedną sesję — a to jest wiedza o układzie, nie o wyrażeniu.

Nazwa etapu w roadmapie zostaje. Proponuję dopisać do niej doprecyzowanie:

> **F3.2 — `EquationSimplifier`, fundament: AC-normalizacja wyrażeń**

W F3.2 `EquationSimplifier` jest cienki — pętla po równaniach i jedna sesja.
Rośnie w F3.3, gdy dojdą przekształcenia działające na **równaniu**, a nie na
wyrażeniu: przenoszenie składników między stronami, mnożenie stronami. Dopiero
wtedy warstwa równań zaczyna robić coś, czego warstwa wyrażeń nie umie.

---

## 10. Zakres — czego w F3.2 nie ma

```text
NIE MA   zbierania wyrazów podobnych (2x + 3x -> 5x)
NIE MA   rozdzielności (a*(b+c) -> a*b + a*c ani w drugą stronę)
NIE MA   tożsamości trygonometrycznych  -- to F3.3
NIE MA   wpięcia w ConstraintBuilder ani w fasadę
```

Trzy pierwsze wymagają decyzji, **która z dwóch równoważnych postaci jest ta
właściwa** — a to pytanie ma sens dopiero, gdy wiadomo, co solver z F3.8 chce
dostać. Rozdzielność w złą stronę potrafi też **powiększyć** drzewo wykładniczo,
więc to nie jest decyzja do podjęcia „przy okazji porządkowania".

I właśnie dlatego etap nazywa się **AC-normalny**, a nie „kanoniczny" (3.1):
`a*(b+c)` i `a*b + a*c` wyjdą stąd jako dwa różne wyrażenia i tak ma być.

Wpięcie w `ConstraintBuilder` i fasadę zostawiam świadomie: dopóki nie ma reguł
trygonometrycznych, normalizacja niczego konsumentowi nie daje poza inną
kolejnością składników — a zmiana tego, co fasada zwraca, jest zmianą kontraktu
modułu, który MotionBridge ma konsumować.

---

## 11. Testy — szkic

```text
porządek  (strukturalny, 5.2)
  IsTotalOnMixedNodeKinds                dowolna para porównywalna
  IsAntisymmetricAndTransitive           na wygenerowanym zbiorze wyrażeń
  AgreesWithStructuralEquality           equal <=> ani <, ani >
  DoesNotDependOnConstructionOrder       to samo wyrażenie zbudowane dwiema drogami
  OrdersByNameNotByHash                  „z" po „a"; łapie podmianę porządku z 5.3

AC-normalizacja
  FlattensNestedAssociativeChains        ((a+b)+c) i (a+(b+c)) -> ta sama postać
  SortsSumsAndProducts
  ReducesConstantsExplicitly             3 + x + 4 -> 7 + x, niezależnie od kształtu
  CanonicalizesInexactConstantsIdentically
                                         1e16 + x + (-1e16) + y + 1  i przestawione
                                         -> structurallyEqual, ten sam UPORZĄDKOWANY
                                            zestaw reszt.  NIE „jedna stała" (3.2)
  ReducesOnlyExactlyRepresentableSums    3 + x + 4 -> 7 + x   ŁĄCZY
                                         q + 1e16 + 1 + (-1e16)  NIE ŁĄCZY (3.2)
  RejectsUnderflowedProductAsInexact     denorm_min * 0,5 — fma mówi „0", odrzucone
  RejectsProductWhoseRoundingErrorUnderflows
                                         nextafter(2^-511)^2 — wynik NORMALNY,
                                         fma mówi „0", predykat odrzuca (3.2.1)
  AcceptsExactProductOutsideTheNormalRange
                                         2^-1000 * 2^-70 — wynik SUBNORMALNY,
                                         ale dokładny: musi zostać złączony
  AcceptsExactProductThatFmaAlsoAccepts   1e200 * 3 — dokładny; odrzucenie go
                                         rozbiłoby postać normalną (3.2.1)
  AgreesWithExactArithmeticOnASweep      staly zestaw par, w tym subnormalne;
                                         wyrocznia liczona bez zaokrągleń
  ExactReductionHasUniquePairingSchedule stałe 1, 2, 2^53 -> reszty [3, 2^53],
                                         nigdy [1, 2^53+2]  (3.2.2)
  ReducesConstantsToAFixedPoint          2^52, 2^52+1, 2^52+3 -> JEDNA stała;
                                         jedno przejście zostawiłoby dwie (3.2.2)
  PreservesZeroSetOfADenominator         ten sam mianownik: 1/(q+1e16+1+(-1e16))
                                         zero pozostaje w q = -1, nie przenosi się na 0
  KeepsUnjoinableConstantsApartInRebuild fabryka nie dostaje dwóch stałych naraz (3.2.3)
  DropsNeutralConstant                   0 + x -> x  po redukcji, nie przez fabrykę
  RebuildsInBalancedShape                jednoznaczny kształt (5.5), nie tylko kolejność
  NormalizesSignByParity                 -(a*b), (-a)*b, a*(-b) -> JEDEN DAG (3.3)
  NormalizesNegativeConstantSign         (-2)*x i 2*(-x) -> -(2*x);  (-1)*x -> -x
  KeepsNegativeZeroOutOfParity           -0.0 nie przełącza znaku (3.3)
  CancelsEvenSignParity                  (-a)*(-b) -> a*b, bez osobnej reguły
  EliminatesSubtractionEntirely          wyjście nie zawiera węzła Subtract (3.4)
  NormalizesNestedSubtractionWithPolarity  a-(b-c)  i  (a-b)+c  -> ta sama postać
  NormalizesExplicitNegatedSum           -(a+b),  0-(a+b),  (-a)+(-b) -> ta sama postać
  FlattensOperatorRevealedByChildNormalization
                                         a + -(-(b+c))  ->  PŁASKIE [a,b,c] (3.4)
  NormalizesSignThroughDivision          -(a/b), (-a)/b, a/(-b) -> JEDEN DAG (3.3)
  NormalizesMinusOneTimesSum             (-1)*(a+b), -(a+b), (-a)+(-b)
                                         -> jedna postać, BEZ Negate nad sumą (3.3)
  RemovesDoubleNegation                  -(-x) -> x  w KANONIZATORZE, nie regułą
  IsIdempotent                           normalize(normalize(E)) sameNode normalize(E)

sesja  (5.4)
  SharesResultAcrossRootsInOneSession     drugi korzeń nie liczy od nowa
  PreservesSharingAcrossNormalization     wspólne poddrzewo zostaje JEDNYM węzłem
  KeepsNodeIdentityWhenAlreadyNormal      sameNode(wejście, wynik)
  KeepsNodeIdentityInAFreshSession        to samo przy PUSTYM memo (5.6)

dziedzina  (6)
  PreservesDefinednessAcrossCanonicalization   1/(q+r), punkt q=1, r=-1
  KeepsOriginalDenominatorIfItBecomesZero      guard z F3.1 2.7

EquationSimplifier  (13)
  NormalizesEveryEquationOfASystem        lhs i rhs wszystkich równań
  PreservesEverythingButTheExpressions    unknowns i ich kolejność, kolejność równań,
                                          task kind, EquationKind, source — wszystko
  SharesOneSessionAcrossTheSystem         wspólne poddrzewo FK zostaje JEDNYM węzłem
                                          -- to jest test, którego syntetyczny nie zastąpi

numeryka i integracja
  AgreesNumericallyWithinToleranceOnFk    12 równań, losowe konfiguracje, próg 1e-12
  NormalizesRealEquationSystem            end-to-end, oba roboty
```

Dwa testy zasługują na komentarz, bo powstały wprost z review:

`OrdersByNameNotByHash` istnieje po to, żeby podmiana porządku strukturalnego na
odciskowy (5.3) **złamała test**, a nie przeszła niezauważona. Bez niego oba
porządki dają „jakąś postać normalną" i różnica jest niewidoczna.

`RebuildsInBalancedShape` pilnuje 5.5. Test sprawdzający tylko kolejność
przeszedłby dla trzech różnych drzew, czyli dla czegoś, co **nie jest** postacią
normalną.

`CanonicalizesInexactConstantsIdentically` to kontrprzykład z review zapisany jako
test — ale z **poprawioną tezą**, i to jest zmiana z v6. v5 kazała wymagać „tej
samej stałej", co po przejściu na łączenie wyłącznie dokładne jest już
**nieprawdą**: `1e16 + 1` się nie złączy, więc poprawny wynik zawiera kilka reszt.
Wymagamy więc tego, co faktycznie obiecujemy: **jednej postaci** i **tego samego
uporządkowanego zestawu reszt**.

`ExactReductionHasUniquePairingSchedule` pilnuje 3.2.2 i istnieje po to, żeby
implementator nie zamienił harmonogramu na „spróbujmy złożyć co się da" — wersja
zachłanna przeszłaby każdy inny test w tym pliku.

`RejectsUnderflowedProductAsInexact` jest jedynym testem, który łapi błąd
z 3.2.1, i bez niego wersja z samą `fma` wygląda poprawnie na wszystkich
pozostałych przypadkach. `AgreesWithExactArithmeticOnASweep` jest za to jedynym
testem, który łapie **fałszywe negatywy** — a te nie objawiają się złym wynikiem,
tylko drugą postacią normalną dla wejścia, które nikt akurat nie sprawdził.

`SharesOneSessionAcrossTheSystem` jest jedynym testem, który wymaga
`EquationSimplifier` — i głównym powodem, dla którego powstaje już teraz (13).
Współdzielenie **między równaniami** da się sprawdzić tylko na prawdziwym
układzie; syntetyczne drzewo dowodzi mechanizmu, nie integracji.

Liczba testowana maszynowo w proposalu implementacyjnym — z powodu opisanego
w F3.1 6.7, gdzie deklarowany gate dwa razy okazał się przewidywaniem.

---

## 12. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Etap dostarcza reguły, które nigdy nie trafiają | **realne, zmierzone** | Zakres przesunięty na AC-normalizację; przy każdej pozostałej transformacji liczba trafień podana wprost (8) |
| Globalne porównanie `divisorsOf` odrzuca poprawną kanonizację | **realne, popełnione w v1 — drugi raz w projekcie** | Warunek behawioralny zamiast strukturalnego (6.4); rozumowanie przez `zeroSet` (6.3) |
| Transformacja bez spadku miary w tablicy reguł | **realne, popełnione w v1** | Kryterium podziału: miara, nie „czy upraszcza" (8); `-(a*b)` i `Subtract` do kanonizatora |
| Kanonizacja rozbija współdzielenie DAG-a | **realne** | `CanonicalizationSession` z memo po tożsamości węzła (5.4), własny test |
| Dwa różne porządki opisane jako jeden | **realne, złapane w review** | Odcisk wycofany; jeden porządek strukturalny (5.3), test `OrdersByNameNotByHash` |
| „Postać normalna" bez zdefiniowanego kształtu drzewa | **realne, złapane w review** | Zbalansowane drzewo, deterministycznie (5.5) |
| Redukcja stałych w kolejności wejściowej daje różne postacie | **realne, zmierzone, złapane w review** | Stałe sortowane **przed** redukcją, składane zbalansowanie (3.2); własny test na kontrprzykładzie |
| Obietnica „wszystkie stałe → jedna" nie do dotrzymania | **realne, zmierzone** | Fabryka odmawia zwinięcia przy przepełnieniu; kontrakt ogranicza obietnicę wprost (3.2) |
| Redukcja stałych zmienia `zeroSet` wyrażenia | **realne, zmierzone, złapane w review** | Łączenie **wyłącznie dokładne** (3.2); dowód z 6.3 nie ma już wyjątku |
| `fma` ogłasza dokładnym iloczyn, którego błąd sam wpadł w underflow | **realne, zmierzone, złapane w review — dwa razy** | Predykat liczony z **reprezentacji**, nie z reszty (3.2.1) |
| Zachowawczy predykat odrzuca dokładny iloczyn i rozbija postać normalną | **realne, złapane w review** | Kontrakt dwustronny: sound **i** complete; zweryfikowany wobec arytmetyki wymiernej na 8000 parach (3.2.1) |
| Reducer deterministyczny, ale nie idempotentny | **realne, zmierzone, złapane w review** | Powtarzanie do punktu stałego, terminacja przez malejącą liczbę stałych (3.2.2) |
| Końcowy znak owija sumę w `Negate` | **realne, złapane w review** | `canonicalNegate` zamiast `factory.negate` (3.3), własny test |
| Usunięcie zwijania stałych jako „uproszczenie" | **realne, kuszące** | Przewraca postać normalną: `(3+4)+x` i `3+x+4` dałyby dwa wyniki (3.2.4) |
| Dwa dopuszczalne parowania stałych dają dwie postacie | **realne, zmierzone, złapane w review** | Harmonogram od lewej z akumulatorem, bez szukania partnera — część kontraktu (3.2.2) |
| Fabryka zwija stałe, których kanonizator świadomie nie połączył | **realne, nieoczywiste** | Stałe doklejane na zewnątrz zbalansowanego drzewa, nigdy dwie w jednym `add` (3.2.3) |
| Kanonizacja dziecka odsłania operator AC już po zebraniu łańcucha | **realne, złapane w review** | Kolektor spłaszcza wynik (splice), terminacja oparta na 5.6 (3.4) |
| `-(a+b)` obok `0-(a+b)` jako dwie postacie | **realne, złapane w review** | Zejście przez `Negate` w kolektorze (3.4), własny test |
| `F3.5` zakłada globalną jednoznaczność znaku | realne | Zakres konwencji wypisany wprost, z `Divide` w środku i z tym, co zostaje poza (3.3) |
| Normalizacja znaku nie daje jednej postaci | **realne, złapane w review** | Konwencja parzystości: znak wychodzi na zewnątrz iloczynu, sortowanie bez znaków (3.3) |
| Transformacja po sortowaniu rozbija własny wynik | **realne, złapane w review** | `Subtract` i znak obsługiwane **przy zbieraniu**, nie po odbudowie (3.3, 3.4) |
| „Dziedzina" znaczy dwie różne rzeczy naraz | **realne, zmierzone, złapane w review** | Dwa rozdzielone kontrakty: symboliczna nad `R` — dokładnie; określoność w `double` — nie (6.2). Konsekwencja dla F3.10 zapisana |
| Parzystość znaku nie widzi ujemnych stałych | **realne, złapane w review** | Znak liczony z **dwóch** źródeł: `Negate` i `Constant(c<0)` (3.3), własny test |
| Zagnieżdżone odejmowanie daje dwie postacie | **realne, złapane w review** | Kolektor sumy ze znakiem, schodzący przez `Subtract` (3.4) |
| Kanonizator przebudowuje DAG, który już był normalny | **realne** | Wymóg tożsamości węzła także w świeżej sesji (5.6) |
| Kanonizacja odtwarza redex usunięty przez regułę | **realne** | `-(-x)` w kanonizatorze; zero reguł produkcyjnych w tym etapie (4, 8.1) |
| Porządek zależny od adresów albo od `std::hash` | **realne, klasyczne** | Zakaz wprost (5.1, 5.3); test `DoesNotDependOnConstructionOrder` |
| Dryf zmiennoprzecinkowy | realne, zmierzone | 1,07e-14 wobec tolerancji 1e-12 (7); kształt zbalansowany raczej pomaga (5.5) |
| `structurallyEqual` wzięte za równość algebraiczną | **realne, po tym etapie kuszące** | Nazwa „AC-normalna" i jawne ograniczenie w komentarzu klasy (3.1) |
| Kanonizacja mianownika daje literalne zero | dziś niemożliwe, po F3.3 realne | Guard z F3.1 2.7 obowiązuje bez zmian (6.5) |

---

## 13. Rekomendacja

Przyjąć **zakres z 3** zamiast listy z roadmapy, z podziałem transformacji z 8
i warstwami z 9. Zapisać w roadmapie, że F3.2 zmienił treść — razem z powodem
i z liczbami, żeby za pięć etapów nikt nie szukał brakujących reguł.

Oba pytania z poprzednich wersji są **rozstrzygnięte**:

**Warstwa** — mechanizm w `symbolic/`, orkiestracja w `EquationSimplifier`, nazwa
etapu w roadmapie bez zmian (9).

**Czy `EquationSimplifier` powstaje już teraz** — **tak**, i argument z 8.1
przeważył: skoro F3.2 nie dostarcza silnikowi ani jednej reguły, to bez cienkiego
orkiestratora **dwa etapy z rzędu** kończą się mechanizmem bez wołającego.
Kształt — minimalny:

```cpp
class EquationSimplifier
{
public:
    explicit EquationSimplifier(ExpressionFactory factory);

    // Every equation of the system through ONE session, so the FK subtrees the
    // twelve equations share stay one node instead of twelve copies.
    //
    // ALL NON-EXPRESSION FIELDS ARE PRESERVED EXACTLY. Not a list of the
    // fields that exist today -- unknowns and their order, equation order,
    // task kind, each equation's kind and source -- but the rule itself, so a
    // field added to Equation next year does not quietly fall out of it.
    //
    // This class normalises expressions. It decides nothing about the system.
    [[nodiscard]] IkEquationSystem normalize(const IkEquationSystem& system);
};
```

Co to daje poza estetyką:

```text
prawdziwy użytkownik mechanizmu, nie tylko testy jednostkowe
test współdzielenia MIĘDZY równaniami -- syntetyczne drzewo tego nie zastąpi
F3.3 dostaje gotowy punkt integracji zamiast projektować orkiestrację przy okazji
etap domknięty pionowo: od wyrażenia do układu równań
```

**Nie** wpinam go w fasadę ani w `ConstraintBuilder` — zmiana tego, co zwraca
`IkEquationBuilder`, jest zmianą kontraktu modułu, który ma konsumować
MotionBridge, a dopóki nie ma trygonometrii, kanonizacja daje konsumentowi
wyłącznie inną kolejność składników.

Jedna rzecz zapisana jako **znane wejście do F3.3**, żeby nie była odkryciem:
gdy pojawią się reguły trygonometryczne, trzeba będzie rozstrzygnąć, czy są
pisane tak, żeby **zachowywać** postać AC-normalną, czy przeplatanie kanonizacji
z przepisywaniem dostanie własny dowód terminacji (4). To pytanie tamtego etapu,
ale rodzi się tutaj.

Po zatwierdzeniu — proposal implementacyjny z pełnym kodem i testami zliczonymi
maszynowo, po uprzedniej kompilacji, z powodu opisanego w F3.1 6.7.
