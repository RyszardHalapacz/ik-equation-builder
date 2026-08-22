# Proposal: postać kanoniczna — F3.2, architektura

**Etap:** F3.2 · **v4 — do review** · 2026-08-22 · część 1 z 2

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

  1. zbierz składniki/czynniki maksymalnego łańcucha AC
        Subtract wchodzi TUTAJ, nigdy nie wychodzi           <-- 3.4
  2. kanonizuj rekurencyjnie każdy składnik
  3. dla iloczynu: zdejmij minusy, policz PARZYSTOŚĆ        <-- 3.3
  4. rozdziel na STAŁE i pozostałe
  5. posortuj stałe porządkiem z 5.2, potem je zredukuj      <-- 3.2, w tej kolejności
  6. odrzuć stałą neutralną, jeśli taka wyszła
  7. posortuj pozostałe składniki porządkiem z 5.2
  8. odbuduj w zbalansowanym kształcie                       <-- 5.5
  9. jeśli parzystość nieparzysta: JEDEN Negate na zewnątrz  <-- 3.3
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

### 3.2 Stałe: **najpierw kolejność, potem redukcja**

Blocker z review i — jak się okazuje — ten sam problem, który §7 opisuje dla
składników symbolicznych, tylko groźniejszy, bo dotyczy **postaci**, a nie tylko
wartości.

v2 kazała redukować stałe „na płaskiej liście, gdzie jest to trywialne
i deterministyczne". Trywialne — tak. Deterministyczne — **nie**, jeśli redukcja
idzie w kolejności przyjścia z wejścia. Dodawanie `double` nie jest łączne, więc
dwa **AC-równoważne** wejścia dają różne stałe wynikowe:

```text
E1:  1e16 + x + (-1e16) + y + 1        stałe w kolejności:  1e16, -1e16, 1
E2:  1e16 + x + 1       + y + (-1e16)  stałe w kolejności:  1e16, 1, -1e16

zmierzone:
    (1e16 + -1e16) + 1  =  1
    (1e16 + 1) + -1e16  =  0
```

Fabryka nie zwinie ich wcześniej — stałe są rozdzielone symbolami, więc żaden
`add` nie widzi dwóch stałych naraz. Dwa wejścia równoważne względem AC
dostałyby więc **różne postacie normalne**, co przewraca cały sens etapu.

**Poprawka: stałe są sortowane porządkiem z 5.2 przed redukcją**, a redukcja idzie
tym samym zbalansowanym układem, co odbudowa (5.5) — bo skoro kształt ma być
ustalony dla drzewa wynikowego, nie ma powodu, żeby składanie stałych było
`left-fold`em. Wtedy oba wejścia widzą `-1e16, 1, 1e16` i składają je identycznie.

**Granica obietnicy.** Nie wolno napisać „wszystkie stałe → jedna stała", bo
`ExpressionFactory` **odmawia zwinięcia**, gdy wynik nie jest skończony — i to nie
jest przypadek teoretyczny, zwłaszcza dla mnożenia:

```text
zmierzone:
    (1e200 * 1e200) * 1e-200  =  inf        fabryka ODMÓWI zwinięcia
     1e200 * (1e200 * 1e-200) =  1e200      zwija się normalnie
```

Kontrakt brzmi więc:

> stałe są redukowane deterministycznie **na tyle, na ile pozwala
> `ExpressionFactory`**; stałe niezredukowane zostają w deterministycznej
> kolejności AC-normalnej.

Zauważ, że ustalona kolejność stałych **poprawia** też szansę na zwinięcie:
posortowane rosnąco i składane parami rzadziej trafiają na przepełnienie niż
przypadkowa kolejność z wejścia. To skutek uboczny, nie gwarancja — ale w dobrą
stronę.

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
parity parzyste   -> zostaw
parity nieparzyste -> jeden Negate NA ZEWNĄTRZ
```

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

### 3.4 `Subtract` nie wychodzi z kanonizatora

To samo rozumowanie, co w 3.3, i też z review. v2 wymieniała normalizację
odejmowania jako krok **po** odbudowie — czyli tworzyła świeży `Add`, który nie
przeszedł ani spłaszczenia, ani sortowania, ani zbalansowanej odbudowy.

v3 pisała to jako:

```text
canonicalize(Subtract(a, b))
    = canonicalizeAddChain( canonicalize(a), negated(canonicalize(b)) )
```

i **to nadal nie domyka zagnieżdżeń**, co review pokazało wprost:

```text
a - (b - c)      ->   a + (-(b + (-c)))      minus został NA ZEWNĄTRZ sumy
(a - b) + c      ->   a + (-b) + c           płasko

to samo wyrażenie, dwie różne postacie
```

**Poprawka: kolektor sumy niesie znak.** Zbieranie składników jest sparametryzowane
biegunowością i schodzi przez `Subtract` bez budowania czegokolwiek po drodze:

```text
collectAdd(expr, sign):

    Add(a, b)        ->  collectAdd(a,  sign);  collectAdd(b,  sign)
    Subtract(a, b)   ->  collectAdd(a,  sign);  collectAdd(b, -sign)

    cokolwiek innego ->  term := canonicalize(expr)
                         jeśli sign < 0:  term := canonicalNegate(term)
                         dopisz term do listy
```

Wtedy oba wyrażenia dochodzą do tej samej płaskiej listy:

```text
a - (b - c)   ->   +a, -b, +c
(a - b) + c   ->   +a, -b, +c
```

`canonicalNegate` to nie `factory.negate` — to nadanie znaku **zgodnie
z konwencją z 3.3**: stałej zmienia wartość na przeciwną, iloczynowi przełącza
parzystość, `Negate(x)` sprowadza do `x`. Dzięki temu `-(-x)` znika tu **z
definicji**, a nie przez osobną regułę — co domyka argument z 4 i 8.

Daje to postcondition, który warto wypowiedzieć osobno, bo jest mocny i tani:

> **Wyjście AC-normalne nie zawiera żadnego węzła `Subtract`.**

`F3.3` i `F3.5` dostają dzięki temu język o jeden rodzaj węzła węższy — wzorce
nie muszą mieć wariantu na odejmowanie. Zamiana jest przy tym **dokładna
zmiennoprzecinkowo**: `a - b` i `a + (-b)` dają bitowo ten sam wynik, więc nie
dokłada się do dryfu z 7.

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
       redukcja stałych                                   -- z zaokrągleniem double
2. zatem dla każdego mianownika d:   d  ==  canonicalize(d)  jako funkcje NAD R
3. zatem                              zeroSet(d) == zeroSet(canonicalize(d))
4. zatem dziedzina SYMBOLICZNA całego wyrażenia jest zachowana
```

Czwarty wiersz kroku 1 pozostaje doprecyzowaniem z v3: `c1 + c2` w liczbach
rzeczywistych i `double(c1) + double(c2)` to **nie zawsze ta sama liczba**, więc
redukcja stałych nie jest równoważnością matematyczną w ścisłym sensie:

> **Przekształcenia strukturalne zachowują znaczenie algebraiczne dokładnie;
> redukcja stałych podlega istniejącej semantyce skończonych `double`
> z `ExpressionFactory` i może wprowadzić zwykłe zaokrąglenie
> zmiennoprzecinkowe.**

To nie jest nowe ryzyko — fabryka zwija stałe od Fazy 1 i cały projekt na tym
stoi.

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
  ReducesConstantsInCanonicalOrder       1e16 + x + (-1e16) + y + 1  i przestawione
                                         -> TA SAMA stała  (3.2, kontrprzykład z review)
  KeepsUnreducibleConstantsInOrder       gdy fabryka odmówi zwinięcia (overflow)
  DropsNeutralConstant                   0 + x -> x  po redukcji, nie przez fabrykę
  RebuildsInBalancedShape                jednoznaczny kształt (5.5), nie tylko kolejność
  NormalizesSignByParity                 -(a*b), (-a)*b, a*(-b) -> JEDEN DAG (3.3)
  NormalizesNegativeConstantSign         (-2)*x i 2*(-x) -> -(2*x);  (-1)*x -> -x
  KeepsNegativeZeroOutOfParity           -0.0 nie przełącza znaku (3.3)
  CancelsEvenSignParity                  (-a)*(-b) -> a*b, bez osobnej reguły
  EliminatesSubtractionEntirely          wyjście nie zawiera węzła Subtract (3.4)
  NormalizesNestedSubtractionWithPolarity  a-(b-c)  i  (a-b)+c  -> ta sama postać
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

`ReducesConstantsInCanonicalOrder` jest wprost kontrprzykładem z review, zapisanym
jako test: dwa AC-równoważne wejścia różniące się **wyłącznie kolejnością stałych**
muszą dać tę samą postać. Bez niego błąd z 3.2 przechodzi niezauważony, bo
na materiale repo stałych w sumach prawie nie ma.

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
