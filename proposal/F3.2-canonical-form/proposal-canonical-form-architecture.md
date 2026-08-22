# Proposal: postać kanoniczna — F3.2, architektura

**Etap:** F3.2 · **v2 — do review** · 2026-08-22 · część 1 z 2

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

Proponowany zakres F3.2:

```text
1. spłaszczenie maksymalnych łańcuchów Add i Multiply
2. wydzielenie i zredukowanie STAŁYCH łańcucha do jednej      <-- jawnie, nie „przy okazji"
3. odrzucenie stałej neutralnej, jeśli taka wyszła
4. uporządkowanie pozostałych składników porządkiem TOTALNYM i deterministycznym
5. odbudowa w JEDNOZNACZNYM kształcie drzewa                  <-- 5.5
6. normalizacja odejmowania i znaku                            <-- 9, tu, nie w regułach
7. jedna reguła silnika F3.1: -(-x) -> x
```

Punkty 2 i 3 były w v1 zapisane jako „fabryka zrobi to sama, gdy porządek zbierze
stałe obok siebie". To było **liczenie na przypadek**: zwijanie zachodzi tylko dla
stałych, które po odbudowie trafią do jednego węzła `Add`, a przy zbalansowanym
kształcie z 5.5 nie ma gwarancji, że trafią. Redukcja stałych jest teraz
**osobnym, jawnym krokiem** — wykonywanym na płaskiej liście, gdzie jest trywialna
i deterministyczna.

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
Kolejność względem silnika ma znaczenie i proponuję:

```text
normalize(E)   ->   rewrite(E)   ->   normalize(E)
     |                  |                  |
CanonicalizationSession  |        ta sama sesja, to samo memo
              RewriteSession
```

Pierwszy przebieg daje regułom przewidywalny kształt do dopasowania, drugi
porządkuje to, co reguły zbudowały. Obie sesje są tworzone **raz na układ
równań**, nie raz na wyrażenie — z powodu opisanego w 5.4. Alternatywa — przeplatanie do punktu stałego
obu mechanizmów — jest **odrzucona** w tym etapie: terminacja takiej pętli nie
wynika już z jednej miary i wymagałaby własnego dowodu.

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

### 6.1 Rozumowanie, które jest poprawne

```text
1. AC-normalizacja jest równoważnością ALGEBRAICZNĄ
       przestawienie i przenawiasowanie sumy/iloczynu nie zmienia wartości
       redukcja stałych nie zmienia wartości
2. zatem dla każdego mianownika d:   d  ==  canonicalize(d)  jako funkcje
3. zatem                              zeroSet(d) == zeroSet(canonicalize(d))
4. zatem dziedzina całego wyrażenia jest zachowana
```

Krok 2 jest przesłanką i to on niesie cały ciężar — dokładnie tak, jak (E) niesie
go w F3.1. Różnica na korzyść tego etapu: tam (E) było kontraktem **autora
reguły**, którego silnik nie sprawdza; tutaj równoważność wynika z **jednej
własności `Add` i `Multiply`**, zapisanej raz i obowiązującej dla wszystkich
wejść. Nie ma dwudziestu autorów reguł, jest jedna transformacja.

### 6.2 Test: **behawioralny**, nie strukturalny

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

Ten sam kształt, co `AgreesOnDefinednessAndValue` z F3.1 — i z tego samego
powodu: mierzymy **dziedzinę**, a nie jej składniowy zapis. Punkt osobliwy jest
w liście z nazwiska, bo tylko w nim widać różnicę, gdyby jakaś była.

### 6.3 Representability guard obowiązuje też tutaj

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

v1 zostawiała dwie reguły silnikowi i **jedna z nich tam nie może być**:

```text
ODPADA całkiem — fabryka robi to przy budowie, reguła nie ma czego dopasować:
    x + 0,  x - 0,  x * 1,  x / 1,  zwijanie stałych

DO KANONIZATORA — poprawne, ale NIE zmniejszają miary:
    Subtract(a, b)        ->  Add(a, Negate(b))          3 -> 4   ROŚNIE
    Negate(Multiply(a,b)) ->  Multiply(Negate(a), b)     4 -> 4   BEZ ZMIANY

REGUŁA SILNIKA F3.1 — jedyna, która spełnia kontrakt miary:
    -(-x) -> x                                           3 -> 1   MALEJE
```

Drugi wiersz jest poprawką z review i **przeoczeniem, które mnie zastanawia**:
v1 policzyła miarę dla `x - y → x + (-y)`, zauważyła wzrost, poświęciła temu
osobną sekcję z trzema wariantami — i **w tym samym dokumencie**, dwie sekcje
wcześniej, zostawiła `-(a*b) → (-a)*b` w tablicy reguł, nie licząc jej wcale.

```text
measure(-(a*b))    = Negate + Multiply + a + b = 4
measure((-a)*b)    = Multiply + Negate + a + b = 4
```

Kontrakt wymaga **ścisłego** spadku, więc `assert` z `RewriteEngine.cpp`
zatrzymałby build diagnostyczny przy pierwszym trafieniu. Dobra wiadomość: ten
`assert` istnieje właśnie po to i zadziałałby, zanim ktokolwiek zdążyłby to
zobaczyć w działaniu.

Wniosek ogólniejszy, wart zapisania: **kryterium podziału nie jest „czy to
upraszcza", tylko „czy to zmniejsza miarę"**. Wszystko, co przekształca do
równoważnej postaci **bez skracania drzewa**, należy do kanonizatora — bo
kanonizator jest jednym przebiegiem po drzewie i nie potrzebuje miary do
terminacji (5.4).

Przy każdej z tych transformacji dokument implementacyjny ma podać **zmierzoną
liczbę trafień na materiale repo**. Dziś wynosi ona zero dla wszystkich, w tym
dla `Subtract` — `sub=0` w obu układach równań — i tak ma być napisane, zamiast
sugerować, że etap coś upraszcza.

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
  DropsNeutralConstant                   0 + x -> x  po redukcji, nie przez fabrykę
  RebuildsInBalancedShape                jednoznaczny kształt (5.5), nie tylko kolejność
  NormalizesSubtractionAndSign           a - b i a + (-b) -> ta sama postać
  IsIdempotent                           normalize(normalize(E)) sameNode normalize(E)

sesja  (5.4)
  SharesResultAcrossRootsInOneSession     drugi korzeń nie liczy od nowa
  PreservesSharingAcrossNormalization     wspólne poddrzewo zostaje JEDNYM węzłem
  KeepsNodeIdentityWhenAlreadyNormal      sameNode(wejście, wynik)

dziedzina  (6)
  PreservesDefinednessAcrossCanonicalization   1/(q+r), punkt q=1, r=-1
  KeepsOriginalDenominatorIfItBecomesZero      guard z F3.1 2.7

numeryka i integracja
  AgreesNumericallyWithinToleranceOnFk    12 równań, losowe konfiguracje, próg 1e-12
  NormalizesRealEquationSystem            end-to-end przez EquationSimplifier, oba roboty
```

Dwa testy zasługują na komentarz, bo powstały wprost z review:

`OrdersByNameNotByHash` istnieje po to, żeby podmiana porządku strukturalnego na
odciskowy (5.3) **złamała test**, a nie przeszła niezauważona. Bez niego oba
porządki dają „jakąś postać normalną" i różnica jest niewidoczna.

`RebuildsInBalancedShape` pilnuje 5.5. Test sprawdzający tylko kolejność
przeszedłby dla trzech różnych drzew, czyli dla czegoś, co **nie jest** postacią
normalną.

Liczba testowana maszynowo w proposalu implementacyjnym — z powodu opisanego
w F3.1 6.7, gdzie deklarowany gate dwa razy okazał się przewidywaniem.

---

## 12. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Etap dostarcza reguły, które nigdy nie trafiają | **realne, zmierzone** | Zakres przesunięty na AC-normalizację; przy każdej pozostałej transformacji liczba trafień podana wprost (8) |
| Globalne porównanie `divisorsOf` odrzuca poprawną kanonizację | **realne, popełnione w v1 — drugi raz w projekcie** | Warunek behawioralny zamiast strukturalnego (6.2); rozumowanie przez `zeroSet` (6.1) |
| Transformacja bez spadku miary w tablicy reguł | **realne, popełnione w v1** | Kryterium podziału: miara, nie „czy upraszcza" (8); `-(a*b)` i `Subtract` do kanonizatora |
| Kanonizacja rozbija współdzielenie DAG-a | **realne** | `CanonicalizationSession` z memo po tożsamości węzła (5.4), własny test |
| Dwa różne porządki opisane jako jeden | **realne, złapane w review** | Odcisk wycofany; jeden porządek strukturalny (5.3), test `OrdersByNameNotByHash` |
| „Postać normalna" bez zdefiniowanego kształtu drzewa | **realne, złapane w review** | Zbalansowane drzewo, deterministycznie (5.5) |
| Porządek zależny od adresów albo od `std::hash` | **realne, klasyczne** | Zakaz wprost (5.1, 5.3); test `DoesNotDependOnConstructionOrder` |
| Dryf zmiennoprzecinkowy | realne, zmierzone | 1,07e-14 wobec tolerancji 1e-12 (7); kształt zbalansowany raczej pomaga (5.5) |
| `structurallyEqual` wzięte za równość algebraiczną | **realne, po tym etapie kuszące** | Nazwa „AC-normalna" i jawne ograniczenie w komentarzu klasy (3.1) |
| Kanonizacja mianownika daje literalne zero | dziś niemożliwe, po F3.3 realne | Guard z F3.1 2.7 obowiązuje bez zmian (6.3) |

---

## 13. Rekomendacja

Przyjąć **zakres z 3** zamiast listy z roadmapy, z podziałem transformacji z 8
i warstwami z 9. Zapisać w roadmapie, że F3.2 zmienił treść — razem z powodem
i z liczbami, żeby za pięć etapów nikt nie szukał brakujących reguł.

Pytanie z v1 — warstwa wyrażeń czy warstwa równań — jest **rozstrzygnięte**:
oba, mechanizm w `symbolic/`, orkiestracja w `EquationSimplifier`, nazwa etapu
w roadmapie bez zmian (9).

Zostało jedno pytanie, tym razem drobniejsze, ale lepiej podjąć je przed kodem:

> **Czy `EquationSimplifier` powstaje już w F3.2, czy dopiero w F3.3?**

W F3.2 byłby to dziesięć linijek — pętla po równaniach i jedna sesja. Argument
za teraz: end-to-end test na prawdziwym układzie równań potrzebuje wołającego,
a bez niego F3.2 kończy się jak F3.1 — mechanizmem bez użytkownika, drugi raz
z rzędu. Argument za później: dopóki nie ma trygonometrii, kanonizacja niczego
konsumentowi nie daje poza inną kolejnością składników.

Skłaniam się do **teraz**, właśnie ze względu na test end-to-end — ale to
zwiększa zakres etapu o nową klasę w innej warstwie i wolę tego nie przesądzać
sam.

Po zatwierdzeniu — proposal implementacyjny z pełnym kodem i testami zliczonymi
maszynowo.
