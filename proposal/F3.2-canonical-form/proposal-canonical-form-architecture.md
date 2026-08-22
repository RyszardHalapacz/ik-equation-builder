# Proposal: postać kanoniczna — F3.2, architektura

**Etap:** F3.2 · **v1 — do review** · 2026-08-22 · część 1 z 2

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
F3.4  pattern detector              musi dopasować kształt równania do wzorca
F3.8  solver pozycji 6R             musi porównać strukturę z katalogiem przypadków
```

Żaden z nich nie działa na materiale, w którym te same wyrażenie ma wiele
zapisów. **Postać kanoniczna jest przesłanką tamtych etapów, a nie ozdobą tego.**

Proponowany zakres F3.2:

```text
1. spłaszczenie maksymalnych łańcuchów Add i Multiply
2. uporządkowanie składników/czynników porządkiem TOTALNYM i deterministycznym
3. odbudowa łańcucha w tej kolejności
4. zwinięcie tego, co po uporządkowaniu stało się zwijalne  (fabryka zrobi to sama)
5. mały zestaw reguł silnika F3.1 -- jako ubezpieczenie, z jawnym „dziś 0 trafień"
```

Punkt 4 jest darmowy i wart odnotowania: gdy porządek zbierze stałe obok siebie,
`add(c1, c2)` zwinie je **przy odbudowie**, bez żadnej reguły. Zmierzony jeden
przypadek w `kr4_r600` (sumowy łańcuch z dwiema stałymi) zniknie sam.

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
canonicalise(E)   ->   rewrite(E)   ->   canonicalise(E)
```

Pierwszy przebieg daje regułom przewidywalny kształt do dopasowania, drugi
porządkuje to, co reguły zbudowały. Alternatywa — przeplatanie do punktu stałego
obu mechanizmów — jest **odrzucona** w tym etapie: terminacja takiej pętli nie
wynika już z jednej miary i wymagałaby własnego dowodu.

---

## 5. Porządek totalny — jedyna trudna decyzja tego etapu

Wymagania, wszystkie konieczne:

```text
totalny          dowolne dwa wyrażenia dają się porównać
deterministyczny ten sam wynik przy każdym uruchomieniu i na każdej platformie
strukturalny     zgodny ze structurallyEqual: equal <=> ani mniejsze, ani większe
tani             sortowanie 19 składników nie może kosztować O(rozmiar) na porównanie
```

Trzeci punkt jest tym, na czym taki porządek zwykle się wykłada, a czwarty tym,
o czym się zapomina.

### 5.1 Pułapka, którą trzeba nazwać

**Porządek nie może dotykać adresów węzłów.** Kuszące, bo `Expression` to
`shared_ptr` i `<` na wskaźnikach jest darmowe — i całkowicie błędne: kolejność
zależałaby od alokatora, czyli od uruchomienia. Te same równania dałyby dwie
różne postacie kanoniczne w dwóch procesach, a `RewrittenFkMatchesGoldenForm`
byłby testem losowym.

To ta sama klasa błędu, którą F3.1 zamknęło kontraktem determinizmu (v3, 1.3) —
i warto, żeby oba etapy odwoływały się do tej samej zasady.

### 5.2 Proponowany porządek

```text
1. RANGA RODZAJU węzła, ustalona jawnie:
       Constant < Symbol < Sin < Cos < Negate < Multiply < Divide < Add < Subtract

2. przy równej randze -- porównanie właściwe dla rodzaju:
       Constant     po wartości  (<, exact)
       Symbol       po nazwie    (leksykograficznie)
       jednoargumentowe   rekurencyjnie po operandzie
       dwuargumentowe     rekurencyjnie: lhs, potem rhs
```

Stałe **przed** symbolami, żeby po uporządkowaniu wylądowały obok siebie i dały
fabryce szansę na zwinięcie ich przy odbudowie (3, punkt 4).

### 5.3 Koszt i klucz sortowania

Naiwnie porównanie dwóch składników to `O(rozmiar)`, a sortowanie łańcucha
`n` składników to `O(n log n)` porównań — dla sum FK z materiałem po kilkadziesiąt
węzłów to jest do przyjęcia, ale zależność jest zła i rośnie z etapami.

Proponuję **klucz porządkowy liczony raz na unikalny węzeł**, memoizowany po
tożsamości węzła — dokładnie tym wzorcem, którym F3.1 liczy strażników dziedziny
(5.4 tamtego dokumentu):

```text
key(Constant c)   = (0, c)
key(Symbol s)     = (1, hash(s))
key(op(a,...))    = (rangaRodzaju, mieszanie(key dzieci))
```

Porównanie zaczyna od klucza; **przy równych kluczach schodzi do porównania
strukturalnego**, więc kolizja mieszania kosztuje czas, a nie poprawność.
To rozstrzygnięcie warto podjąć teraz, bo determinizm klucza (żadnych adresów,
żadnego `std::hash` na wskaźniku) jest jego częścią, a nie optymalizacją.

---

## 6. Bezpieczeństwo dziedziny przy przestawianiu

Krótko, bo wychodzi z definicji z F3.1 §2.1:

```text
dom(E) zależy WYŁĄCZNIE od węzłów Divide
spłaszczenie + sortowanie + odbudowa NIE dodaje ani nie usuwa żadnego Divide
=> multizbiór dzielników jest ten sam
=> divisorsOf przed == divisorsOf po, jako zbiory
=> dziedzina zachowana
```

Jest jednak jeden przypadek brzegowy i trzeba go obsłużyć **tym samym
mechanizmem, co w F3.1**: jeśli kanonizacja mianownika zwinie go do literalnego
zera, `ExpressionFactory::divide` przewróci `assert`. Dziś nie może się to
zdarzyć — `add` nie skraca składników, a zwija tylko stałe — ale to jest
własność dzisiejszej fabryki, nie twierdzenie. **Representability guard z F3.1
2.7 obowiązuje więc także tutaj**: mianownik, który po kanonizacji stał się
zerem, zostaje w postaci oryginalnej.

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

## 8. Co zostaje z listy roadmapy

Nie proponuję jej wyrzucenia — proponuję **przeniesienie i opisanie prawdą**:

```text
ZOSTAJE w F3.2, jako zestaw regul silnika F3.1:
    -(-x) -> x
    -(a*b) -> (-a)*b        normalizacja znaku, potrzebna dla porzadku z 5
    x - y  -> x + (-y)      wprowadza kanoniczna postac sumy    <-- UWAGA, 9

ODPADA, bo fabryka robi to przy budowie i regula nie ma czego dopasowac:
    x + 0, x - 0, x * 1, x / 1, constant folding
```

Przy każdej z trzech, które zostają, dokument implementacyjny ma podać
**zmierzoną liczbę trafień na materiale repo**. Dziś dla wszystkich trzech jest
to zero — i tak ma być napisane, zamiast sugerować, że etap coś upraszcza.

---

## 9. Rzecz, którą trzeba rozstrzygnąć przed implementacją

`x - y → x + (-y)` **zwiększa miarę**:

```text
measure(x - y)      = 3
measure(x + (-y))   = 4
```

Czyli łamie kontrakt reguły silnika F3.1 tak samo, jak reguły porządkujące.
A jednocześnie bez niej porządek kanoniczny sum jest niepełny: `a - b` i
`(-b) + a` są tym samym wyrażeniem o dwóch nieporównywalnych zapisach.

Trzy wyjścia, moja rekomendacja na końcu:

```text
A. zostawić Subtract jako osobny rodzaj i nie kanonizować go
   -- tanie, ale zostawia dwa zapisy tego samego

B. przenieść x-y -> x+(-y) do przebiegu kanonizujacego, nie do regul
   -- spojne z 4: kanonizacja to osobny mechanizm, nie podlega mierze

C. zmienic miare na cos innego niz liczba wezlow
   -- rusza fundament F3.1 dzien po jego zatwierdzeniu
```

**Rekomendacja: B.** Kanonizacja i tak jest osobnym przebiegiem o własnej
terminacji (jeden przebieg po drzewie, bez punktu stałego), więc mieści to
naturalnie i nie dotyka niczego z F3.1. Zmierzone `sub=0` w obu układach równań
znaczy przy tym, że dziś ta reguła też nie ma na czym pracować — ale pojawi się
natychmiast, gdy F3.3 zacznie przenosić składniki między stronami równania.

---

## 10. Zakres — czego w F3.2 nie ma

```text
NIE MA   zbierania wyrazów podobnych (2x + 3x -> 5x)
NIE MA   rozdzielności (a*(b+c) -> a*b + a*c ani w drugą stronę)
NIE MA   tożsamości trygonometrycznych  -- to F3.3
NIE MA   wpięcia w ConstraintBuilder ani w fasadę
```

Trzy pierwsze wymagają decyzji, **która postać jest kanoniczna** — a to pytanie
ma sens dopiero, gdy wiadomo, co solver z F3.8 chce dostać. Wpięcie zostawiam,
bo dopóki nie ma reguł trygonometrycznych, kanonizacja niczego konsumentowi nie
daje poza inną kolejnością składników.

---

## 11. Testy — szkic

```text
porządek
  IsTotalOnMixedNodeKinds                dowolna para porównywalna
  IsAntisymmetricAndTransitive           na wygenerowanym zbiorze wyrażeń
  AgreesWithStructuralEquality           equal <=> ani <, ani >
  DoesNotDependOnConstructionOrder       to samo wyrażenie zbudowane dwiema drogami
  IsStableAcrossProcessRuns              klucz nie zależy od adresów  (patrz 5.1)

kanonizacja
  FlattensNestedAssociativeChains        ((a+b)+c) i (a+(b+c)) -> ta sama postać
  SortsProductsAndSums
  FoldsConstantsBroughtTogetherByOrder   darmowy zysk z 3, punkt 4
  IsIdempotent                           canonicalise(canonicalise(E)) == canonicalise(E)
  KeepsOriginalDenominatorIfItBecomesZero        guard z F3.1 2.7
  PreservesDomainGuardsAsASet

numeryka i integracja
  AgreesNumericallyWithinToleranceOnFk    12 równań, losowe konfiguracje, próg 1e-12
  CanonicalisesRealEquationSystem         end-to-end, oba roboty
```

Około czternastu testów; dokładna liczba w proposalu implementacyjnym, po
zliczeniu **maszynowym**, nie na piechotę — z powodu opisanego w F3.1 6.7.

---

## 12. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Etap dostarcza reguły, które nigdy nie trafiają | **realne, zmierzone** | Zakres przesunięty na kanonizację; przy każdej pozostałej regule liczba trafień podana wprost (8) |
| Porządek zależny od adresów | **realne, klasyczne** | Zakaz wprost w 5.1; test `DoesNotDependOnConstructionOrder` |
| Dryf zmiennoprzecinkowy | realne, zmierzone | 1,07e-14 wobec tolerancji 1e-12 (7); test numeryczny w zestawie |
| Reguła porządkująca w tablicy reguł zapętla silnik | **realne** | Kanonizacja jest osobnym przebiegiem, nie regułą (4) |
| `x-y → x+(-y)` łamie kontrakt miary | **realne** | Wariant B z 9: do przebiegu kanonizującego, nie do reguł |
| Kanonizacja mianownika daje literalne zero | mało prawdopodobne, niezamykane | Representability guard z F3.1 2.7 obowiązuje też tutaj (6) |
| Koszt porównań przy sortowaniu | świadome | Klucz memoizowany po tożsamości węzła, wzorzec z F3.1 5.4 (5.3) |

---

## 13. Rekomendacja i pytanie do ciebie

Przyjąć **zakres z 3** zamiast listy z roadmapy, z wariantem **B** z 9, i zapisać
w roadmapie, że F3.2 zmienił treść — razem z powodem, żeby za pięć etapów nikt
nie szukał brakujących reguł.

Pytanie, na które potrzebuję twojej decyzji, bo jest projektowe, nie techniczne:

> **Czy postać kanoniczna ma być własnością warstwy wyrażeń, czy warstwy
> równań?** Innymi słowy: czy `canonicalise(Expression)` w `symbolic/`, czy
> `EquationSimplifier` operujący na `IkEquationSystem`?

Roadmapa mówi „`EquationSimplifier`", ale wszystko, co ten etap robi, jest
własnością pojedynczego wyrażenia. Skłaniam się do `symbolic/`, z cienkim
`EquationSimplifier` jako konsumentem w F3.3 — ale to zmienia nazwę etapu
w roadmapie i wolę tego nie robić bez ciebie.

Po zatwierdzeniu — proposal implementacyjny z pełnym kodem i testami
zliczonymi maszynowo.
