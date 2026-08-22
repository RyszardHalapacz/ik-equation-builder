# Proposal: semantyka rewrite engine (F3.1), architektura

**Etap:** F3.1 · **v1 — do review** · 2026-08-22 · część 1 z 2

Pierwszy dokument Fazy 3. Poprzednik: [`../done/F2.7-facade-integration/proposal-phase2-facade-integration.md`](../done/F2.7-facade-integration/proposal-phase2-facade-integration.md)
— Faza 2 zamknięta 2026-08-21, 422/422 w obu konfiguracjach.

Ten etap nie upraszcza jeszcze niczego. Buduje **mechanizm**, w którym da się
upraszczać, nie tracąc prawdy — i to jest jedyny powód, dla którego jest osobnym
etapem przed `EquationSimplifier`.

## Prompt

> Roadmapa, F3.1 — zaprojektować bezpieczny mechanizm przekształcania wyrażeń.
>
> Najważniejsza zasada odziedziczona z Fazy 1:
>
> ```text
> nie wolno upraszczać kosztem zmiany dziedziny
> ```
>
> Przykład: `(1/q) * 0` nie może zostać bezwarunkowo zamienione na `0`.
>
> Zakres: wynik rewritingu; informacja „zmieniono / nie zmieniono"; warunki
> poprawności; iteracja do punktu stałego; limit kroków; unikanie pętli rewrite;
> zachowanie DAG lub świadome przebudowanie; testowanie równoważności numerycznej
> z uwzględnieniem dziedziny.

---

## 0. Stan wyjściowy — co warstwa symboliczna już gwarantuje

Trzy rzeczy z Fazy 1, na których cały ten etap stoi:

```text
Expression        niemutowalny uchwyt na współdzielony węzeł; kopia to
                  wskaźnik, nigdy drzewo. Graf jest acykliczny z konstrukcji.
ExpressionFactory zamknięta nad kontraktem: stałe skończone, nazwy symboli
                  niepuste, brak dzielenia przez literalne zero (R5)
structurallyEqual porównanie kształtu i wartości liści, z krótkim spięciem
                  na sameNode
```

Czwarta, mierzona: FK dla `kr640` to **281 unikalnych węzłów** i **21 882**
liczonych z krotnościami. Ta liczba wraca w 5 i przesądza jedną decyzję.

Piąta, ważniejsza niż wygląda — dzisiejsza fabryka **odmawia** zwijania `x * 0`:

```cpp
// No x * 0 -> 0 here: see the class comment.
```

Powód był słuszny i zostaje słuszny: w momencie budowania fabryka nie ma jak
orzec, czy `x` jest wszędzie określone. F3.1 daje narzędzie, które **ma** jak
(2.5).

---

## 1. Cel i zakres

**Cel:** mechanizm, w którym pojedyncze przekształcenie wyrażenia jest
**sprawdzalnie bezpieczne**, a nie tylko napisane przez kogoś ostrożnego.

**W zakresie:**

```text
pojęcie reguły i jej wyniku
maszynowy warunek poprawności (bezpieczeństwo dziedziny)
iteracja do punktu stałego, limit kroków, gwarancja terminacji
zachowanie współdzielenia DAG
raport: zmieniono / nie zmieniono / ile kroków / czy limit
strategia testowania równoważności z uwzględnieniem dziedziny
```

**Poza zakresem, świadomie:**

```text
konkretne reguły algebraiczne          -> F3.2
trygonometria                          -> F3.3
przepisywanie równań i układów         -> 7.4, komponuje się z tego
rozwiązywanie czegokolwiek             -> F3.6+
tolerancje numeryczne / szum trygonometryczny  -> 2.6, to inny problem
```

---

## 2. Bezpieczeństwo dziedziny — sedno etapu

### 2.1 W tym języku wyrażeń dziedzina ma **jedno** źródło

Dziewięć rodzajów węzłów. Osiem z nich jest **totalnych** na `R`: stała, symbol,
`+`, `−`, `*`, negacja, `sin`, `cos`. Dziewiąty nie:

```text
Divide(a, b)   nieokreślone tam, gdzie b = 0
```

Stąd definicja, która jest jednocześnie przepisem na algorytm:

```text
dom(E) = { q : każdy węzeł Divide w E ma w q niezerowy mianownik }
```

To nie jest uproszczenie na potrzeby tego etapu — to pełna prawda o tym języku.
Gdyby kiedyś doszedł `sqrt`, `log` albo `asin`, definicja rośnie o wiersz, a
reszta tego dokumentu zostaje bez zmian (10).

### 2.2 Kontrakt: dziedzina zachowana w **obie** strony

Naiwne sformułowanie brzmi „nie wolno zawężać dziedziny". Jest za słabe.
Przepisanie `A → B` musi spełniać:

```text
dom(A) = dom(B)          ORAZ          A(q) = B(q) dla wszystkich q z dom(A)
```

Obie strony równości dziedzin są potrzebne i każda z innego powodu:

```text
dom(B) ⊂ dom(A)   -> gubimy rozwiązania: konfiguracja, w której cel był
                     osiągalny, przestaje być opisana
dom(B) ⊃ dom(A)   -> WYMYŚLAMY rozwiązania: solver zwróci q, w którym
                     pierwotne ograniczenie nie miało sensu
```

Drugi przypadek jest gorszy i to on jest w promptcie. `(1/q) * 0 → 0` daje
solverowi prawo odpowiedzieć `q = 0` na pytanie, które przy `q = 0` nie było
zadane. Wynik wygląda poprawnie, przechodzi każdą kontrolę numeryczną **poza
punktem zero** i jest fałszem.

### 2.3 Warunek sprawdzalny maszynowo: zbiór dzielników

Dziedziny nie da się porównać wprost — trzeba by rozstrzygać zbiory zer wyrażeń.
Ale z 2.1 wynika warunek **konserwatywny i tani**:

```text
divisorsOf(E)  =  zbiór wyrażeń stojących w mianownikach wszystkich węzłów
                  Divide w E (bez powtórzeń, porównywane structurallyEqual)

przepisanie A -> B jest DOPUSZCZONE gdy
    divisorsOf(A)  ==  divisorsOf(B)
```

Kierunek błędu jest tu jednostronny i to jest cała jego wartość: **warunek może
odrzucić przepisanie bezpieczne, nigdy nie przepuści niebezpiecznego.** Odrzuci
np. `x/x → 1` (dzielniki `{x}` kontra `{}`) — i słusznie, bo to przepisanie
naprawdę rozszerza dziedzinę o zero.

Odrzuci też przypadki, które są bezpieczne, a wyglądają inaczej strukturalnie:
`a/(x+y)` kontra `a/(y+x)` to dwa różne dzielniki dla `structurallyEqual`. To
jest znana i zaakceptowana konserwatywność — po kanonizacji z F3.2 takich par
będzie mniej, a do tego czasu kosztem jest przepisanie **niewykonane**, nie
przepisanie błędne.

### 2.4 Kompozycyjność: dlaczego wolno przepisywać **wewnątrz** mianownika

Zarzut, który nasuwa się natychmiast: skoro dzielniki muszą się zgadzać
strukturalnie, to silnik nigdy nie wejdzie do mianownika, bo każda zmiana tam
zmienia zbiór dzielników.

Odpowiedź jest kompozycyjna, i to ona sprawia, że mechanizm w ogóle działa.
Przepisanie `d → d'` **wewnątrz** `a/d` nie jest sprawdzane warunkiem z 2.3 na
poziomie `a/d`, tylko na poziomie samego `d`. A jeśli `d → d'` przeszło własny
warunek, to znaczy, że:

```text
dom(d) = dom(d')   oraz   d(q) = d'(q) na tej dziedzinie
```

a stąd **zbiory zer `d` i `d'` są identyczne**, więc:

```text
dom(a/d) = dom(a/d')
```

Innymi słowy: warunek lokalny wymusza globalną własność, bo bezpieczeństwo
dziedziny jest przechodnie przez zagnieżdżanie. Ten akapit jest jedynym
„dowodem" w tym proposalu i całe uzasadnienie rekurencji z 4 opiera się na nim.

### 2.5 Konsekwencja, która coś **daje**: `x * 0 → 0` bywa bezpieczne

Fabryka odmawia tego zwinięcia zawsze, bo w chwili budowania patrzy lokalnie.
Silnik patrzy na całe poddrzewo, więc może orzec:

```text
x * 0 -> 0    DOPUSZCZONE gdy divisorsOf(x) jest puste
              ODRZUCONE   gdy x zawiera jakikolwiek Divide
```

`q1 * 0 → 0` — wolno. `(1/q) * 0 → 0` — nie wolno. Ta sama reguła, dwie różne
odpowiedzi, obie wyprowadzone, nie zgadnięte.

To jest konkretny zysk z tego etapu i warto go nazwać: F3.1 nie „obchodzi"
ostrożności z Fazy 1, tylko **zamienia zakaz globalny na warunek lokalny**.
Zakaz w fabryce zostaje bez zmian — fabryka nadal nie ma czym orzekać.

### 2.6 Czego ten warunek **nie** dotyczy

Szum trygonometryczny (`sin(π) = 1.22e-16`) nie jest problemem dziedziny, tylko
liczbowym, i F3.1 go **nie rozwiązuje**. Przepisanie `x + 1.22e-16 → x` jest
niedopuszczalne nie dlatego, że rusza dziedzinę, ale dlatego, że **nie jest
równością**. Tolerancja algebraiczna to decyzja z F3.2/F3.3 i zostaje tam.

---

## 3. Reguła

```cpp
// A rewrite rule: given an expression, either produce a replacement or decline.
//
// nullopt means "does not apply", NOT "applies and changes nothing". A rule
// that returns its input unchanged would spin the fixed-point loop forever, so
// the engine treats an unchanged result as a defect (4.3).
//
// A rule NEVER checks the domain condition itself. That check belongs to the
// engine and is applied to every rule's output without exception -- the point of
// F3.1 is that safety is a property of the mechanism, not of each author's care.
struct RewriteRule
{
    std::string_view name;                                 // for diagnostics
    std::optional<Expression> (*apply)(const Expression&, const ExpressionFactory&);
};
```

**Reguła jest danymi, nie klasą.** Trzy powody: zestaw reguł staje się `span`em
przekazywanym do silnika, testy mogą uruchomić **jedną** regułę bez reszty, a
nazwa w raporcie mówi, co zadziałało — bez tego debugowanie serii przepisań to
zgadywanie.

**Reguła nie sprawdza dziedziny.** To jest najważniejsze zdanie tej sekcji.
Gdyby sprawdzała, bezpieczeństwo byłoby własnością *dyscypliny autora reguł*, a
projekt ma już precedens, który mówi, jak to się kończy i jak się to naprawia:
R5 zamknęło `ExpressionFactory` nad kontraktem skończoności zamiast liczyć na to,
że każdy wołający pamięta.

---

## 4. Silnik

### 4.1 Kształt

```text
rewrite(expression, rules, limit)
    1. przepisz rekurencyjnie dzieci        (z memoizacją, 5)
    2. spróbuj reguł na węźle bieżącym, po kolei
    3. każde trafienie sprawdź warunkiem z 2.3
    4. jeśli coś się zmieniło, wróć do 1 dla tego węzła
    5. zatrzymaj się na punkcie stałym albo na limicie
```

Kolejność „najpierw dzieci" (innermost) jest wyborem, nie przypadkiem: reguła
działająca na węźle bieżącym widzi wtedy poddrzewa już uproszczone, więc wzorce
z F3.2 mogą być pisane pod postać kanoniczną, a nie pod wszystkie warianty
wejścia.

### 4.2 Terminacja — trzy niezależne zabezpieczenia

Roadmapa wymienia „limit kroków" i „unikanie pętli rewrite" jako dwie pozycje.
To są trzy różne mechanizmy i warto je rozdzielić, bo chronią przed czym innym:

```text
a) kontrakt reguły     reguła MUSI zmniejszać miarę (4.3)
b) limit kroków        twardy budżet, niezależny od (a)
c) raport o limicie    wynik mówi, że nie osiągnięto punktu stałego
```

`(a)` jest właściwym zabezpieczeniem: przy malejącej mierze pętla `A → B → A`
jest niemożliwa, bo wymagałaby, żeby miara zmalała dwa razy i wróciła.
`(b)` istnieje mimo `(a)`, bo `(a)` jest kontraktem, a kontrakty bywają łamane
przez regułę napisaną jutro; budżet zamienia nieskończoną pętlę w **zdiagnozowany
błąd**. `(c)` istnieje, bo cicho zwrócone „prawie uproszczone" wyrażenie jest
gorsze niż jawne „nie zbiegło" — konsument z F3.2 musi wiedzieć, że wynik nie
jest kanoniczny.

Alternatywa dla `(a)` — pamiętanie odwiedzonych postaci i wykrywanie cyklu —
jest **odrzucona**: wymaga haszowania struktury każdego pośredniego wyrażenia,
kosztuje pamięć proporcjonalną do liczby kroków i wykrywa problem dopiero, gdy
już wystąpi. Miara wyklucza go z góry.

### 4.3 Miara

```text
measure(E) = liczba węzłów liczona Z KROTNOŚCIAMI
```

Z krotnościami, nie unikalnych — bo miara unikalnych węzłów potrafi **wzrosnąć**
przy przepisaniu, które zmniejsza wyrażenie (rozbicie współdzielenia), i wtedy
kontrakt reguły byłby nie do spełnienia dla poprawnych reguł.

Kontrakt: `measure(B) < measure(A)`. Reguły czysto porządkujące (zamiana
argumentów przemiennych, kanonizacja kolejności) **nie spełniają** go i to jest
świadome — w F3.2 wchodzą jako osobny przebieg z własną, leksykograficzną miarą,
albo jako etap normalizacji przed przepisywaniem. F3.1 nie udaje, że rozwiąże to
z góry dla reguł, których jeszcze nie widział; zapisuje natomiast, że **silnik
musi tę różnicę widzieć**, i zostawia na to miejsce w API (7.3).

Sprawdzenie miary jest `assert`em, nie kontrolą runtime: złamanie kontraktu to
błąd programisty, a limit kroków i tak zamienia go w skończone zakończenie.
Precedens i uzasadnienie tego kształtu — `assert` bez wyłuskania — pochodzą z
F2.5Q.

---

## 5. Zachowanie DAG — nie do negocjacji

Naiwna rekurencja przepisująca odwiedza **każde wystąpienie** poddrzewa. Dla FK
`kr640` to różnica między 281 a 21 882 odwiedzinami, czyli **77×**, i to przy
najmniejszym z dwóch robotów w repo.

Silnik memoizuje po **tożsamości węzła**:

```cpp
std::unordered_map<const ExpressionNode*, Expression> rewritten;
```

Dokładnie ten sam mechanizm, którym `ExpressionEvaluator` liczy wartości, i z
tego samego powodu. Skutki są dwa i oba są wymagane:

```text
złożoność   liniowa w liczbie UNIKALNYCH węzłów, nie w rozmiarze rozwinięcia
struktura   współdzielenie przeżywa przepisanie -- wynik jest DAG-iem,
            nie drzewem
```

Drugi skutek jest ważniejszy dla F3.2: gdyby wynik był drzewem, każde kolejne
przejście symplifikatora startowałoby od materiału 77 razy większego, a to
kaskaduje przez cały pozostały etap Fazy 3.

Klucz jest surowym wskaźnikiem i to jest bezpieczne dokładnie tak długo, jak
żyje wejściowe wyrażenie — a żyje, bo `Expression` trzyma `shared_ptr`, a mapa
istnieje wyłącznie w obrębie jednego wywołania.

---

## 6. Wynik

```cpp
struct RewriteResult
{
    Expression expression;      // punkt stały albo stan po wyczerpaniu limitu
    bool changed{};             // czy cokolwiek zadziałało
    std::size_t steps{};        // ile przepisań wykonano
    bool reachedLimit{};        // czy zatrzymał nas budżet, a nie punkt stały
};
```

`changed` jest w promptcie wprost i ma konkretnego konsumenta: `EquationSimplifier`
z F3.2 będzie puszczał kilka przebiegów różnymi zestawami reguł i musi wiedzieć,
kiedy przestać.

`reachedLimit` **nie jest** błędem i dlatego nie jest `expected`. Wyrażenie po
wyczerpaniu limitu jest w pełni poprawne — równoważne wejściu i o tej samej
dziedzinie — tylko nie kanoniczne. Zwracanie go jako błędu zmusiłoby caller'a do
wyrzucenia poprawnego wyniku.

Nie ma tu natomiast **listy zastosowanych reguł**, choć kusi. Powód: byłaby to
struktura rosnąca ze `steps`, potrzebna wyłącznie do diagnostyki, a diagnostykę
da się mieć taniej — nazwa reguły trafia do komunikatu asercji przy złamaniu
miary (4.3). Jeśli F3.2 pokaże, że ślad jest naprawdę potrzebny, dołoży go z
konkretnym uzasadnieniem.

---

## 7. API

### 7.1 Nagłówki

```text
symbolic/RewriteRule.hpp     RewriteRule, RewriteResult
symbolic/RewriteEngine.hpp   rewrite(), divisorsOf(), preservesDomain()
symbolic/RewriteEngine.cpp   silnik
```

W `symbolic/`, nie w `builders/`: to jest narzędzie warstwy wyrażeń, a nie
budowniczy niczego. `EquationSimplifier` z F3.2 stanie piętro wyżej i będzie z
tego korzystał.

### 7.2 Funkcje

```cpp
// The domain condition from 2.3, exposed because F3.2 will want to assert it in
// its own tests and because a rule author must be able to read it.
[[nodiscard]] std::vector<Expression> divisorsOf(const Expression& expression);

[[nodiscard]] bool preservesDomain(const Expression& before, const Expression& after);

// Rewrites to a fixed point, or until the step budget runs out.
//
// Every rule application is checked with preservesDomain() and DISCARDED if it
// fails: a rule that would change the domain simply does not fire, and the
// engine moves on to the next one. Not an error -- rules are allowed to be
// written for the general case and refused in the specific one. That is exactly
// what happens to x * 0 -> 0 when x contains a division.
[[nodiscard]] RewriteResult rewrite(const Expression& expression,
                                    std::span<const RewriteRule> rules,
                                    const ExpressionFactory& factory,
                                    std::size_t stepLimit = kDefaultRewriteStepLimit);
```

### 7.3 Miejsce zostawione dla F3.2

Reguły porządkujące (4.3) nie zmniejszają miary. Zamiast wpychać je teraz w
model, którego jeszcze nie znamy, F3.1 zostawia **jawny szew**: `rewrite` bierze
`span` reguł, więc F3.2 może wywołać go dwa razy z dwoma zestawami i sama
zdecydować o kolejności. Gdyby okazało się, że potrzebny jest jeden przebieg z
dwiema miarami — to zmiana w silniku, którą F3.2 uzasadni na konkretach.

### 7.4 Równania i układy

F3.1 **nie** dotyka `Equation` ani `IkEquationSystem`. Kompozycja jest
jednolinijkowa i należy do F3.2:

```text
przepisz lhs, przepisz rhs, zachowaj source
```

i jest bezpieczna, bo `Equation` to `lhs = rhs` przechowywane tak, jak podane —
przepisanie każdej strony osobno nie rusza granicy między stroną robota a stroną
zadania, na której stoi cały model z F2.3.

---

## 8. Testy

```text
dziedzina -- rdzeń etapu
  CollectsDivisorsFromNestedExpression      dzielniki z zagnieżdżeń, bez powtórzeń
  RejectsRewriteThatWidensDomain            (1/q)*0 -> 0 ODRZUCONE
  AllowsSameRewriteWhenNoDivisionPresent    q1*0 -> 0 DOPUSZCZONE
  RejectsRewriteThatNarrowsDomain           reguła wprowadzająca nowy dzielnik
  RejectsCancellationOfSelfDivision         x/x -> 1 odrzucone
  AllowsRewriteInsideDenominator            kompozycyjność z 2.4

silnik
  ReportsChangedWhenARuleFires
  ReportsUnchangedOnFixedPoint
  StopsAtStepLimitAndSaysSo                 reachedLimit == true, wynik poprawny
  AppliesRulesInnermostFirst                obserwowalne przez regułę licznikową
  DiscardsUnsafeRuleAndContinues            reguła odrzucona nie blokuje kolejnych

DAG
  PreservesSharingAcrossRewrite             licznik unikalnych węzłów po
  RewritesSharedSubtreeOnce                 reguła liczy wywołania: 1, nie 77

równoważność numeryczna Z DZIEDZINĄ
  EquivalentOnSampledDomain                 patrz niżej
  RewrittenKr640FkMatchesOriginal           end-to-end na prawdziwym FK
```

**`EquivalentOnSampledDomain`** jest testem, dla którego ten etap istnieje, i
jego kształt jest nietypowy, więc opis wprost:

```text
1. weź wyrażenie z dzieleniem, np. a/(q1 - 0.5)
2. przepisz zestawem reguł
3. wylosuj N konfiguracji ze STAŁYM ziarnem
4. dla każdej: jeśli którykolwiek dzielnik zeruje się blisko zera -> POMIŃ punkt
5. dla pozostałych: evaluate(przed) == evaluate(po) w granicach 1e-12
6. osobno: divisorsOf(przed) == divisorsOf(po)
```

Krok 4 jest tym, co odróżnia ten test od zwykłego „porównaj wartości": bez niego
test albo losowo trafiałby w biegun i wybuchał, albo — gorzej — **przechodziłby
mimo rozszerzenia dziedziny**, bo poza zerem obie postacie są równe. Sama
równoważność numeryczna **nie potrafi** wykryć błędu, przed którym chroni ten
etap; dlatego krok 6 jest osobny i obowiązkowy.

**`RewritesSharedSubtreeOnce`** liczy wywołania reguły na współdzielonym
poddrzewie. Bez memoizacji z 5 liczba jest równa krotności wystąpień; z nią jest
równa jedynce. To jedyny test, który pilnuje własności wydajnościowej — i pilnuje
jej przez zachowanie, nie przez pomiar czasu.

---

## 9. Plan zmian w plikach

```text
NOWE   symbolic/RewriteRule.hpp        RewriteRule, RewriteResult, limit domyślny
       symbolic/RewriteEngine.hpp      divisorsOf, preservesDomain, rewrite
       symbolic/RewriteEngine.cpp      silnik, memoizacja, kontrola miary
       tests/test_rewrite_engine.cpp   ~15 testów

ZMIENIONE  CMakeLists.txt              +1 źródło
           tests/CMakeLists.txt        +1 plik testowy

BEZ ZMIAN  Expression, ExpressionFactory, ExpressionEvaluator, SymbolicMatrix,
           cały model, ConstraintBuilder, fasada, loader
```

Ostatni wiersz jest sprawdzianem: F3.1 **dokłada warstwę**, nie modyfikuje
istniejącej. Gdyby okazało się, że silnik potrzebuje zmiany w `Expression` albo
w fabryce, byłby to sygnał, że któreś założenie z 2 jest nie do utrzymania — i
ważniejszy niż sam etap.

---

## 10. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Reguła zmienia dziedzinę | **realne, to jest powód istnienia etapu** | Warunek sprawdzany przez **silnik** przy każdym trafieniu, nie przez autora reguły (3) |
| Warunek strukturalny odrzuca bezpieczne przepisania | **realne, świadome** | Kierunek błędu jest jednostronny (2.3); kosztem jest przepisanie niewykonane, nie błędne |
| Pętla rewrite | realne | Miara malejąca jako kontrakt + limit jako siatka + raport (4.2) |
| Rozbicie współdzielenia | **realne, policzone** | Memoizacja po tożsamości węzła; 281 kontra 21 882 dla `kr640` (5) |
| Test równoważności przepuszcza rozszerzenie dziedziny | **realne i nieoczywiste** | Osobne sprawdzenie `divisorsOf`, krok 6 w 8 — bo poza biegunem obie postacie są równe |
| Nowy rodzaj węzła zmienia definicję dziedziny | niskie | 2.1: definicja rośnie o wiersz; `divisorsOf` jest jawnym `std::visit`, więc build stanie |
| Reguły porządkujące nie mieszczą się w mierze | świadome | Jawny szew: `span` reguł i dwa przebiegi po stronie F3.2 (7.3) |

---

## 11. Otwarte pytania do review

**11.1 Czy `preservesDomain` ma porównywać zbiory, czy multizbiory dzielników?**
Rekomendacja: **zbiory**. `a/x + b/x` ma jeden dzielnik występujący dwa razy, a
dla dziedziny liczy się tylko to, że `x ≠ 0`. Multizbiór odrzucałby przepisania
łączące takie składniki bez żadnego zysku dla bezpieczeństwa.

**11.2 Domyślny limit kroków.** Proponuję `kDefaultRewriteStepLimit = 10 000` —
rząd wielkości powyżej liczby unikalnych węzłów największego FK w repo (281),
więc żadne realne wyrażenie go nie dotknie, a pętla zatrzyma się w milisekundach.
Liczba jest **arbitralna i taka zostaje**: to siatka bezpieczeństwa, nie próg
numeryczny, i nie należy jej „mierzyć", żeby nie sugerować, że coś znaczy.

**11.3 Czy `rewrite` ma przyjmować `ExpressionFactory` argumentem?** Rekomendacja:
tak. Fabryka jest bezstanowa, ale to ona niesie kontrakty (skończoność, brak
dzielenia przez zero), więc reguła budująca nowe węzły musi ją dostać, a nie
tworzyć własną. Precedens: `JointTransformBuilder` i `ForwardKinematicsBuilder`
biorą ją tak samo.

---

## 12. Rekomendacja

Przyjąć definicję dziedziny z 2.1, kontrakt dwustronny z 2.2, warunek zbioru
dzielników z 2.3 wraz z argumentem kompozycyjnym z 2.4, oraz podział
odpowiedzialności z 3: **reguła proponuje, silnik dopuszcza**.

Po zatwierdzeniu — proposal implementacyjny z pełnym kodem trzech nowych plików
i około piętnastoma testami, gate `437/437` (422 + 15).

Po F3.1 wchodzi **F3.2** — pierwsze prawdziwe reguły: kanonizacja algebraiczna,
czyli `x + 0`, `x * 1`, zwijanie stałych i porządek argumentów. Dopiero tam
okaże się, czy szew z 7.3 był potrzebny.
