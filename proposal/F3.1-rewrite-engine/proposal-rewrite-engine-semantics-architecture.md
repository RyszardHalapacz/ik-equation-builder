# Proposal: semantyka rewrite engine (F3.1), architektura

**Etap:** F3.1 · **v2 — do review** · 2026-08-22 · część 1 z 2

> **v2, po review.** Rdzeń utrzymany — `divisorsOf`, przejście innermost, malejąca
> miara i podział „reguła proponuje, silnik dopuszcza" zostają. Cztery blockery,
> wszystkie przyjęte, i wszystkie sprowadzają się do jednego: **v1 obiecywała
> więcej, niż silnik potrafi sprawdzić.**
>
> 1. **„Sprawdzalnie bezpieczne" było za mocne.** Silnik testuje warunek na
>    dzielnikach, nie równoważność algebraiczną — reguła `x + 0 -> 42` przeszłaby
>    ten test. Granice gwarancji rozpisane wprost (2.2, 3.2).
> 2. **§2.4 przeczyło §8 i publicznemu `preservesDomain`.** Dla `a/-(-q) -> a/q`
>    dzielniki różnią się strukturalnie, więc globalne porównanie **odrzuciłoby**
>    poprawne przepisanie. Warunek jest **wyłącznie lokalny** i nazywa się teraz
>    `hasSameDomainGuards`; globalna własność wynika indukcyjnie, nie z porównania
>    (2.3, 2.4). Test dziedziny w 8 przepisany na **behawioralny**.
> 3. **Uproszczenie mianownika może złamać kontrakt fabryki.** `a/(q-q)` z regułą
>    `q - q -> 0` prowadzi do `divide(a, 0)`, a tam stoi
>    `assert(!isZero(rhs) && "division by literal zero")` — w Release UB, w
>    `build-werror` abort. Dochodzi **representability guard** (2.7).
> 4. **Surowy `ExpressionNode*` jako klucz memoizacji.** Pośrednie wyrażenia
>    umierają, a adres bywa użyty ponownie. Projekt rozwiązał to już w
>    `ExpressionEvaluator`, kluczując cache przez `Expression` — F3.1 robi tak
>    samo (5.2).
>
> Poza tym: poprawiona derywacja limitu kroków (11.2), wycofana obietnica „jeden
> wiersz" przy `sqrt`/`log` (2.1), wycofany rzekomy szew dla reguł porządkujących
> (7.3) i rozstrzygnięte, czy silnik jest sesją (5.3).

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

## 0. Stan wyjściowy

Cztery rzeczy z Fazy 1, na których stoi cały ten etap:

```text
Expression        niemutowalny uchwyt na współdzielony węzeł; kopia to wskaźnik,
                  nigdy drzewo. Graf acykliczny z konstrukcji.
ExpressionFactory zamknięta nad kontraktem: stałe skończone, nazwy niepuste,
                  assert(!isZero(rhs)) w divide()  <-- wraca w 2.7
ExpressionEvaluator  cache kluczowany przez Expression z ExpressionIdentityHash,
                  NIE przez surowy wskaźnik      <-- wraca w 5.2
FK kr640          281 unikalnych węzłów, 21 882 z krotnościami  <-- wraca w 5.1 i 11.2
```

Piąta rzecz, ważniejsza niż wygląda — fabryka **odmawia** zwijania `x * 0`:

```cpp
// No x * 0 -> 0 here: see the class comment.
```

Powód był słuszny i zostaje: w chwili budowania fabryka nie ma jak orzec, czy `x`
jest wszędzie określone. F3.1 daje narzędzie, które **ma** jak (2.6).

---

## 1. Cel i zakres

**Cel:** mechanizm, w którym przekształcenie wyrażenia nie może po cichu zmienić
**dziedziny** ani zapętlić się w nieskończoność — a wszystko, czego mechanizm nie
gwarantuje, jest nazwane wprost.

**W zakresie:**

```text
pojęcie reguły i jej wyniku
lokalny warunek zachowania ograniczeń dziedziny, sprawdzany przez silnik
representability guard: przepisanie nie może stworzyć węzła, którego
    fabryka nie zbuduje
iteracja do punktu stałego, limit kroków, gwarancja terminacji
zachowanie współdzielenia DAG, także między wieloma korzeniami
raport: zmieniono / ile kroków / czy limit
strategia testowania równoważności z uwzględnieniem dziedziny
```

**Poza zakresem, świadomie:**

```text
konkretne reguły algebraiczne          -> F3.2
normalizacja kolejności argumentów     -> 7.3, POZA tym silnikiem
trygonometria i tolerancje numeryczne  -> F3.3
przepisywanie równań i układów         -> 7.4, komponuje się z tego
dowodzenie równoważności algebraicznej -> 2.2, nie jest zadaniem silnika
```

---

## 2. Dziedzina

### 2.1 W tym języku wyrażeń ograniczenia dziedziny mają **jedno** źródło

Dziewięć rodzajów węzłów. Osiem jest **totalnych** na `R`: stała, symbol, `+`,
`−`, `*`, negacja, `sin`, `cos`. Dziewiąty nie:

```text
Divide(a, b)   nieokreślone tam, gdzie b = 0
```

Stąd, dla **obecnego** języka:

```text
dom(E) = { q : każdy węzeł Divide w E ma w q niezerowy mianownik }
```

**Ograniczenie tego modelu, poprawione w v2.** v1 twierdziła, że po dodaniu
`sqrt`, `log` czy `asin` „definicja rośnie o wiersz, a reszta zostaje". To
nieprawda: `sqrt(x)` wnosi warunek `x ≥ 0`, `log(x)` — `x > 0`, czyli warunki,
których **nie da się wyrazić zbiorem mianowników**. Cały mechanizm `divisorsOf`
jest kompletny **dokładnie dla dzisiejszych dziewięciu typów węzłów**; nowy
operator częściowy wymaga uogólnienia modelu na *domain guards* — coś w rodzaju
listy predykatów, a nie listy wyrażeń. F3.1 tego **nie** buduje i nie udaje, że
zbuduje: `divisorsOf` jest jawnym `std::visit`, więc dołożenie dziesiątego typu
zatrzyma build i zmusi do decyzji.

### 2.2 Co silnik gwarantuje, a czego **nie**

To jest sekcja, której v1 nie miała, i brak którejtej sprawiał, że dokument
obiecywał za dużo.

Poprawne przepisanie `A → B` spełnia dwa warunki:

```text
(D)  dom(A) = dom(B)
(E)  A(q) = B(q) dla wszystkich q z dom(A)
```

Silnik sprawdza maszynowo **tylko okrojoną wersję (D)** — warunek składniowy z
2.3 — plus terminację. Warunku **(E) nie sprawdza i nie może**: rozstrzyganie
równoważności wyrażeń symbolicznych to zadanie, którego ten projekt nie ma i
mieć nie będzie.

```text
gwarantuje silnik           reguła nie zmieni składniowych ograniczeń dziedziny
                            przepisanie nie stworzy węzła nie do zbudowania (2.7)
                            proces się zatrzyma (4.2)

gwarantuje AUTOR reguły     równoważność algebraiczna (E)
                            egzekwowana testami TEJ reguły, w F3.2
```

Konkretnie: reguła `x + 0 → 42` przechodzi warunek dziedziny (żadnych
dzielników po obu stronach), zmniejsza miarę i **zostanie zastosowana**. Jest
matematycznie fałszywa i żaden mechanizm z tego etapu tego nie wykryje. Wykryją
to testy reguły — i dlatego 8 mówi o testowaniu *reguł*, a nie tylko silnika.

Zwrot „sprawdzalnie bezpieczne" z v1 zostaje **wycofany**. Właściwe zdanie brzmi:
*silnik odbiera autorowi reguły możliwość popsucia dziedziny, nie możliwość
pomylenia się w algebrze.*

To i tak jest dużo. Błąd dziedziny jest cichy, przechodzi testy numeryczne poza
biegunem i objawia się dopiero jako rozwiązanie IK w konfiguracji, w której
ograniczenie nie miało sensu. Błąd algebraiczny wywala pierwszy test wartości.

### 2.3 Warunek składniowy — **lokalny**

```text
divisorsOf(E)  =  zbiór wyrażeń stojących w mianownikach wszystkich węzłów
                  Divide w E (bez powtórzeń, porównywane structurallyEqual)

hasSameDomainGuards(A, B)  <=>  divisorsOf(A) == divisorsOf(B)
```

Nazwa zmieniona z `preservesDomain` **celowo**: stara sugerowała, że funkcja
orzeka o dziedzinie, a orzeka o **składniowych strażnikach** dziedziny — i tylko
lokalnie.

Warunek jest stosowany **wyłącznie w jednym miejscu**: gdy reguła trafi, silnik
porównuje wyrażenie wejściowe reguły z jej wynikiem. Nigdy nie jest stosowany do
całych drzew przed i po przepisaniu — 2.4 tłumaczy, dlaczego to byłoby błędem.

Kierunek błędu jest jednostronny i to jest cała wartość warunku: **może odrzucić
przepisanie bezpieczne, nigdy nie przepuści niebezpiecznego.** Odrzuci np.
`x/x → 1` (dzielniki `{x}` kontra `{}`), i słusznie — to przepisanie naprawdę
rozszerza dziedzinę o zero.

### 2.4 Globalna własność wynika **indukcyjnie**, nie z porównania

v1 miała tu błąd, który review nazwało wprost. Weźmy bezpieczną regułę
`-(-q) → q` zastosowaną wewnątrz `a / -(-q)`:

```text
przed:  divisorsOf(a / -(-q))  =  { -(-q) }
po:     divisorsOf(a / q)      =  { q }
```

Strukturalnie różne. Gdyby warunek z 2.3 zastosować **globalnie**, poprawne
przepisanie zostałoby odrzucone — a v1 kazała dokładnie to robić w kroku 6 testu
`EquivalentOnSampledDomain`. Sprzeczność między §2.4 a §8 była realna.

Poprawnie jest tak: warunek działa lokalnie, a własność globalna to **twierdzenie
indukcyjne po strukturze**, przy założeniu, że autorzy reguł dotrzymują (E):

```text
baza:   reguła trafiająca w węzeł E daje E', przy czym
        hasSameDomainGuards(E, E')  oraz  E ≡ E' na dom(E)     [(E) od autora]
        => dom(E) = dom(E')

krok:   niech d -> d' przez powyższe. Skoro d ≡ d' na wspólnej dziedzinie,
        ich ZBIORY ZER są identyczne, więc
        dom(a/d) = dom(a/d')
```

Czyli: dziedzina całego drzewa jest zachowana **dlatego**, że każde pojedyncze
przepisanie ją zachowuje — a nie dlatego, że ktoś porównał listy dzielników na
końcu. Indukcja opiera się na (E), które pochodzi od autora reguły (2.2), i to
jest dokładnie miejsce, w którym gwarancja silnika przechodzi w kontrakt reguły.

### 2.5 Konsekwencja dla testów

Skoro globalne porównanie dzielników jest błędne, test dziedziny musi być
**behawioralny**, nie składniowy. `ExpressionEvaluator` daje na to gotowe
narzędzie:

```cpp
enum class EvaluationErrorCode { MissingSymbol, NonFiniteSymbolValue,
                                 DivisionByZero, NonFiniteResult };
```

Dla wylosowanego `q`: obie postacie muszą **zgadzać się co do tego, czy są
określone**, a gdy są — co do wartości. Szczegóły w 8.

### 2.6 Konsekwencja, która coś daje: `x * 0 → 0` bywa dopuszczalne

Fabryka odmawia zawsze, bo patrzy lokalnie w chwili budowania. Silnik widzi całe
poddrzewo:

```text
x * 0 -> 0    DOPUSZCZONE gdy divisorsOf(x) jest puste
              ODRZUCONE   gdy x zawiera jakikolwiek Divide
```

`q1 * 0 → 0` — wolno. `(1/q) * 0 → 0` — nie wolno. Ta sama reguła, dwie różne
odpowiedzi, obie wyprowadzone. F3.1 nie obchodzi ostrożności z Fazy 1, tylko
**zamienia zakaz globalny na warunek lokalny**. Zakaz w fabryce zostaje — fabryka
nadal nie ma czym orzekać.

### 2.7 Representability guard: uproszczony mianownik może być **nie do zbudowania**

Blocker z review, i najbardziej konkretny z czterech, bo wynika wprost z kodu.
Dziś `q - q` zostaje węzłem `Subtract`, więc **legalnie** istnieje wyrażenie:

```text
a / (q - q)
```

W F3.2 naturalna reguła `q - q → 0` jest lokalnie poprawna algebraicznie i ma
pusty zbiór dzielników po obu stronach, więc warunek z 2.3 ją **przepuści**.
Silnik, odbudowując rodzica, wywoła:

```cpp
factory.divide(a, /* 0 */)
```

a tam stoi:

```cpp
assert(!isZero(rhs) && "division by literal zero");
```

W `build-werror` (asercje żywe) to abort. W Release asercja znika i powstaje
`Divide(a, 0)` — węzeł, którego cała reszta projektu ma prawo uważać za
niemożliwy. Oba wyniki są nie do przyjęcia.

**Reguła silnika:** przy odbudowie węzła `Divide` przepisany mianownik jest
przyjmowany tylko wtedy, gdy **nie jest literalnym zerem**; w przeciwnym razie ta
pozycja zachowuje mianownik oryginalny.

```text
Divide(a, d)  i  d -> d'
    isZero(d')  ->  zachowaj d, przepisanie w tej POZYCJI odrzucone
    inaczej     ->  Divide(a', d')
```

Trzy rzeczy warto powiedzieć o tej regule, bo każda jest nieoczywista:

**Nie jest to błąd.** `a/(q-q)` jest wyrażeniem, którego dziedzina jest pusta —
caller zbudował coś bez sensu, ale F3.1 nie jest od diagnozowania tego. Zwrócenie
wyrażenia niezmienionego jest uczciwe: nie umiemy go uprościć, nie psujemy go.

**Odrzucenie jest zależne od kontekstu**, a memoizacja — nie. Przepisanie
`q - q → 0` pozostaje ważne wszędzie indziej, więc wpis w memo zostaje. Guard
działa dopiero przy **składaniu rodzica**, wybierając między dwoma już
policzonymi wyrażeniami. Dzięki temu współdzielenie nie cierpi (5.1), a jedyny
koszt to jedno porównanie `isZero` na każdy węzeł `Divide`.

**Nie zastępuje warunku z 2.3, tylko go uzupełnia.** Tamten pilnuje dziedziny,
ten — **reprezentowalności**: czy wynik da się w ogóle zbudować fabryką, która
ma własne kontrakty. To dwa różne pytania i potrzebne są oba.

---

## 3. Reguła

### 3.1 Kształt

```cpp
// A rewrite rule: given an expression, either produce a replacement or decline.
//
// nullopt means "does not apply", NOT "applies and changes nothing". A rule that
// returned its input unchanged would spin the fixed-point loop, so the engine
// treats an unchanged result as a defect (4.3).
struct RewriteRule
{
    std::string_view name;                                 // for diagnostics
    std::optional<Expression> (*apply)(const Expression&, const ExpressionFactory&);
};
```

Reguła jest **danymi, nie klasą**: zestaw reguł staje się `span`em, testy mogą
uruchomić jedną regułę bez reszty, a nazwa w komunikacie asercji mówi, co
zadziałało.

### 3.2 Podział odpowiedzialności — dokładnie

```text
REGUŁA odpowiada za:    (E) -- że B jest algebraicznie równe A na dom(A)
                        malejącą miarę (4.3)
                        własne testy wartości, w F3.2

SILNIK odpowiada za:    warunek składniowy dziedziny (2.3)
                        representability guard (2.7)
                        terminację (4.2)
                        zachowanie współdzielenia (5)
```

Reguła **nigdy** nie sprawdza dziedziny sama. Gdyby sprawdzała, bezpieczeństwo
byłoby własnością dyscypliny autora — a projekt ma precedens mówiący, jak to się
kończy i jak się to naprawia: R5 zamknęło `ExpressionFactory` nad kontraktem
skończoności zamiast liczyć na pamięć wołających.

Symetrycznie: silnik **nie** sprawdza (E), bo nie umie. Ta asymetria jest treścią
2.2 i jedyną rzeczą, którą trzeba o tym etapie zapamiętać.

---

## 4. Silnik

### 4.1 Kształt przejścia

```text
rewrite(expression)
    1. przepisz rekurencyjnie dzieci        (memoizacja, 5)
    2. złóż węzeł z przepisanych dzieci     (representability guard, 2.7)
    3. spróbuj reguł na węźle bieżącym, po kolei
    4. każde trafienie sprawdź warunkiem z 2.3; odrzucone -> następna reguła
    5. jeśli coś zadziałało, wróć do 1 dla wyniku
    6. stop na punkcie stałym albo na limicie
```

Kolejność **innermost** jest wyborem: reguła na węźle bieżącym widzi poddrzewa
już uproszczone, więc wzorce z F3.2 pisze się pod postać po uproszczeniu, a nie
pod wszystkie warianty wejścia.

### 4.2 Terminacja — trzy niezależne mechanizmy

```text
a) kontrakt reguły     reguła MUSI zmniejszać miarę (4.3)
b) limit kroków        twardy budżet, niezależny od (a)
c) raport o limicie    wynik mówi, że nie osiągnięto punktu stałego
```

`(a)` jest właściwym zabezpieczeniem: przy ściśle malejącej mierze pętla
`A → B → A` jest niemożliwa. `(b)` istnieje **mimo** `(a)`, bo `(a)` jest
kontraktem, a kontrakty łamie reguła napisana jutro; budżet zamienia
nieskończoną pętlę w **zdiagnozowany** koniec. `(c)` istnieje, bo ciche „prawie
uproszczone" jest gorsze niż jawne „nie zbiegło".

Alternatywa — pamiętanie odwiedzonych postaci i wykrywanie cyklu — **odrzucona**:
wymaga haszowania struktury każdego pośredniego wyrażenia, kosztuje pamięć
proporcjonalną do liczby kroków i wykrywa problem dopiero, gdy wystąpi.

### 4.3 Miara

```text
measure(E) = liczba węzłów liczona Z KROTNOŚCIAMI
```

Z krotnościami, nie unikalnych: miara unikalnych potrafi **wzrosnąć** przy
przepisaniu zmniejszającym wyrażenie (rozbicie współdzielenia), więc kontrakt
byłby nie do spełnienia dla poprawnych reguł.

Kontrakt: `measure(B) < measure(A)`. Sprawdzany `assert`em z nazwą reguły w
komunikacie — złamanie to błąd programisty, a limit i tak zamienia go w
skończone zakończenie. Precedens takiego `assert`a bez wyłuskania: F2.5Q.

---

## 5. DAG

### 5.1 Memoizacja jest wymogiem, nie optymalizacją

Naiwna rekurencja odwiedza **każde wystąpienie** poddrzewa: dla FK `kr640` to
281 kontra 21 882, czyli **77×** — przy mniejszym z dwóch robotów w repo.

Skutki memoizacji są dwa i oba są wymagane:

```text
złożoność   liniowa w liczbie UNIKALNYCH węzłów
struktura   współdzielenie przeżywa przepisanie -- wynik jest DAG-iem, nie drzewem
```

Drugi jest ważniejszy dla F3.2: gdyby wynik był drzewem, każde kolejne przejście
symplifikatora startowałoby od materiału 77 razy większego.

### 5.2 Klucz: `Expression`, **nie** surowy wskaźnik

v1 proponowała `unordered_map<const ExpressionNode*, Expression>` i argumentowała,
że wejściowe wyrażenie utrzymuje węzły przy życiu. To prawda **tylko dla węzłów
oryginalnego DAG-a**. Po udanym przepisaniu (4.1, krok 5) silnik schodzi ponownie
w wyrażenie **świeżo zbudowane**; jego węzły żyją tak długo, jak trzyma je lokalna
zmienna, a po zwolnieniu allocator może oddać ten sam adres innemu węzłowi.
Wpis w memo wskazywałby wtedy na coś zupełnie innego, i to bez żadnego objawu.

Projekt rozwiązał to już raz, w `ExpressionEvaluator`:

```cpp
using Memo = std::unordered_map<Expression, double,
                                ExpressionIdentityHash, ExpressionIdentityEqual>;
```

Klucz jest `Expression`, więc **sam utrzymuje węzeł przy życiu**; hash liczy się z
adresu, a równość to `sameNode`. F3.1 używa dokładnie tego samego wzorca, z
`Expression` po obu stronach mapy. Koszt: jeden `shared_ptr` na wpis. Zysk:
klasa błędów, która nie istnieje.

### 5.3 Silnik jest **sesją**, nie wolną funkcją

Rozstrzygnięcie, o które prosiło review, i lepiej podjąć je teraz niż w F3.2.

`ExpressionEvaluator` jest sesją, bo szesnaście komórek macierzy to wiele korzeni
**jednego** współdzielonego DAG-a i cache działa między nimi. F3.2 ma dokładnie
ten sam kształt: przepisze `lhs` i `rhs` dwunastu równań jednego układu, a te
strony współdzielą poddrzewa FK. Wolna funkcja z memo per wywołanie przebudowałaby
wspólne poddrzewo **24 razy** i rozbiła współdzielenie **między** równaniami.

```cpp
class RewriteSession        // rules + factory + limit + memo
{
public:
    RewriteSession(std::span<const RewriteRule> rules, ExpressionFactory factory,
                   std::size_t stepLimit = kDefaultRewriteStepLimit);

    [[nodiscard]] RewriteResult rewrite(const Expression& expression);
};
```

Dziedziczy przy tym ostrzeżenie z evaluatora, wpisane w komentarz klasy: **memo
rośnie monotonicznie**, a jedna sesja to jeden zestaw reguł. Sesja jest tania —
kilka pól — więc dla innego zestawu tworzy się nową.

Wolna funkcja jednorazowa zostaje jako wygoda dla testów i wywołań pojedynczych,
zaimplementowana przez sesję.

---

## 6. Wynik

```cpp
struct RewriteResult
{
    Expression expression;      // punkt stały albo stan po wyczerpaniu limitu
    bool changed{};
    std::size_t steps{};
    bool reachedLimit{};
};
```

`changed` ma konkretnego konsumenta: F3.2 puszcza kilka przebiegów różnymi
zestawami reguł i musi wiedzieć, kiedy przestać.

`reachedLimit` **nie jest błędem**, więc nie jest `expected`. Wyrażenie po
wyczerpaniu limitu jest poprawne — o tej samej dziedzinie i (przy dotrzymanym (E))
równoważne — tylko nie kanoniczne. Zwracanie go jako błędu zmuszałoby caller'a do
wyrzucenia poprawnego wyniku.

Nie ma listy zastosowanych reguł: rosłaby ze `steps`, a diagnostykę da się mieć
taniej — nazwa reguły trafia do komunikatu asercji przy złamaniu miary.

---

## 7. API

### 7.1 Pliki

```text
symbolic/RewriteRule.hpp     RewriteRule, RewriteResult, kDefaultRewriteStepLimit
symbolic/RewriteEngine.hpp   divisorsOf, hasSameDomainGuards, RewriteSession, rewrite
symbolic/RewriteEngine.cpp   sesja, memoizacja, guardy, kontrola miary
```

W `symbolic/`, nie w `builders/`: to narzędzie warstwy wyrażeń.

### 7.2 Funkcje

```cpp
// The syntactic domain guards of an expression: the divisor of every Divide
// node, without duplicates. Complete for the nine node kinds that exist today --
// a new partial operator (sqrt, log, asin) would need a richer model, not one
// more entry here.
[[nodiscard]] std::vector<Expression> divisorsOf(const Expression& expression);

// Compares those guards. Deliberately NOT called preservesDomain: it decides a
// syntactic question, and only LOCALLY -- at the point a rule fires, between the
// rule's input and its output. Applying it to whole trees before and after a
// rewrite is wrong and would reject correct rewrites; the whole-tree property
// follows by induction instead.
[[nodiscard]] bool hasSameDomainGuards(const Expression& before, const Expression& after);
```

### 7.3 Reguły porządkujące — **poza tym silnikiem**

v1 twierdziła, że `span` reguł jest „szwem" dla reguł porządkujących. To była
deklaracja przyszłej zmiany, nie szew: drugi przebieg używa **tej samej** miary,
więc `x + y → y + x` nadal jej nie przejdzie, bo miary nie zmienia.

Rozstrzygnięcie: **normalizacja kolejności nie jest przepisywaniem w rozumieniu
tego silnika.** To osobny przebieg kanonizujący, który F3.2 zaimplementuje
własnym mechanizmem — z własnym porządkiem dobrze ufundowanym — i wywoła *przed*
albo *po* `RewriteSession`, nie *przez* nią.

Gdyby F3.2 pokazało, że jeden silnik z parametryzowaną miarą jest lepszy, to
zmiana w `RewriteSession` uzasadniona konkretami. F3.1 nie udaje, że ma na to
gotowe API.

### 7.4 Równania i układy

F3.1 **nie** dotyka `Equation` ani `IkEquationSystem`. Kompozycja należy do F3.2:

```text
przepisz lhs, przepisz rhs, zachowaj source -- JEDNĄ sesją dla całego układu (5.3)
```

Bezpieczne, bo `Equation` to `lhs = rhs` przechowywane tak, jak podane —
przepisanie każdej strony osobno nie rusza granicy między stroną robota a stroną
zadania, na której stoi model z F2.3.

---

## 8. Testy

```text
strażnicy dziedziny -- warunek lokalny
  CollectsDivisorsFromNestedExpression       bez powtórzeń, z zagnieżdżeń
  RejectsRewriteThatWidensDomain             (1/q)*0 -> 0 ODRZUCONE
  AllowsSameRewriteWhenNoDivisionPresent     q1*0 -> 0 DOPUSZCZONE
  RejectsCancellationOfSelfDivision          x/x -> 1 odrzucone
  RewritesInsideDenominator                  a/-(-q) -> a/q PRZECHODZI (2.4)

representability guard
  KeepsOriginalDenominatorWhenRewriteYieldsZero    a/(q-q) nietknięte
  StillRewritesSameSubtreeElsewhere                q-q -> 0 dziala poza mianownikiem

silnik
  ReportsChangedWhenARuleFires
  ReportsUnchangedOnFixedPoint
  StopsAtStepLimitAndSaysSo                  reachedLimit, wynik poprawny
  AppliesRulesInnermostFirst
  DiscardsUnsafeRuleAndContinues             odrzucona nie blokuje kolejnych

DAG
  PreservesSharingAcrossRewrite
  RewritesSharedSubtreeOnce                  licznik wywołań: 1, nie 77
  SharesMemoAcrossRootsInOneSession          drugi korzeń nie liczy od nowa (5.3)
  SurvivesRewriteOfTemporaryExpression       klucz Expression, nie wskaźnik (5.2)

równoważność Z DZIEDZINĄ
  AgreesOnDefinednessAndValue                patrz niżej
  RewrittenKr640FkMatchesOriginal            end-to-end na prawdziwym FK
```

**`AgreesOnDefinednessAndValue`** zastępuje `EquivalentOnSampledDomain` z v1 i
jest zbudowany inaczej, bo tamten zawierał błąd z 2.4:

```text
1. weź wyrażenie z dzieleniem, np. a/(q1 - 0.5)
2. przepisz
3. wylosuj N konfiguracji ze STAŁYM ziarnem, w tym punkty BLISKO bieguna
4. dla każdej policz evaluate(przed) i evaluate(po)
5. wymagaj ZGODNOŚCI CO DO OKREŚLONOŚCI:
       obie zwracają wartość            -> wartości równe w granicach 1e-12
       obie zwracają DivisionByZero     -> zgoda
       jedna i druga inaczej            -> BŁĄD, dziedzina się zmieniła
```

Krok 5 jest sednem. v1 kazała pomijać punkty przy biegunie i **osobno** porównywać
`divisorsOf` przed i po — czyli robić dokładnie to, czego 2.4 zabrania, bo
`a/-(-q)` i `a/q` mają różne dzielniki, a tę samą dziedzinę. Test behawioralny
mierzy **dziedzinę**, a nie jej składniowy zapis, więc przechodzi tam, gdzie
tamten dawał fałszywy alarm, i wywala się przy `(1/q)*0 → 0`, gdzie jedna postać
jest w zerze określona, a druga nie.

Sama równość wartości poza biegunem **nie potrafi** wykryć błędu, przed którym
chroni ten etap — i to jest powód, dla którego krok 3 celowo losuje punkty blisko
zera mianownika.

---

## 9. Plan zmian w plikach

```text
NOWE   symbolic/RewriteRule.hpp        RewriteRule, RewriteResult, limit
       symbolic/RewriteEngine.hpp      divisorsOf, hasSameDomainGuards,
                                       RewriteSession, rewrite()
       symbolic/RewriteEngine.cpp      sesja, memo, guardy, miara
       tests/test_rewrite_engine.cpp   ~18 testów

ZMIENIONE  CMakeLists.txt, tests/CMakeLists.txt   +1 wiersz każdy

BEZ ZMIAN  Expression, ExpressionFactory, ExpressionEvaluator, SymbolicMatrix,
           cały model, ConstraintBuilder, fasada, loader
```

Ostatni wiersz jest sprawdzianem: F3.1 **dokłada warstwę**. Gdyby silnik
potrzebował zmiany w `Expression` albo w fabryce, byłby to sygnał, że któreś
założenie z 2 jest nie do utrzymania — ważniejszy niż sam etap.

Szczególnie: **`divide` nie jest luzowane.** Representability guard z 2.7 istnieje
właśnie po to, żeby nie trzeba było ruszać kontraktu fabryki.

---

## 10. Ryzyka

| Ryzyko | Waga | Reakcja |
|---|---|---|
| Reguła zmienia dziedzinę | **realne, powód istnienia etapu** | Warunek sprawdzany przez silnik przy każdym trafieniu, nie przez autora (3.2) |
| Dokument obiecuje więcej, niż silnik sprawdza | **realne, złapane w review v1** | 2.2 rozdziela gwarancje silnika od kontraktu reguły; „sprawdzalnie bezpieczne" wycofane |
| Globalne porównanie dzielników odrzuca poprawne przepisania | **realne, złapane w review v1** | Warunek wyłącznie lokalny (2.3), własność globalna indukcyjnie (2.4), test behawioralny (8) |
| Uproszczony mianownik nie do zbudowania | **realne, złapane w review v1** | Representability guard (2.7); `assert` w `divide()` zostaje nietknięty |
| Klucz memo przeżywa swój węzeł | **realne, złapane w review v1** | Klucz `Expression`, wzorzec z `ExpressionEvaluator` (5.2), własny test |
| Warunek strukturalny odrzuca bezpieczne przepisania | świadome | Kierunek błędu jednostronny (2.3) |
| Rozbicie współdzielenia między równaniami | realne | Sesja z jednym memo dla wielu korzeni (5.3) |
| Nowy operator częściowy łamie model dziedziny | **realne, nie zamykane** | 2.1 mówi wprost: `divisorsOf` jest kompletne dla dziewięciu typów; dziesiąty wymaga uogólnienia, a `std::visit` zatrzyma build |
| Reguły porządkujące nie mieszczą się w mierze | świadome | 7.3: to osobny mechanizm, poza tym silnikiem |

---

## 11. Otwarte pytania do review

**11.1 Zbiory czy multizbiory dzielników?** Rekomendacja: **zbiory**. `a/x + b/x`
ma jeden dzielnik występujący dwa razy, a dla dziedziny liczy się tylko `x ≠ 0`.
Multizbiór odrzucałby łączenie takich składników bez zysku dla bezpieczeństwa.

**11.2 Domyślny limit kroków.** v1 podawała `10 000` i uzasadniała to
porównaniem z **281 unikalnymi węzłami** — a miara z 4.3 liczy **z krotnościami**,
czyli 21 882 dla tego samego FK. Uzasadnienie było więc oparte na niewłaściwej
liczbie.

Poprawnie: każdy krok **ściśle** zmniejsza miarę, więc liczba kroków jest z góry
ograniczona przez `measure(wejścia)`. Dla FK `kr640` to 21 882. Limit ma być
siatką na złamany kontrakt, a nie progiem zatrzymującym normalną pracę, więc musi
leżeć **powyżej** tej wartości:

```text
kDefaultRewriteStepLimit = 100 000
```

Rząd wielkości powyżej miary największego znanego wejścia. Liczba pozostaje
arbitralna i taka ma zostać — to siatka bezpieczeństwa, nie próg numeryczny.

**11.3 Czy sesja ma przyjmować `ExpressionFactory` przez wartość?** Rekomendacja:
tak, jak robią to `JointTransformBuilder` i `ForwardKinematicsBuilder`. Fabryka
jest dziś bezstanowa; gdyby kiedyś zyskała stan (hash-consing), zmiana własności
będzie osobną decyzją, zapisaną już w `STATUS.md` jako znany brak.

---

## 12. Rekomendacja

Przyjąć: definicję z 2.1 wraz z jej jawnym ograniczeniem, rozdział gwarancji z
2.2, **lokalny** warunek z 2.3 z indukcją z 2.4, representability guard z 2.7,
sesję z 5.3 i klucz memo z 5.2.

Po zatwierdzeniu — proposal implementacyjny z pełnym kodem trzech nowych plików
i około osiemnastoma testami, gate `440/440` (422 + 18).

Po F3.1 wchodzi **F3.2** — pierwsze prawdziwe reguły: `x + 0`, `x * 1`, zwijanie
stałych, `q - q → 0`. To tam okaże się, czy guard z 2.7 był potrzebny — i to
tam, po raz pierwszy, ktoś będzie musiał dotrzymać (E).
