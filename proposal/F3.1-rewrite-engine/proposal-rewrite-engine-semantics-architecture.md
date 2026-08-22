# Proposal: semantyka rewrite engine (F3.1), architektura

**Etap:** F3.1 · **v5 — do review** · 2026-08-22 · część 1 z 2

> **v5, po czwartym review.** Jeden blocker, tym razem w **definicji punktu
> stałego**, plus trzy rekomendacje przyjęte w całości:
>
> 1. **Silnik mógł zameldować punkt stały, który nim nie był.** Reguła dostaje
>    fabrykę, więc wolno jej zbudować poddrzewo, którego w wejściu nie było;
>    powrót „do kroku 3" go nie normalizował. Po udanej regule silnik wraca do
>    **przepisania dzieci**, nie do próby reguł (4.1).
> 2. **Sesja posiada swoje reguły** — `vector<RewriteRule>` z `std::string name`,
>    `span` zostaje kształtem parametru konstruktora (3.1).
> 3. **Cache strażników też jest transakcyjny** — nie dla poprawności, tylko żeby
>    ścieżka ratunkowa sprzątała po sobie zamiast zostawiać milion żywych
>    tymczasowych DAG-ów (5.5).
> 4. **Limit przestaje udawać twierdzenie.** Budżet defensywny, nie granica
>    wyprowadzona z `measure(root)` — bo ta granica nie jest prawdziwa (11.2).

> **v4, po trzecim review.** Jeden blocker i trzy korekty, wszystkie na tym samym
> styku co poprzednio — sesja żyjąca dłużej niż pojedyncze wywołanie:
>
> 1. **Wynik przerwany limitem mógł wyciec do memo sesji** i zostać podany
>    następnemu korzeniowi jako gotowy, z `steps = 0` i `reachedLimit = false`.
>    Memo sesji przyjmuje odtąd **wyłącznie udowodnione punkty stałe**; wpisy
>    wywołania są lokalne i commitowane dopiero na końcu (5.5).
> 2. **Limit 100 000 był mniejszy od największego realnego wejścia.** Zmierzone:
>    `kr4_r600` to **153 703** węzłów z krotnościami, nie `kr640` z 21 882.
>    Limit to `1 000 000` (11.2).
> 3. **Deklaracja O(N) nadal była za mocna** — cache zdejmuje powtarzane skany,
>    ale nie koszt porównywania zbiorów strażników (5.4).
> 4. Powrót po udanej regule jest **pętlą**, nie rekurencją (4.1).

> **v3, po drugim review.** Rdzeń nietknięty; trzy problemy, wszystkie na
> **stykach** mechanizmów wprowadzonych w v2:
>
> 1. **`changed` i `steps` traciły sens przy sesji dzielonej między korzeniami.**
>    Z memo możliwe są oba przypadki naraz: reguła zadziałała, a wynik jest
>    identyczny z wejściem, oraz wynik się różni, choć żadna reguła w tym wywołaniu
>    nie poszła. Kontrakt obu pól zdefiniowany od nowa, razem z wymogiem
>    **zachowania tożsamości węzła** (6).
> 2. **Test dziedziny nadal mógł przejść mimo błędu, przed którym chroni.** Dla
>    `(1/q)*0 -> 0` punkty **blisko** zera dają po obu stronach `0`; różnica jest
>    widoczna **wyłącznie w dokładnym zerze**. Test dostaje deterministyczne punkty
>    graniczne (8).
> 3. **Obietnica złożoności liniowej nie była uzasadniona**, bo `divisorsOf`
>    skanowało pod-DAG przy każdym trafieniu reguły. Dochodzi drugi cache sesji,
>    liczony kompozycyjnie (5.4).
>
> Plus dwie drobne: porównanie strażników **niezależne od kolejności** (7.2) oraz
> rozdzielenie zasięgów — memo na sesję, limit i licznik kroków **na wywołanie**
> (5.3, 11.2).

> **v2, po pierwszym review.** Rdzeń utrzymany — `divisorsOf`, przejście innermost, malejąca
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
    // An owning std::string, not a string_view. A view would make the rule's
    // name a reference into storage the session does not control, and the
    // session outlives the call that built it (5.3). Rules number a dozen at
    // most and are copied once, at construction.
    std::string name;                                      // for diagnostics
    std::optional<Expression> (*apply)(const Expression&, const ExpressionFactory&);
};
```

Reguła jest **danymi, nie klasą**: zestaw reguł przekazuje się jako `span`, testy
mogą uruchomić jedną regułę bez reszty, a nazwa w komunikacie asercji mówi, co
zadziałało.

**Sesja reguły kopiuje, nie pożycza.** `span` jest kształtem **parametru
konstruktora**, nie kształtem pola:

```cpp
RewriteSession(std::span<const RewriteRule> rules, ExpressionFactory factory, ...)
    : rules_(rules.begin(), rules.end()), factory_(std::move(factory)) { }

std::vector<RewriteRule> rules_;
```

Bez tego pierwsza naturalna fabryka sesji jest błędem, którego kompilator nie
złapie:

```cpp
RewriteSession makeCanonicalSession()
{
    std::vector<RewriteRule> rules = canonicalRules();
    return RewriteSession{rules, ExpressionFactory{}};
}   // rules umiera; sesja zostaje z wiszącym span'em
```

Sam `vector<RewriteRule>` by tego nie załatwił, dopóki `name` było
`string_view` — stąd `std::string` wyżej. Precedens jest w projekcie:
`ExpressionEvaluator` posiada swoje `SymbolValues`, a `JointTransformBuilder`
i `ForwardKinematicsBuilder` biorą `ExpressionFactory` przez wartość. API
niewłasnościowe nie kupuje tu **niczego** — kilkanaście reguł kopiuje się raz,
przy konstrukcji.

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
rewriteNode(expression)
    current = expression
    pętla:
        1. przepisz rekurencyjnie dzieci current      (memoizacja, 5)
        2. złóż current z przepisanych dzieci        (representability guard, 2.7)
        3. znajdź pierwszą dopuszczalną regułę na current
              trafienie sprawdzone warunkiem z 2.3; odrzucone -> następna reguła
        4. brak dopuszczalnej reguły  -> ZWRÓĆ current      (punkt stały)
        5. current = wynik reguły; ++steps; WRÓĆ DO 1
```

Dwie rzeczy w tej pętli są rozstrzygnięciami, nie zapisem oczywistości.

**Krok 5 wraca do 1, nie do 3.** v4 wracała do 3 i twierdziła, że to wystarczy, bo
„wynik reguły składa się z materiału, który przeszedł przez krok 1". **To było
nieprawdą.** `RewriteRule::apply` dostaje `ExpressionFactory` — z premedytacją,
bo bez niej nie zbuduje niczego — więc wolno jej **skonstruować poddrzewo, którego
w wejściu nie było**. Kontrakt wymaga od niej tylko (E) i malejącej miary.

Kontrprzykład, całkowicie legalny względem 3.2:

```text
A  = (q + q) + (r - r)

R1:  A -> 2 * -(-q)          (E):  (q+q)+(r-r) = 2q  oraz  2*-(-q) = 2q
                             miara: 7 -> 5, ściśle malejąca
                             dziedzina: brak Divide po obu stronach, guardy równe

R2:  -(-x) -> x              zwykła reguła kanonizująca z F3.2
```

Przy powrocie do kroku 3 silnik próbuje reguł na `Multiply(2, -(-q))`. Żadna nie
pasuje do `Multiply`, więc silnik kończy i **melduje punkt stały** — podczas gdy
w środku siedzi `-(-q)`, na które R2 pasuje. Wynik nie jest punktem stałym, a
`RewriteResult` twierdzi, że jest.

Sprawdzone w kodzie, bo od tego zależy, czy przykład w ogóle istnieje:
`ExpressionFactory::negate` składa **wyłącznie stałe** (`ExpressionFactory.cpp:77`),
więc `-(-q)` jest w pełni konstruowalne i nie znika przy budowie.

Powrót do kroku 1 zamyka dziurę: każde świeże poddrzewo wprowadzone przez regułę
przechodzi **pełną normalizację innermost**, zanim cokolwiek zostanie uznane za
punkt stały. Dla dzieci już znormalizowanych to kosztuje jedno trafienie w memo.

**Pętla, nie rekurencja.** To zostaje z v4 i współgra z powyższym: kolejne
zastosowania reguł na tym samym węźle siedzą w `while`, nie w łańcuchu ramek
stosu. Rekurencja występuje **wyłącznie** w kroku 1, przy schodzeniu po dzieciach,
gdzie głębokość ogranicza wysokość DAG-a, a nie liczba kroków. Bez tego limit z
11.2 nie byłby siatką bezpieczeństwa — proces padałby na stosie, zanim limit by
zadżiałał.

**Dlaczego pętla się kończy.** Każdy obrót wymaga zastosowania reguły na
`current`, a to ściśle zmniejsza `measure(current)` (4.3). Krok 1 miary nie
zwiększa — przepisanie dziecka samo jest ciągiem kroków ściśle malejących. Liczba
obrotów na danym węźle jest więc ograniczona przez `measure` tego węzła. To
argument **per węzeł** i tylko taki jest tu prawdziwy — dlaczego nie sumuje się do
ładnej granicy dla całego wywołania, rozliczone w 11.2.

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
`A → B → A` jest niemożliwa, a pętla z 4.1 na danym węźle ma skończoną liczbę
obrotów. `(b)` istnieje **mimo** `(a)`, bo `(a)` jest kontraktem, a kontrakty
łamie reguła napisana jutro; budżet zamienia nieskończoną pętlę w
**zdiagnozowany** koniec.

Uwaga, która oszczędzi czytelnikowi złego wniosku: `(a)` daje terminację, ale
**nie** daje granicy `steps ≤ measure(root)` — krok zaakceptowany głębiej może
zostać wyrzucony przy składaniu rodzica przez guard z 2.7, więc licznik rośnie,
a miara korzenia nie maleje. Rozliczone w 11.2, gdzie z tego powodu limit jest
budżetem, a nie wyprowadzoną liczbą. `(c)` istnieje, bo ciche „prawie
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

Naiwna rekurencja odwiedza **każde wystąpienie** poddrzewa. Zmierzone na obu
robotach w repo, na dzisiejszym pipeline:

```text
kr640.urdf       21 882 z krotnościami      281 unikalnych       77,9x
kr4_r600.urdf   153 703 z krotnościami      516 unikalnych      297,9x
```

Skutki memoizacji są dwa i oba są wymagane:

```text
przejście   każdy UNIKALNY węzeł odwiedzany raz    (a nie raz na wystąpienie)
struktura   współdzielenie przeżywa przepisanie -- wynik jest DAG-iem, nie drzewem
```

Pierwsza linijka mówi o **przejściu**, nie o całym koszcie etapu — v2 pisała tu
„złożoność liniowa" i to było za mocne nawet po dodaniu cache'u z 5.4. Analiza
dziedziny ma własny koszt, który nie znika, tylko przestaje się powtarzać;
rozliczenie w 5.4.

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
class RewriteSession
{
public:
    // Takes a span and COPIES it -- the session owns its rules (3.1).
    RewriteSession(std::span<const RewriteRule> rules, ExpressionFactory factory,
                   std::size_t stepLimit = kDefaultRewriteStepLimit);

    [[nodiscard]] RewriteResult rewrite(const Expression& expression);

private:
    std::vector<RewriteRule> rules_;
    ExpressionFactory factory_;
    std::size_t stepLimit_;
    Memo memo_;         // proven fixed points only (5.5)
    Guards guards_;     // domain guards per node (5.4)
};
```

Dziedziczy przy tym ostrzeżenie z evaluatora, wpisane w komentarz klasy: **memo
rośnie monotonicznie**, a jedna sesja to jeden zestaw reguł. Sesja jest tania —
kilka pól — więc dla innego zestawu tworzy się nową.

**Zasięgi, rozdzielone świadomie** (poprawka z v3):

```text
reguły        na SESJĘ    -- kopiowane, nie pożyczane (3.1)
memo sesji    na SESJĘ    -- po to sesja istnieje: współdzielenie między korzeniami
                            ...ale WYŁĄCZNIE punkty stałe, 5.5
guard cache   na SESJĘ    -- 5.4, też transakcyjny, 5.5
memo lokalne  na WYWOŁANIE -- wpisy nowe obu cache'y, commit dopiero na końcu, 5.5
stepLimit     na WYWOŁANIE
steps         na WYWOŁANIE
```

Limit i licznik są per `rewrite(root)`, bo tylko wtedy zdanie z 11.2 — *liczba
kroków jest ograniczona przez `measure(root)`* — pozostaje prawdziwe. Budżet
wspólny dla sesji obsługującej dwadzieścia cztery strony równań znaczyłby coś
zupełnie innego i nie dałoby się go wyprowadzić z miary pojedynczego wyrażenia.

Wolna funkcja jednorazowa zostaje jako wygoda dla testów i wywołań pojedynczych,
zaimplementowana przez sesję.

### 5.4 Drugi cache: strażnicy dziedziny

v2 obiecywała złożoność **liniową w liczbie unikalnych węzłów** i to było
nieuzasadnione. Samo przejście z memo — owszem. Ale przy **każdym trafieniu
reguły** silnik woła `hasSameDomainGuards`, a to dwa razy `divisorsOf`, a to
skan pod-DAG-u. Bez pamiętania wyników powtarzające się skany tych samych
poddrzew dokładają do kosztu składnik śmiało kwadratowy, plus porównania
`structurallyEqual`.

Sesja dostaje więc **drugi cache**, kluczowany tak samo jak memo:

```cpp
std::unordered_map<Expression, std::vector<Expression>,
                   ExpressionIdentityHash, ExpressionIdentityEqual> guards_;
```

liczony **kompozycyjnie**, nie przez skan od zera:

```text
guards(Constant | Symbol)        = {}
guards(op(a, b))                 = guards(a) u guards(b)
guards(op(a))                    = guards(a)
guards(Divide(a, b))             = guards(a) u guards(b) u { b }
```

Przy memoizacji po tożsamości węzła **wpis cache'u dla każdego unikalnego węzła
liczony jest raz**. To jest cała obietnica i więcej obiecywać nie wolno:

```text
liczba wyliczeń guards(...)      raz na unikalny węzeł                O(N)
koszt POJEDYNCZEGO wyliczenia    suma zbiorów dzieci + deduplikacja   zależy od |guards|
koszt hasSameDomainGuards        parowanie structurallyEqual          kwadratowy w |guards|
```

Dla wyrażenia z wieloma **różnymi** dzielnikami zbiory strażników rosną i oba
dolne wiersze przestają być stałe, więc całość **nie jest** `O(N)` — jest `O(N)`
wywołań o koszcie zależnym od liczby różnych strażników. Formalnej granicy nie
podaję, bo zależy od materiału, którego jeszcze nie ma.

**Ile to realnie kosztuje: zero.** Zmierzone w tym repo: **żaden** budowniczy nie
woła `ExpressionFactory::divide` — dwa trafienia na `divide` w `src/ik_equations/`
poza samą fabryką to komentarze. FK, kompozycja TCP i wszystkie równania
ograniczeń nie zawierają **ani jednego** węzła `Divide`, więc dla dzisiejszego
materiału wszystkie zbiory strażników są puste, a `hasSameDomainGuards` porównuje
dwa puste zbiory.

Cała maszyneria z 2 jest więc **ubezpieczeniem**, nie kosztem na gorącej ścieżce:
dzielenie pojawi się dopiero wtedy, gdy wprowadzi je reguła Fazy 3 (podstawienia
półkątowe, `tan`) albo caller zbuduje je ręcznie. Warto to wiedzieć przy ocenie
proporcji: płacimy dziś niemal nic za własność, która stanie się krytyczna, gdy
solver zacznie dzielić.

I dlatego **dla dzisiejszego materiału** oba zależne od `|guards|` wiersze są
zerowe, a całość wychodzi `O(N)` w praktyce — architektonicznie nic nie tracimy
na tym, że nie deklarujemy tego jako twierdzenia. Gdyby kiedyś zaczęło
kosztować, właściwym ruchem będzie kanonizacja strażników (identyfikator na
unikalny dzielnik, porównanie zbiorów po `id`), a nie rezygnacja z warunku —
zapisane jako znany kierunek, nie jako plan.

### 5.5 Memo sesji przyjmuje **wyłącznie punkty stałe**

Zestawienie zasięgów z 5.3 — memo na sesję, limit na wywołanie — jest poprawne
dopiero wtedy, gdy każde wywołanie dochodzi do punktu stałego. Przy
`reachedLimit` powstaje dziura, której v3 nie widziała:

```text
r1 = session.rewrite(root1);      // limit wyczerpany w połowie roboty
r1.reachedLimit == true           // r1 wie, że dostał wynik nieukończony

   ...ale po drodze memo_[poddrzewo] = CZʌCIOWO przepisane poddrzewo

r2 = session.rewrite(root2);      // trafia w ten wpis
r2.steps        == 0
r2.reachedLimit == false          // KŁAMSTWO
```

Drugi caller dostaje niekanoniczne poddrzewo i **nie ma jak się o tym
dowiedzieć** — jego własny licznik kroków jest zerowy, jego własny limit
nietknięty. Co gorsza, `reachedLimit` przestaje znaczyć „to wywołanie nie
zbiegło" i zaczyna znaczyć „to wywołanie nie zbiegło **albo** któreś wcześniejsze
zostawiło śmieci" — czyli nie znaczy nic sprawdzalnego.

**Rozstrzygnięcie: memo sesji jest transakcyjne.**

```text
memo sesji      = WYŁĄCZNIE wyniki potwierdzone jako punkt stały

rewrite(root):
    czyta      memo sesji  +  memo lokalne tego wywołania
    zapisuje   TYLKO do memo lokalnego
    na końcu:
        punkt stały     -> commit memo lokalnego do memo sesji
        reachedLimit    -> memo lokalne PORZUCONE w całości
```

Porzucamy **całe** memo lokalne, nie tylko wpisy nieukończone, i to jest
świadome. Po przerwaniu limitem wpisy kompletne i niekompletne są
**nieodróżnialne** — rozwijający się stos zapisuje częściowe wyniki tak samo jak
każde inne. Odzyskiwanie tych dobrych wymagałoby znacznika na wpis, a wtedy
niesiemy dodatkowe pole przez cały czas życia sesji, żeby ratować cache **na
ścieżce diagnostycznej złamanego kontraktu reguły** — czyli tam, gdzie wydajność
nie ma żadnego znaczenia.

Rozważana alternatywa, zapisana żeby nie wracała:

```cpp
struct MemoEntry { Expression expression; bool reachedFixedPoint{}; };
```

Odrzucona nie dlatego, że jest zła, tylko dlatego, że **nic nie kupuje**: wpisu
z `false` i tak nie wolno użyć między wywołaniami, więc jedyną różnicą wobec
porzucenia jest trzymanie w pamięci danych, których nie wolno przeczytać.
Wersja transakcyjna ma za to jedno zdanie inwariantu, które daje się wypowiedzieć
bez zastrzeżeń:

> **Każdy wpis w memo sesji jest punktem stałym względem zestawu reguł tej sesji.**

Koszt: wywołanie przerwane limitem nie zostawia po sobie cache'u. Zgoda — to
ścieżka błędu, nie ścieżka pracy, a caller, który chce spróbować jeszcze raz,
może podać wynik z powrotem na wejście: jest poprawny, o tej samej dziedzinie,
tylko nieukończony.

**Cache strażników z 5.4 dostaje ten sam reżim** — i to jest zmiana względem v4,
która zostawiała go na sesji bezwarunkowo.

Powodem **nie** jest poprawność: `guards(E)` jest własnością samego `E`, nie
wynikiem przepisywania, więc przerwanie w połowie nie czyni żadnego wpisu
nieprawdziwym. Powodem są **zasoby**. Wyobraźmy sobie dokładnie ten scenariusz, dla
którego limit istnieje: zepsuta reguła w Release produkuje przez milion kroków nowe
węzły. Limit ratuje proces, memo przepisań zostaje porzucone — ale `guards_`
kluczuje po `Expression`, czyli **trzyma te wszystkie tymczasowe DAG-i przy
życiu**. Sesja przeżywałaby uratowanie z cache'em na milion wpisów po
wyrażeniach, do których nikt już nigdy nie sięgnie.

Ścieżka bezpieczeństwa ma także **posprzątać**, nie tylko przerwać. Oba cache'e
mają więc warstwę lokalną i wspólny moment commitu:

```text
punkt stały     -> commit memo lokalnego I lokalnych strażników
reachedLimit    -> porzucone OBA
```

Koszt ponownego policzenia strażników po nieudanym przebiegu jest żadny — dziś
zbiory są puste (5.4), a jutro liczy się je kompozycyjnie.

---

## 6. Wynik

### 6.1 Problem, którego v2 nie widziała

Sesja dzielona między korzeniami (5.3) rozjechała znaczenie obu pól, i to w obie
strony naraz. Przykład z review, dosłownie:

```text
shared = q - q
root1  = 1 / shared
root2  = shared + 5

rewrite(root1)   reguła q-q -> 0 ODPALA i trafia do memo,
                 ale representability guard (2.7) nie wpuszcza 0 do mianownika
                 => WYNIK IDENTYCZNY Z WEJŚCIEM, choć reguła zadziałała

rewrite(root2)   memo już ma shared -> 0, więc wynik to 0 + 5
                 => WYNIK INNY NIŻ WEJŚCIE, choć w tym wywołaniu żadna reguła
                    nie poszła
```

Przy definicji z v2 (`changed` = „czy cokolwiek zadziałało") dostalibyśmy
`root1: changed = true` przy niezmienionym wyniku i `root2: changed = false`
przy zmienionym. Oba kłamią, a F3.2 ma na `changed` sterować przebiegami.

### 6.2 Kontrakt

```cpp
struct RewriteResult
{
    Expression expression;      // punkt stały albo stan po wyczerpaniu limitu
    bool changed{};             // wynik RÓŻNI SIĘ od wejścia TEGO wywołania
    std::size_t steps{};        // NOWE przyjęte zastosowania reguł w TYM wywołaniu
    bool reachedLimit{};
};
```

```text
changed  =  !sameNode(wejście, wynik)
steps    =  liczba reguł przyjętych w tym wywołaniu; trafienie w memo NIE liczy się
```

Dla przykładu z 6.1:

```text
root1:   changed = false,  steps = 1
root2:   changed = true,   steps = 0
```

Przykład pozostaje ważny pod reżimem z 5.5: `rewrite(root1)` **dochodzi do punktu
stałego** — strażnik odrzucił podstawienie w mianowniku, ale nie przerwał pracy —
więc wpis `shared -> 0` jest commitowany i `root2` ma prawo go zobaczyć. Gdyby
`root1` skończył na limicie, `root2` nie zobaczyłby niczego i policzył od nowa.

Wygląda to na pierwszy rzut oka przewrotnie i **takie ma być**: te dwa pola
odpowiadają na dwa różne pytania. `changed` mówi konsumentowi, czy dostał coś
nowego; `steps` mówi, ile pracy wykonało to konkretne wywołanie. Przy dzielonym
memo te liczby **muszą** się rozjechać i lepiej, żeby rozjechały się zgodnie z
kontraktem niż przypadkiem.

Konsekwencja dla F3.2, wpisana wprost w komentarz: **do sterowania przebiegami
służy `changed`, nigdy `steps > 0`.**

### 6.3 Wymóg: zachowanie tożsamości węzła

Definicja `changed` przez `sameNode` nakłada na silnik obowiązek, który v2
przemilczała, a który i tak byłby potrzebny:

> Jeśli po przepisaniu dzieci **nic się nie zmieniło**, węzeł **nie jest
> przebudowywany** — silnik zwraca oryginalny `Expression`.

Bez tego każde przejście budowałoby strukturalnie identyczny, ale nowy graf,
`changed` byłoby zawsze prawdziwe, a współdzielenie z 5 przestawałoby istnieć przy
pierwszym przebiegu, który niczego nie uprasza. Dzięki temu wymogowi
`changed` jest też **darmowe**: jedno porównanie wskaźników, bez chodzenia po
drzewie.

### 6.4 Czego tu nie ma

`reachedLimit` **nie jest błędem**, więc wynik nie jest `expected`. Wyrażenie po
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

// Compares those guards AS SETS -- order-independent, because divisorsOf walks
// the tree and the same two guards may come out in either order. A plain
// vector == vector would reject a rewrite for reordering its own output.
// Matching is pairwise structurallyEqual, which is quadratic in the number of
// guards and irrelevant in practice: nothing this project builds today contains
// a single Divide node (5.4), and even a solver-era expression carries a handful.
//
// Deliberately NOT called preservesDomain: it decides a syntactic question, and
// only LOCALLY -- at the point a rule fires, between the rule's input and its
// output. Applying it to whole trees before and after a rewrite is wrong and
// would reject correct rewrites; the whole-tree property follows by induction
// instead.
[[nodiscard]] bool hasSameDomainGuards(const Expression& before, const Expression& after);
```

**Te dwie są API i narzędziem testów, a nie ścieżką, którą chodzi silnik.** Wołane
wprost, przechodzą DAG od zera przy każdym wywołaniu — dokładnie ten koszt, który
5.4 zdejmuje cache'em sesji. Silnik używa wersji sesyjnej:

```cpp
class RewriteSession
{
    // cached; consults guards_, fills it compositionally on a miss (5.4)
    [[nodiscard]] const std::vector<Expression>& domainGuards(const Expression&);
    [[nodiscard]] bool hasSameDomainGuards(const Expression& before,
                                           const Expression& after);
};
```

Wolne funkcje są jednorazowymi opakowaniami: sesja jednorazowa, jedno pytanie,
cache umiera z odpowiedzią. **Algorytm istnieje raz** — dwa niezależne
analizatory dziedziny to dokładnie ten rodzaj rozjazdu, na którym warunek
bezpieczeństwa przestaje znaczyć to, co mówi jego test. Szczegół należy do
proposalu implementacyjnego; tu jest zapisany, żeby nie został przeoczony.

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
  RewritesFreshDescendantsIntroducedByRule   A -> 2*-(-q) -> 2*q w JEDNYM rewrite (4.1)
  StopsAtStepLimitAndSaysSo                  reachedLimit, wynik poprawny
  ReportsUnchangedWhenGuardRejectsInDenominator   root1 z 6.1: changed=false, steps=1
  ReportsChangedWithoutStepsOnMemoHit             root2 z 6.1: changed=true,  steps=0
  KeepsNodeIdentityWhenNothingApplies             sameNode(wejście, wynik) (6.3)
  AppliesRulesInnermostFirst
  DiscardsUnsafeRuleAndContinues             odrzucona nie blokuje kolejnych

DAG
  PreservesSharingAcrossRewrite
  RewritesSharedSubtreeOnce                  licznik wywołań: 1, nie 77
  SharesMemoAcrossRootsInOneSession          drugi korzeń nie liczy od nowa (5.3)
  SurvivesRewriteOfTemporaryExpression       klucz Expression, nie wskaźnik (5.2)
  DoesNotReuseIncompleteMemoAfterStepLimit   wpisy przerwanego wywołania porzucone (5.5)
  CommitsMemoOnlyAfterReachingFixedPoint     druga strona tego samego inwariantu
  OutlivesTheRuleArrayItWasBuiltFrom         sesja z reguł w tymczasowym vectorze (3.1)

równoważność Z DZIEDZINĄ
  AgreesOnDefinednessAndValue                patrz niżej
  RewrittenKr640FkMatchesOriginal            end-to-end na prawdziwym FK
```

Czego na tej liście **nie ma i nie będzie**: testu na iteracyjny retry z 4.1.
Żeby rekurencyjna wersja przewróciła się na stosie, potrzeba reguły strzelającej
dziesiątki tysięcy razy z rzędu, a test, który to robi, kosztuje sekundy i zależy
od rozmiaru stosu na maszynie CI. To jest wymóg **strukturalny**, pilnowany przy
review kodu i zapisany w komentarzu przy pętli, a nie własność sprawdzana
automatem — lepiej powiedzieć to wprost, niż dopisać test, który przechodzi
również dla wersji rekurencyjnej.

**`AgreesOnDefinednessAndValue`** zastępuje `EquivalentOnSampledDomain` z v1 i
jest zbudowany inaczej, bo tamten zawierał błąd z 2.4:

```text
1. weź wyrażenie z dzieleniem o ZNANYM zerze mianownika, np. a/(q1 - 0.5)
2. przepisz
3. zbuduj listę punktów z DWOCH ZRODEL:
      a) DETERMINISTYCZNE punkty graniczne, wpisane ręcznie:
             q1 = 0.5          <- DOKŁADNE zero mianownika
             q1 = 0.5 +/- 1e-9, 0.5 +/- 1e-6
      b) losowe ze stałym ziarnem, w użytecznym zakresie
4. dla każdego punktu policz evaluate(przed) i evaluate(po)
5. wymagaj ZGODNOŚCI CO DO OKREŚLONOŚCI:
       obie zwracają wartość            -> wartości równe w granicach 1e-12
       obie zwracają DivisionByZero     -> zgoda
       jedna i druga inaczej            -> BŁĄD, dziedzina się zmieniła
```

**Punkt 3a jest tym, czego brakowało w v2, i bez niego test przechodzi mimo
błędu, przed którym ma chronić.** v2 kazała losować „blisko bieguna". Weźmy
kanoniczny przypadek `(1/q) * 0 -> 0`:

```text
q = 1e-6    przed: (1/1e-6)*0 = 0        po: 0     ZGODA
q = 1e-12   przed: (1/1e-12)*0 = 0       po: 0     ZGODA
q = 0       przed: DivisionByZero        po: 0     RÓŻNICA
```

Poza dokładnym zerem obie postacie są **równe**, więc losowanie — choćby
najbliższe bieguna — może przejść w stu procentach. Różnica dziedziny ujawnia
się **wyłącznie w punkcie osobliwym**, więc ten punkt musi być w teście **z
nazwiska**, a nie z losowania.

Nie budujemy przy tym żadnego szukacza zer mianownika: test operuje na
**kontrolowanym** wyrażeniu, więc jego biegun jest znany z konstrukcji i wpisuje
się go ręcznie. Automatyczne znajdowanie osobliwości to zadanie z F3.10, nie
z tego etapu.

Krok 5 jest drugą połową poprawki. v1 kazała **osobno** porównywać `divisorsOf`
przed i po — czyli robić dokładnie to, czego zabrania 2.4, bo `a/-(-q)` i `a/q`
mają różne dzielniki i tę samą dziedzinę. Test behawioralny mierzy **dziedzinę**,
a nie jej składniowy zapis.

---

## 9. Plan zmian w plikach

```text
NOWE   symbolic/RewriteRule.hpp        RewriteRule, RewriteResult, limit
       symbolic/RewriteEngine.hpp      divisorsOf, hasSameDomainGuards,
                                       RewriteSession, rewrite()
       symbolic/RewriteEngine.cpp      sesja, memo, guardy, miara
       tests/test_rewrite_engine.cpp   25 testów

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
| `changed`/`steps` mylące przy dzielonym memo | **realne, złapane w review v2** | Kontrakt obu pól zdefiniowany osobno, z przykładem obu patologii; `changed` przez `sameNode` (6) |
| Współdzielenie ginie przy przebiegu bez zmian | **realne** | Wymóg zachowania tożsamości węzła (6.3) — warunek konieczny dla `changed` i dla 5.1 |
| Test dziedziny przechodzi mimo rozszerzenia | **realne, złapane w review v2** | Deterministyczne punkty graniczne, w tym **dokładne** zero mianownika (8) |
| Deklarowana liniowość nieprawdziwa przez `divisorsOf` | **realne, złapane w review v2** | Drugi cache sesji, strażnicy liczeni kompozycyjnie (5.4) |
| Wynik przerwany limitem wyciekł do memo sesji | **realne, złapane w review v3** | Memo sesji transakcyjne: wyłącznie punkty stałe, commit na końcu wywołania (5.5), własny test |
| Limit porównany z niewłaściwym maksimum | **realne, złapane w review v1 i v3** | Zmierzone oba roboty; `kr4_r600` = 153 703 jest maksimum, limit `1 000 000` (11.2) |
| Rekurencyjny retry zabija proces przed limitem | **realne, złapane w review v3** | Krok 5 jest pętlą; rekurencja zostaje tylko po dzieciach (4.1) |
| Silnik melduje punkt stały, który nim nie jest | **realne, złapane w review v4** | Powrót po regule do przepisania dzieci, nie do próby reguł; kontrprzykład i test w 4.1 |
| Sesja z wiszącym `span`em reguł | **realne, złapane w review v4** | Sesja kopiuje reguły, `name` jest `std::string` (3.1), własny test |
| Cache strażników trzyma milion tymczasowych DAG-ów po limicie | **realne, złapane w review v4** | Ten sam reżim transakcyjny co memo (5.5) |
| Limit przedstawiony jako wyprowadzona granica | **realne, złapane w review v4** | 11.2: budżet defensywny; kontrprzykład na `steps ≤ measure(root)` wprost w dokumencie |
| Dwa niezależne analizatory dziedziny (wolna funkcja vs cache) | realne | Jedna implementacja, wolne funkcje jako opakowania (7.2) |
| Koszt porównywania strażników przy wielu dzielnikach | świadome, otwarte | Deklaracja `O(N)` wycofana do „raz na unikalny węzeł"; kanonizacja strażników zapisana jako kierunek (5.4) |

---

## 11. Otwarte pytania do review

**11.1 Zbiory czy multizbiory dzielników?** Rekomendacja: **zbiory**. `a/x + b/x`
ma jeden dzielnik występujący dwa razy, a dla dziedziny liczy się tylko `x ≠ 0`.
Multizbiór odrzucałby łączenie takich składników bez zysku dla bezpieczeństwa.

**11.2 Domyślny limit kroków.** Ta pozycja była poprawiana trzy razy i za każdym
razem z tego samego powodu — próby **wyprowadzenia** limitu z rozmiaru wejścia:

```text
v1   10 000    porownane z 281 UNIKALNYMI wezlami kr640
               ...a miara z 4.3 liczy Z KROTNOSCIAMI
v3  100 000    porownane z 21 882, czyli z kr640
               ...a kr640 NIE jest najwiekszym robotem w repo
v4  100 000 -> 1 000 000, porownane z kr4_r600 = 153 703
               ...a samo rozumowanie "steps <= measure(root)" NIE JEST prawdziwe
```

Trzecia poprawka jest inna od dwóch pierwszych: tam była zła liczba, tu jest zły
**dowód**. Przeciwprzykład dostarcza sam ten dokument, w 2.7:

```text
a / (q - q)

krok:   reguła q-q -> 0 zaakceptowana na poddrzewie      ++steps
        measure(q-q) spada 3 -> 1

składanie rodzica:
        representability guard odrzuca 0 w mianowniku
        Divide zostaje z ORYGINALNYM mianownikiem

wynik:  steps = 1, a measure(root) NIE ZMALAŁA
```

Krok został wykonany i policzony, a redukcja została wyrzucona przy skladaniu
rodzica. To ta sama sytuacja, którą 6.2 wykorzystuje do pokazania
`changed = false` przy `steps = 1` — więc albo dowód, albo tamten przykład; obu
naraz mieć nie można.

**Terminacja to nie to samo co granica na `steps`, i tylko ta pierwsza jest tu
potrzebna.** Terminacja stoi na argumencie **per węzeł** z 4.1: każdy obrót pętli
na danym węźle ściśle zmniejsza miarę **tego** węzła. Całkowita liczba kroków w
wywołaniu sumuje się po węzłach i **nie** jest ograniczona przez `measure(root)`.
Dałoby się pewnie skonstruować ostrożniejszy potencjał i wyprowadzić jakąś
granicę — tylko po co, skoro limit i tak nie ma być progiem pracy.

**Rozstrzygnięcie:**

```text
kDefaultRewriteStepLimit = 1 000 000
```

> Milion jest **defensywnym budżetem operacyjnym**. Nie jest matematyczną granicą
> wyprowadzoną z rozmiaru wejściowego DAG-a. Terminację zapewnia własny,
> dobrze ufundowany kontrakt miary każdej reguły (4.3); limit istnieje jako
> **niezależne** zabezpieczenie na wypadek złamania tego kontraktu.

Zmierzone rozmiary zostają w dokumencie, ale już jako **sanity check**, nie jako
dowód:

```text
kr640.urdf        21 882 węzłów z krotnościami
kr4_r600.urdf    153 703
```

Milion daje wobec dzisiejszych wejść duży zapas i to jest całe uzasadnienie tej
liczby. Kosztu nie zmienia żadnego: limit nie **wykonuje** miliona kroków, tylko
na tyle pozwala.

Limit tej wielkości jest za to powodem, dla którego pętla z 4.1 **musi** być
pętlą: milion zagnieżdżonych wywołań nie doczekałby limitu.

**11.3 Czy sesja ma przyjmować `ExpressionFactory` przez wartość?** Rekomendacja:
tak, jak robią to `JointTransformBuilder` i `ForwardKinematicsBuilder`. Fabryka
jest dziś bezstanowa; gdyby kiedyś zyskała stan (hash-consing), zmiana własności
będzie osobną decyzją, zapisaną już w `STATUS.md` jako znany brak.

---

## 12. Rekomendacja

Przyjąć: definicję z 2.1 wraz z jej jawnym ograniczeniem, rozdział gwarancji z
2.2, **lokalny** warunek z 2.3 z indukcją z 2.4, representability guard z 2.7,
sesję z 5.3 i klucz memo z 5.2.

Z v3 dochodzą trzy rozstrzygnięcia, wszystkie wymuszone przez sesję dzieloną
między korzeniami: kontrakt `changed`/`steps` z 6.2 wraz z wymogiem zachowania
tożsamości węzła z 6.3, deterministyczne punkty graniczne w teście dziedziny z 8
oraz drugi cache sesji z 5.4, bez którego deklaracja złożoności z 5.1 nie była
prawdziwa.

Z v4 dochodzi czwarte i to ono domyka sesję: **memo sesji jest transakcyjne**
(5.5), więc każdy jego wpis jest punktem stałym, a wywołanie przerwane limitem
nie zostawia po sobie niczego. Do tego pętla zamiast rekurencji (4.1) i wycofana
deklaracja `O(N)` (5.4).

Z v5 — poprawna definicja punktu stałego: po udanej regule silnik wraca do
**przepisania dzieci**, bo reguła może wprowadzić materiał, którego nikt jeszcze
nie znormalizował (4.1). Plus własność reguł przez sesję (3.1), transakcyjny
cache strażników (5.5) i limit przedstawiony jako budżet, nie jako wyprowadzona
granica (11.2).

Po zatwierdzeniu — proposal implementacyjny z pełnym kodem trzech nowych plików
i dwudziestoma pięcioma testami, gate `447/447` (422 + 25).

Po F3.1 wchodzi **F3.2** — pierwsze prawdziwe reguły: `x + 0`, `x * 1`, zwijanie
stałych, `q - q → 0`. To tam okaże się, czy guard z 2.7 był potrzebny — i to
tam, po raz pierwszy, ktoś będzie musiał dotrzymać (E).
