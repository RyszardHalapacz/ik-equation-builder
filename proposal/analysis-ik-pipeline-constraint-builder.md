# Analiza architektoniczna: gdzie w pipeline'ie żyją ograniczenia orientacji TCP?

## Status dokumentu

To **nie jest proposal zmiany** w formacie ustalonym dla tego repo — to analiza architektoniczna poprzedzająca ewentualny przyszły proposal, zgodnie z wyraźną prośbą ("chciałbym przeanalizować, zanim przejdziemy dalej z implementacją"). Bez kodu, bez plików `.hpp`/`.cpp`, bez implementacji — wyłącznie odpowiedzi na 6 zadanych pytań i rekomendacja podziału odpowiedzialności.

## Kontekst: dlaczego to pytanie jest trafne w tym momencie

Obecny, częściowo zaimplementowany pipeline:

```
URDF → UrdfModelLoader → RobotDescription
     → KinematicChainBuilder → KinematicChain
     → JointTransformBuilder (per joint) → SymbolicTransform
     → ForwardKinematicsBuilder → Symboliczne FK (jedna macierz 4×4, funkcja q)
```

`ForwardKinematicsBuilder` ma już zadeklarowany kontrakt (`SymbolicTransform build(const KinematicChain&, const JointTransformBuilder&) const`) i README wprost go opisuje jako: *"multiplies the whole chain of transforms together into one final symbolic transform."* Nic więcej. Zero pojęcia o "celu", "zadaniu", "ograniczeniu orientacji" — i to jest kluczowe dla całej dalszej analizy.

Ważna obserwacja upraszczająca: **dane potrzebne do ograniczeń orientacji już istnieją w FK** — `SymbolicTransform` to macierz 4×4, gdzie górny-lewy blok 3×3 to `R(q)` (orientacja), a prawa kolumna to `P(q)` (pozycja). Nie potrzeba żadnej zmiany w sposobie liczenia FK, żeby mieć dostęp do orientacji — potrzeba czegoś, co **z tego, co już policzone, zbuduje właściwy układ równań względem zadanego celu**. To przesądza kierunek odpowiedzi na wszystkie 6 pytań.

---

## 1. Czy obecny pipeline jest wystarczający, czy brakuje etapu między symbolicznym FK a solverem IK?

**Brakuje etapu.** Nie dlatego, że symboliczna warstwa (`Expression`/`SymbolicMatrix`) czegoś nie potrafi wyrazić — potrafi (odejmowanie, więc `Px(q) - target.x` jest trywialne do zbudowania z istniejących `ExpressionFactory`). Brakuje **miejsca w architekturze**, którego odpowiedzialnością jest ta budowa.

Dziś FK to czysta funkcja `q → pose`, bez żadnego pojęcia o tym, co ma być rozwiązane. Solver IK potrzebuje czegoś fundamentalnie innego: **układu równań względem konkretnego celu** — `Px(q) - Px_target = 0`, ewentualnie tylko wybranych składowych orientacji, ewentualnie z jednym stopniem swobody celowo zostawionym wolnym (obrót wokół osi narzędzia). To jest osobny rodzaj wiedzy: nie "jak porusza się robot", tylko "co akurat chcemy, żeby robot osiągnął". Mieszanie tych dwóch bytów w jednym komponencie byłoby błędem — patrz pytanie 3.

README już zresztą zakłada dalsze etapy (`EquationSimplifier`, `EquationSolver`, `IkPatternDetector`, `CodeGenerator`), ale żaden z nich w opisie nie odpowiada wprost za **sformułowanie** układu równań z FK + celu — `EquationSolver` w opisie "rozwiązuje", zakładając, że układ już istnieje. To jest dokładnie ta brakująca luka.

## 2. Czy warto wprowadzić osobny komponent (`ConstraintBuilder`/`EquationBuilder`)?

**Tak.** Rekomendowana nazwa: **`ConstraintBuilder`** — `EquationBuilder` kolidowałoby pojęciowo z istniejącą fasadą `IkEquationBuilder` (myląca bliskość nazw dla kogoś czytającego kod pierwszy raz).

Odpowiedzialność (w zakresie):
- przyjmuje symboliczne FK (`SymbolicTransform`, wynik `ForwardKinematicsBuilder`) oraz **specyfikację zadania** (co ma być ograniczone i względem czego — patrz pytanie 4),
- wyodrębnia z FK potrzebne składowe (pozycję, pełną orientację, wybraną oś orientacji — to tylko *odczyt* istniejących komórek macierzy, żadna nowa matematyka symboliczna),
- składa je z celem w konkretne równania (`Expression`, przez istniejący `ExpressionFactory` — `subtract`, ewentualnie `dot`/iloczyn skalarny, jeśli trzeba dodać taką operację do `ExpressionFactory` przy okazji implementacji — to jedyne miejsce, gdzie być może brakuje jednej operacji w warstwie symbolicznej, nie architektury),
- zwraca uporządkowany zestaw równań gotowy dla kolejnego etapu.

Poza zakresem: nie upraszcza równań (to `EquationSimplifier`), nie rozwiązuje (to `EquationSolver`), nie liczy FK (to już zrobione), nie zna nic o konkretnym procesie technologicznym (spawanie/malowanie — patrz pytanie 5).

## 3. Czy taki podział jest lepszy niż logika wymuszonej orientacji wprost w `ForwardKinematicsBuilder`?

**Zdecydowanie tak**, z czterech niezależnych powodów:

1. **Naruszenie już ustalonej, węższej odpowiedzialności.** `ForwardKinematicsBuilder` ma jedno zadanie: policzyć pozę jako funkcję `q`. To jest własność **robota**. "Jaką orientację chcemy wymusić" to własność **zadania/procesu** — zupełnie inna oś zmienności, zmienia się niezależnie od robota i niezależnie od tego, czy FK jest poprawnie policzone.
2. **Utrata reużywalności FK.** Ten sam symboliczny FK dla jednego robota powinien dać się użyć do wielu różnych zadań (dziś spawanie z wymuszoną osią, jutro chwytak z pełną orientacją, pojutrze operacja tylko pozycyjna) — bez ponownego liczenia FK. Gdyby logika ograniczeń siedziała w `ForwardKinematicsBuilder`, każda zmiana zadania wymagałaby albo przebudowy FK, albo rozrostu parametrów tej klasy do obsługi wszystkich wariantów na raz.
3. **Zgodność z już przyjętą filozofią projektu.** README wprost: *"Each stage owns exactly one responsibility [...] future stages [...] can be added without changing the earlier pipeline."* Nowy komponent jest **czystym dodaniem** — zero zmian w `UrdfModelLoader`, `KinematicChainBuilder`, `JointTransformBuilder`, `ForwardKinematicsBuilder`. To dokładnie test tej deklarowanej właściwości, i przechodzi go.
4. **Testowalność.** Testowanie "czy poprawnie buduję równanie wyrównania osi" nie powinno wymagać za każdym razem wyprowadzania pełnego FK konkretnego robota — przy osobnym komponencie można testować logikę ograniczeń na uproszczonym/spreparowanym FK, niezależnie od poprawności derywacji geometrii.

## 4. Jak taki komponent powinien wyglądać koncepcyjnie?

**Wejście:**
- `SymbolicTransform` — FK, funkcja `q1..qn`.
- **Specyfikacja zadania** — nowy koncept, nieistniejący dziś w modelu danych. Powinien rozróżniać *kształt* ograniczenia (nie proces technologiczny — patrz pytanie 5), np.:
  - `PositionOnly` — 3 równania, `P(q) = target`.
  - `FullPose` — pozycja + pełna orientacja.
  - `AxisAlignment` — pozycja + jedna oś narzędzia wymuszona na kierunek (przypadek "dysza w dół", "kamera w kierunku ruchu").
  - `PartialOrientation` — dowolny podzbiór składowych orientacji, dla przypadków niestandardowych.
- Sam **cel** (target) — punkt/kierunek/macierz orientacji względem której budowane są równania. To nie pochodzi z URDF ani z FK — to zewnętrzny parametr zadania, analogicznie do tego, jak `baseLink`/`toolLink` są zewnętrznym parametrem dla `KinematicChainBuilder`.

**Wyjście:** uporządkowany zestaw równań symbolicznych (`Expression`), każde postaci "coś policzonego z FK minus cel". To wystarczająco różne od `KinematicChain` czy `SymbolicTransform`, żeby zasługiwać na własny model danych pośredni — analogicznie do uzasadnienia, dlaczego istnieje `KinematicChain`, a nie przekazywanie `RobotDescription` wprost (ten sam argument stosuje się rekurencyjnie: kolejny etap nie powinien znać FK ani zadania, tylko gotowy układ równań). Roboczo: `IkEquationSystem` — lista równań + referencja do tego, które `qN` są niewiadomymi (już znane z `KinematicChain`).

**Istotna uwaga matematyczna, do rozstrzygnięcia przy właściwym proposalu implementacyjnym (nie tutaj):** macierz rotacji `R(q)` ma 9 wpisów symbolicznych, ale rotacja właściwa ma tylko 3 niezależne stopnie swobody (SO(3)) — porównanie `R(q) = R_target` "po współrzędnych" (9 równań) jest nadmiarowe/częściowo zależne. Podobnie wyrównanie jednej osi narzędzia do kierunku to **2 niezależne równania**, nie 3 — bo kierunek na sferze S² ma 2 stopnie swobody, a trzeci (obrót wokół samej osi narzędzia) zostaje świadomie wolny (dokładnie przypadek "dysza w dół" — robot ma nadmiarowy stopień swobody wokół osi Z narzędzia). `ConstraintBuilder` musi mieć jawną strategię reprezentacji ograniczeń orientacji (surowe wpisy macierzy vs. iloczyny skalarne osi vs. inna parametryzacja) — to jedno z ważniejszych pytań otwartych do rozstrzygnięcia w przyszłym proposalu implementacyjnym tego komponentu, nie coś do zdecydowania w tym dokumencie.

**Ważne rozgraniczenie zakresu:** `ConstraintBuilder` odpowiada za ograniczenia **przestrzeni zadania** (pozycja/orientacja TCP — funkcje `q`, równości). To **nie** to samo, co ograniczenia **przestrzeni złączowej** (limity `q` z `KinematicJoint::limits` — nierówności na samych zmiennych, niezależne od FK). Te drugie prawdopodobnie trafią do `EquationSolver` bezpośrednio z `KinematicChain`, bez przechodzenia przez `ConstraintBuilder` — inaczej komponent traci jedną, jasną odpowiedzialność.

## 5. Czy taki podział będzie skalowalny dla różnych robotów i procesów (spawanie, malowanie, paletyzacja, montaż)?

**Tak, pod jednym warunkiem: `ConstraintBuilder` musi mówić językiem geometrii, nie językiem procesu.**

- Różne roboty: `ConstraintBuilder` operuje wyłącznie na `SymbolicTransform`/`KinematicChain`, czyli na bytach już niezależnych od konkretnego robota (to jest cały sens pipeline'u napędzanego URDF-em) — nie potrzebuje wiedzieć, czy to KR4 czy KR640.
- Różne procesy: proces technologiczny **wybiera**, który kształt ograniczenia (`PositionOnly`/`FullPose`/`AxisAlignment`/...) i jaki target zastosować — nie definiuje nowego kształtu API. "Spawanie" to nie metoda na `ConstraintBuilder`, tylko konkretne użycie `AxisAlignment` z konkretnym targetem. Mapowanie proces → wybór ograniczenia + target może kiedyś żyć jako osobna, cienka warstwa "presetów" nad `ConstraintBuilder` (np. biblioteka konfiguracji, niekoniecznie kod) — ale to świadomie **nie** jest częścią tego komponentu.

Ryzyko do pilnowania przy przyszłej implementacji: pokusa dodawania metod w stylu `weldingConstraint()`/`paintingConstraint()` wprost do `ConstraintBuilder`. To by złamało skalowalność — każdy nowy proces wymagałby zmiany w rdzeniu zamiast bycia zwykłym wywołaniem istniejącego, generycznego API z innymi parametrami.

## 6. Docelowy pipeline — od URDF do analitycznego IK

```
URDF
 → UrdfModelLoader              → RobotDescription
 → KinematicChainBuilder        → KinematicChain
 → JointTransformBuilder (×n)   → SymbolicTransform (per joint)
 → ForwardKinematicsBuilder     → SymbolicTransform (FK, funkcja q1..qn)
 → ConstraintBuilder  [NOWY]    → IkEquationSystem
      (FK + specyfikacja zadania: PositionOnly / FullPose / AxisAlignment / PartialOrientation + target)
 → EquationSimplifier  [już w planie README]  → uproszczony IkEquationSystem
 → EquationSolver      [już w planie README]  → analityczne rozwiązanie q(target)
 → IkPatternDetector   [już w planie README]  → rozpoznane wzorce strukturalne (np. spherical wrist),
                                                  prawdopodobnie wspiera/przyspiesza EquationSolver,
                                                  a nie działa ściśle "po" nim — dokładna kolejność
                                                  względem EquationSolver to temat osobnej analizy,
                                                  nie tego dokumentu
 → CodeGenerator       [już w planie README]  → wygenerowany kod dla konkretnego robota+zadania
```

`ConstraintBuilder` wchodzi w jedną, konkretną, wcześniej pustą lukę — między już zaimplementowanym/zaprojektowanym FK a już zaplanowanymi (ale jeszcze nieopisanymi w tym szczególe) etapami upraszczania/rozwiązywania. Nie wymaga żadnej zmiany w czterech wcześniejszych etapach.

**Konsekwencja dla fasady (`IkEquationBuilder`):** dziś `selectChain(baseLink, toolLink)` przyjmuje tylko geometrię łańcucha. Docelowo fasada będzie potrzebować drugiego, niezależnego wejścia — specyfikacji zadania/celu (np. przyszła metoda w stylu `defineTask(...)`) — zanim będzie mogła zawołać `ConstraintBuilder`. To nie wymaga decyzji teraz (fasada nie ma dziś żadnej realnej implementacji poza konstruktorem), ale warto mieć to na uwadze przy projektowaniu jej właściwego API w kolejnym kroku.

## Podsumowanie rekomendacji

| Pytanie | Odpowiedź |
|---|---|
| 1. Czy brakuje etapu? | Tak — między FK a solverem, na budowę układu równań względem celu. |
| 2. Czy wprowadzić `ConstraintBuilder`? | Tak. |
| 3. Czy lepiej niż w `ForwardKinematicsBuilder`? | Tak — inna oś odpowiedzialności (robot vs. zadanie), inna reużywalność, zgodność z filozofią projektu. |
| 4. Jak wygląda koncepcyjnie? | FK + specyfikacja zadania → `IkEquationSystem` (nowy model pośredni); ograniczenia orientacji jako osobna, przemyślana reprezentacja (nie naiwne 9 równań na `R(q)=R_target`). |
| 5. Czy skalowalne na roboty/procesy? | Tak, pod warunkiem trzymania API w języku geometrii, nie procesu — proces to tylko wybór parametrów. |
| 6. Docelowy pipeline? | Jak w sekcji 6 — jedno nowe ogniwo, zero zmian w istniejących czterech etapach. |

## Sugerowany następny krok

Jeśli ten kierunek jest akceptowalny: pełny proposal architektoniczny dla `ConstraintBuilder` w tym samym formacie co proposal `KinematicChainBuilder` (odpowiedzialność / wejście-wyjście / struktury danych / algorytm / błędy / API / testy / granice), w tym rozstrzygnięcie kwestii reprezentacji ograniczeń orientacji zasygnalizowanej w pytaniu 4.
