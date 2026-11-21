# Odpowiedź na raport integracyjny MotionBridge (WQM)

Status: **PROPOZYCJA — czeka na akceptację, nic nie zaimplementowano.**
Dotyczy: raport z 2026-11-21 (commit 6f54b2d), MotionBridge używa jądra solvera analitycznego jako drugiego silnika IK dla mapy obszaru roboczego KR640.

## 1. Prompt (skrót raportu)

MotionBridge generuje mapę WQM (siatka walcowa r,z,θ, setki tysięcy komórek, stała orientacja). Dotychczasowy solver iteracyjny (LM, 130 startów) daje zaszumione wyniki blisko osobliwości i łamie niezmiennik „jedna postawa na komórkę". Oczekiwania, w kolejności ważności:

1. rodzina osobliwości overhead (q1 swobodne) jako `PositionalFamily`,
2. opisane i przetestowane zachowanie w paśmie okołoosobliwym + miara bliskości osobliwości,
3. pomiar czasu wywołania i obiekt „prepared" (jednorazowe przygotowanie, wielokrotne użycie),
4. osobny target jądra bez klas kolidujących nazwami z gospodarzem,
5. udokumentowana kolejność gałęzi,
6. (opcjonalnie) jakobian / miara uwarunkowania.

Pozytywny wynik raportu: 60/60 celów odzyskuje konfigurację wyjściową, więc konwencje (RPY, zero stawów, limity, ramki) są zgodne z KR640.

## 2. Weryfikacja twierdzeń raportu w repo (stan na main 6f54b2d)

| Punkt | Twierdzenie | Wynik sprawdzenia |
|---|---|---|
| 5.1 | `kinemaforge_ik` kompiluje `Kinematics.cpp`, `kinematics/robot_model*` i wystawia `src` jako PUBLIC | **Potwierdzone** (`CMakeLists.txt:86-115`). Do tego target zawiera też warstwę symboliczną, generator kodu i `UrdfModelLoader`, więc jądro solvera nie jest osobnym targetem. |
| 5.2 | Rodzina overhead nieobsłużona | **Potwierdzone** (`STATUS.md`, Known gaps; `UnsupportedKinematicStructure`, generated: `UnsupportedSingularity`). To jest też jedyna otwarta pozycja przed v1.0. |
| 5.3 | Pasmo osobliwości bardzo wąskie | **Potwierdzone** (`IkTolerances.hpp`: `kSingularLinear` = 1e-9 m, `kSingularAngular` = 1e-10 rad). Zachowanie w paśmie 1–20 mm nie jest w testach opisane — do zmierzenia. |
| 5.4 | Brak gwarancji kolejności gałęzi | **Potwierdzone** w sensie: w solverze nie ma jawnego sortowania wyniku; kolejność wynika z konstrukcji i nie jest kontraktem. |
| 5.5 | Brak pomiarów czasu i obiektu „prepared" | Do potwierdzenia przy implementacji (nie znalazłem benchmarków). `solveCompact` istnieje (`AnalyticIkSolver.hpp:75`), `maxBranches` = 4096. |
| 5.6 | Brak jakobianu | Zgodne z zakresem biblioteki. |

Raport słusznie zaznacza, że sam nie zweryfikował 5.2 wywołaniem i nie mierzył 5.3/5.5. Nie ma tu rozbieżności z kodem.

## 3. Proponowane pozycje (kolejność = ważność dla MotionBridge, nie koszt)

| # | Pozycja | Rozmiar | Uwagi |
|---|---|---|---|
| A | **Rodzina overhead** jako `PositionalFamily` (q1 swobodne, zakres, reszta przez `evaluate`, sprawdzanie FK w próbkach) w solverze ogólnym **i** w generated runtime; w razie potrzeby też rodzina elbow-folded | duży | Domyka jedyną pozycję otwartą przed v1.0. Własna propozycja z pełnym kodem. |
| B | **Pasmo okołoosobliwe**: testy zachowania dla środka nadgarstka w 1/5/20 mm od osi A1 oraz dla |sin q5| w analogicznym paśmie (ciągłość, dokładność, próg); udostępnienie miary bliskości na gałąź (odległość środka nadgarstka od osi, |sin q5|) | średni | Najpierw pomiar, dopiero potem decyzja, czy progi w `IkTolerances.hpp` wymagają zmiany. Każda zmiana progu musi zachować `static_assert`-y z drabiny tolerancji. |
| C | **Prepared solver**: niemutowalny obiekt (łańcuch + geometria + wykryty wzorzec), `solve` const, bezpieczny wielowątkowo; benchmark jednego wywołania na KR640 | średni | Zależy od zakresu A (żeby nie projektować API dwa razy). |
| D | **Target `kinemaforge_ik_core`**: bez `Kinematics.cpp`, `kinematics/robot_model*`, `UrdfModelLoader.cpp`, bez PUBLIC `src/common` i `src/kinematics`; publiczne nagłówki pod `include/kinemaforge/...` | średni | Raport sprawdził, że w `ik_equations` jedynym plikiem dołączającym `kinematics/robot_model_loader.hpp` jest `UrdfModelLoader.cpp`, więc podział jest wykonalny. Do potwierdzenia: czy `model/` i `builders/` nie ciągną `common/`. Zmiana nagłówków i namespace'ów dotyka wielu plików — wymaga osobnej, ostrożnej propozycji. |
| E | **Kontrakt kolejności**: jawne, deterministyczne sortowanie (Shoulder, Elbow, Wrist, potem wartości stawów) + test + dokumentacja `solve` vs `solveCompact` | mały | Zmienia kolejność wyniku, więc test regresji musi to obejmować. Warto zrobić razem z C. |
| F | **Jakobian / wartości osobliwe** | mały–średni | Opcjonalne, na końcu. |

Poza zakresem tej propozycji (po stronie MotionBridge): przekazywanie rodzin zamiast odrzucania gałęzi z `singularity`, przetestowanie ścieżek błędów na prawdziwych danych, kompilacja z `-Werror`.

## 3a. DECYZJE (2026-11-21, po odpowiedzi MotionBridge)

**Zatwierdzona kolejność: benchmark → A → E → D → B → C → F.**

1. **Benchmark jednego wywołania (KR640) idzie pierwszy i osobno.** To pomiar, nie funkcja. Jego wynik może zmienić priorytet obiektu „prepared" (C).
2. **A i E wchodzą do v1.0 RC.** E tym bardziej teraz, bo MotionBridge nie ma jeszcze żadnej mapy zapisanej z naszą kolejnością, więc zmiana nic nie kosztuje.
3. **D zaraz po E** (przed B). MotionBridge kopiuje dziś nagłówki jako obejście; zmiana nazw targetów i ścieżek nagłówków jest tym tańsza dla nich, im wcześniej nastąpi.
4. **Elbow-folded: osobno i później.** To osobliwość pełnego wyprostowania, na granicy zasięgu; MotionBridge jej nie potrzebuje. Uwaga na nazewnictwo: „Elbow" w nazwie komórki raportu to klasa postawy mapy (pojedyncza degeneracja nadgarstka), a nie osobliwość łokcia złożonego.
5. **B przed C** (B dotyczy poprawności, C wydajności).

Konsekwencja dla A: zakres to wyłącznie rodzina overhead (q1 swobodne). Jedyna otwarta pozycja przed v1.0 z `STATUS.md` zostaje wtedy domknięta w części „shoulder-axis"; elbow-folded zostaje w Known gaps jako świadoma granica wydania.

## 3b. WYNIK BENCHMARKU (2026-11-21, punkt „benchmark" z 3a)

Warunki: KR640, 60 losowych konfiguracji (ziarno 0x5005, zakres 5–95% limitów), 50 powtórzeń na cel (3000 wywołań na wiersz), jeden wątek, Release `-O3`, MinGW GCC 13.1, po rozgrzewce. Kod w `CLionProjects\_claude_scratch\bench\bench.cpp` (nie jest częścią repo).

| Ścieżka | µs / wywołanie |
|---|---|
| A: zbudowanie łańcucha + `AnalyticIkSolver::solve` (tak robi dziś adapter MotionBridge) | 37,6 |
| B: `solve`, łańcuch zbudowany raz | 33,9 |
| C: `solveCompact`, łańcuch zbudowany raz | 16,2 |
| D: wygenerowany runtime `solve` | 12,6 |
| E: wygenerowany runtime `solveCompact` | 7,1 |

Średnio 16,3 gałęzi na cel w tej próbce (raport podawał 14,7; inna próbka, ten sam rząd wielkości).

Wnioski:
- Budowanie łańcucha przy każdym wywołaniu kosztuje tylko ~3,7 µs (10%). **Samo „przygotowanie łańcucha" nie da dużego zysku.** Jeśli obiekt „prepared" (C) ma coś dać, to przez wykrywanie wzorca i geometrię w środku `solve`; ile to z 33,9 µs, trzeba dopiero zmierzyć profilem.
- Największy zysk dają `solveCompact` (2,1×) i wygenerowany runtime (2,7× względem `solve`). Ścieżka E jest ok. 5× szybsza niż A.
- Dla skali mapy: 500 000 komórek × 37,6 µs ≈ 19 s w jednym wątku (ścieżka A), ≈ 3,5 s (ścieżka E). Czas wywołania nie jest więc wąskim gardłem generacji mapy.
- Wniosek dla priorytetów: **C (obiekt prepared) spada na niski priorytet**; zostaje sensowny tylko jako bezpieczeństwo wątkowe i czystość API, a nie jako optymalizacja. Gdyby MotionBridge potrzebował szybkości, tańsza jest zmiana na `solveCompact` albo na wygenerowany kod.
- Czego ten pomiar nie pokazuje: zachowania w pobliżu osobliwości (tam liczba gałęzi i ścieżki kodu mogą się różnić), wielowątkowości ani kosztu `expand` przy zamianie compact → pełna lista.

## 3c. KOREKTA: pozycja A była już zaimplementowana; wynik pomiaru pasma (pozycja B)

**A.** Raport MotionBridge oparł się na nieaktualnym wpisie Known gaps w `STATUS.md`. Rodziny `ShoulderAxis` i `ElbowFolded` są w kodzie od follow-upu 2026-11-16 #1 (commit 6f54b2d): `PositionalFamily`, `AnalyticIkSolution::positionalFamilies`, `evaluate(family, value)`, status `InfiniteFamily`, w solverze ogólnym i w wygenerowanym runtime, z 5 testami. Niezaimplementowany jest tylko przypadek dwuparametrowy (oś barku I złożony łokieć naraz). Nie dopisywałem więc drugiej implementacji; poprawiłem `STATUS.md`.

**Ważne dla adaptera MotionBridge:** rodziny leżą w `positionalFamilies`, **nie w `branches`**. Adapter, który czyta tylko `branches` i odrzuca te z polem `singularity`, nigdy ich nie zobaczy. Przy `status == InfiniteFamily` lista `branches` może być pusta.

**B (pasmo okołoosobliwe).** Pomiar KR640, środek nadgarstka w zadanej odległości od osi A1 (cel z kinematyki prostej konfiguracji, więc rozwiązanie znane):

| odległość od osi [m] | status | gałęzie / rodziny | błąd q1 konfiguracji wyjściowej [rad] | wyjściowa odzyskana |
|---|---|---|---|---|
| 0, 1e-12, 1e-10 | InfiniteFamily | 0 / 2 | ≤ 5e-12 | tak (w rodzinie) |
| 1e-9 | Solutions | 8 / 0 | 6e-8 | tak |
| 1e-8 | Solutions | 8 / 0 | 1,5e-8 | tak |
| 1e-6 … 2e-2 (1 µm … 20 mm) | Solutions | 8 / 0 | ≤ 1e-10 | tak |

Wygenerowany runtime daje te same statusy. Przejście jest w `kSingularLinear` = 1e-9 m i jest ciągłe w sensie reprezentacji: po obu stronach konfiguracja wyjściowa jest odzyskiwana, żadna gałąź nie „wypada". Dla komórek mapy w odległości milimetrów od osi (to jest realne pasmo mapy) wynik jest zwykły, bez szumu iteracyjnego, z dokładnością q1 ≤ 1e-10 rad. Obawa z p. 5.3 raportu o skokowe „wpadanie i wypadanie" gałęzi nie potwierdziła się. Zachowanie jest teraz przypięte testem `NearTheShoulderAxisTheGeneratingConfigurationIsAlwaysRecovered`.

Co zostaje z B: **miara bliskości osobliwości na gałąź** (odległość środka nadgarstka od osi, |sin q5|) jako publiczne API. Nie jest zaimplementowana; adapter może policzyć ją sam z własnej kinematyki prostej.

## 3d. WYKONANIE (2026-11-21)

Wszystkie pozycje z 3a zrealizowane test-first (najpierw czerwone, potem zielone), poza E: tam kod już spełniał kontrakt, więc testy były zielone od początku i dopisano tylko kontrakt w `AnalyticIkSolver.hpp`. Bramka: 710/710 w Release, Debug i `KINEMAFORGE_WERROR=ON`. Szczegóły i tabela wyników w `STATUS.md` (Stage F3.M).

Pomiar po zmianach (te same 60 celów; wartości drżą o ok. 10–20 % między uruchomieniami): `solve` z budowaniem łańcucha 36–49 µs, `solve` z gotowym łańcuchem 33–42 µs, `PreparedAnalyticIk::solve` ok. 38 µs w tym samym przebiegu co 42 µs dla gotowego łańcucha, `solveCompact` 16–19 µs, `PreparedAnalyticIk::solveCompact` ok. 16 µs, generated 13–15 µs / 7–9 µs, jakobian z SVD ok. 3 µs. Obiekt „prepared" oszczędza więc tylko kilka–kilkanaście procent; największy zysk daje `solveCompact` lub wygenerowany runtime.

Dla MotionBridge: (1) rodziny czytać z `positionalFamilies`, nie z `branches`; (2) `shoulderAxisDistance` i `wristPitchSine` zastępują własny próg bliskości; (3) w CMake wystarczy `kinemaforge::ik_core` (bez pugixml); nagłówki leżą nadal pod `ik_equations/...`, a klasy dawnego `mt::kinematics` pod `legacy/...` i nie są częścią rdzenia; (4) indeks gałęzi nie jest stabilny między celami, kluczem do zapisu powinny być etykiety (shoulder, elbow, wrist) i wartości stawów.

## 4. Do rozstrzygnięcia przed implementacją (stan pierwotny — rozstrzygnięte w 3a)

1. **Kolejność.** Proponuję A → E → B → C → D → F. Alternatywa: D wcześniej, jeśli blokuje MotionBridge bardziej niż brak rodziny overhead (dziś obchodzą to kopiowaniem nagłówków).
2. **Zakres A.** Czy rodzina elbow-folded wchodzi razem z overhead, czy osobno? (STATUS wymienia ją jako „still open".)
3. **Wersja.** Czy A i E mają wejść do v1.0 release candidate, czy lecimy jako 1.1? E zmienia kolejność wyniku, a MotionBridge utrwala ją w pliku mapy, więc im wcześniej, tym taniej.

## 5. Następny krok

Po akceptacji kolejności napiszę osobną propozycję dla pierwszej pozycji (opis, stan obecny, pełny nowy kod). Do tego czasu nie ruszam kodu.
