# Proposal: stała transformacja TCP — architektura (v3)

## Prompt

> Zaprojektować obsługę stałej transformacji od końca wybranego łańcucha kinematycznego do rzeczywistego TCP: `T_base_tcp(q) = T_base_chain_tip(q) · T_chain_tip_tcp`. Nie hardkoduj nazwy `tool0`. Wyłącznie dokument architektoniczny.

Etap F2.1 roadmapy Fazy 2. **Rewizja v3** po dwóch rundach `REQUEST CHANGES` — trzy blockery łącznie i kilkanaście brakujących kontraktów. Wszystkie przyjęte; co się zmieniło: §16 (v3) i §17 (v2).

## Status weryfikacji

**Ten dokument nie zawiera kodu produkcyjnego i niczego nie kompilowałem.** Stan repozytorium sprawdzony w plikach.

Jedno ustalenie z review **zweryfikowałem i okazało się mocniejsze**, niż zostało zgłoszone — §6.1.

---

## 1. Cel i zakres

```
T_base_tcp(q) = T_base_chain_tip(q) · T_chain_tip_tcp
```

`T_chain_tip_tcp` jest **stała** — niezależna od `q`. Wynik pozostaje `SymbolicTransform`.

**W zakresie:** model danych, walidacja, budowa jako `SymbolicTransform`, złożenie przez istniejące `multiplyTransforms`, integracja z fasadą, graf zależności i unieważnianie, model błędów, plan walidacji numerycznej.

**Poza zakresem** — §13.

---

## 2. Aktualny stan repozytorium

`HEAD = 224a9b7`. **Faza 1 zaimplementowana i zacommitowana**, `STATUS.md` zgodny z rzeczywistością: `ctest` daje **221/221**.

**Rozbieżność w liście plików z promptu:** `KinemaForge_IkEquationBuilder_Roadmap.md` **nie istnieje**. Jedyny dokument roadmapowy to `proposal/KinemaForge_Phase2_Phase3_Roadmap.md`, **nieśledzony przez gita**.

### 2.1 Fasada dziś

Pięć metod (trzy operacje `std::expected`, dwa akcesory wskaźnikowe), trzy pola `std::optional`, cztery kody błędu plus `std::optional<KinematicChainError>` przy `ChainBuildFailed`.

### 2.2 Warstwa symboliczna

`hasCanonicalHomogeneousLastRow`, `isIdentityTransform`, `multiplyTransforms` — z pięcioma fast pathami, w tym `if (isIdentityTransform(rhs)) return lhs;`. **Tożsamościowy TCP nie doda ani jednego węzła** i nie wymaga nowego kodu.

### 2.3 Gdzie mieszka konwersja RPY

W anonimowej przestrzeni `JointTransformBuilder.cpp`: `buildPrincipalRotation` (31), `buildRpyRotation` (67), `toSymbolicVector` (192), `assembleTransform` (215). `assembleTransform` **istnieje już w dwóch kopiach** — druga w anonimowej przestrzeni `SymbolicTransform.cpp`.

### 2.4 Referencja testowa

`tests/support/NumericForwardKinematics` ma `fromRollPitchYaw`, `compose`, `RigidTransform`. Rozszerzenie o TCP to jedno złożenie.

---

## 3. Równanie i semantyka frame'ów

```
R_base_tcp = R_base_tip(q) · R_tip_tcp
p_base_tcp = p_base_tip(q) + R_base_tip(q) · p_tip_tcp
```

**Translacja TCP jest wyrażona w układzie końcówki, nie w bazie.** Implementacja dodająca `p_tip_tcp` wprost do `p_base_tip` byłaby poprawna wyłącznie przy jednostkowym `R_base_tip` — przechodziłaby test w konfiguracji zerowej i failowała wszędzie indziej. Stąd §10.4.

Nazwa `tool0` nie występuje nigdzie: końcówką jest to, co wybrał użytkownik przez `selectChain`.

---

## 4. Model danych

### 4.1 Nazwa i ogólność

| Wariant | Za | Przeciw |
|---|---|---|
| `TcpTransform` | nazwa mówi, do czego służy | matematyka nie ma nic wspólnego z TCP; pierwszy inny stały offset wymusi drugi identyczny typ |
| **`FixedRigidTransform`** | opisuje to, czym jest | ogólna nazwa zaprasza do ogólnego użycia |
| reużycie `JointOrigin` | istnieje, ma `{translation, rpy}` | wiąże TCP z modelem URDF; nazwa kłamie w miejscu użycia |

**Rekomendacja: `FixedRigidTransform`**, w `src/ik_equations/model/FixedRigidTransform.hpp`.

### 4.2 Semantyka kierunku i jednostek — **wymagana przez review**

Ogólna nazwa bez ustalonego kierunku jest pułapką: ten sam typ da się użyć jako `T_tcp_tip` zamiast `T_tip_tcp`, a **kompilator niczego nie wykryje**. Kontrakt w komentarzu typu:

```
FixedRigidTransform reprezentuje T_parent_child.

translation : położenie początku układu `child` wyrażone w układzie `parent`
              jednostka: metry
rpy         : orientacja `child` względem `parent`
              jednostka: radiany
              R = Rz(yaw) · Ry(pitch) · Rx(roll)   (konwencja stałoosiowa URDF)
```

Dla TCP: `parent` = koniec aktualnego łańcucha, `child` = TCP.

### 4.3 Reprezentacja rotacji

| Wariant | Walidacja |
|---|---|
| **translacja + RPY** | wyłącznie skończoność sześciu liczb |
| translacja + kwaternion | skończoność **plus norma jednostkowa z tolerancją** |
| macierz | skończoność, ortogonalność, `det = +1`, obie z tolerancją |

**Rekomendacja: translacja + RPY.** Argument rozstrzygający jest o walidacji, nie o ergonomii: **RPY jest reprezentacją totalną** — każda szóstka skończonych liczb opisuje poprawną transformację sztywną. Kwaternion i macierz wymagają progu, a próg staje się częścią semantyki i wymaga własnego uzasadnienia; projekt takich decyzji nie podejmuje mimochodem.

Wejście kwaternionowe można później dodać jako **konwersję na granicy API**, nie jako drugi kształt składowanego stanu.

### 4.4 `Vector3` — ekstrakcja wymagana przez review

`FixedRigidTransform` potrzebuje `Vector3`, a ten jest dziś zadeklarowany w **`model/UrdfJoint.hpp`**. Bez ekstrakcji `FixedRigidTransform.hpp` musiałby włączyć `UrdfJoint.hpp` — czyli powstałaby dokładnie ta zależność od warstwy URDF, której §4.1 chciał uniknąć, tylko o poziom niżej. Review ma rację.

**Rekomendacja: wydzielić `src/ik_equations/model/Vector3.hpp`.** `UrdfJoint.hpp` i `FixedRigidTransform.hpp` włączają go.

Zasięg zmiany zweryfikowany: `UrdfJoint.hpp` jest włączany przez **dwa** pliki (`KinematicChain.hpp`, `RobotDescription.hpp`), oba tranzytywnie dostaną `Vector3` bez zmian. Ekstrakcja jest mechaniczna i bez zmiany semantyki.

---

## 5. Rekomendowane API

### 5.1 Fasada

```cpp
[[nodiscard]] std::expected<void, IkEquationBuilderError> setTcp(const FixedRigidTransform&);
              void clearTcp() noexcept;
[[nodiscard]] std::expected<void, IkEquationBuilderError> buildTcpForwardKinematics();

[[nodiscard]] const FixedRigidTransform* tcp() const noexcept;
[[nodiscard]] const SymbolicTransform*   tcpForwardKinematics() const noexcept;
```

Istniejące pięć metod **bez zmian**.

### 5.2 `setTcp` wymaga wybranego łańcucha — **blocker z review**

v1 twierdziła, że TCP jest związany z końcem łańcucha, a jednocześnie proponowała `setTcp` bez preconditionu. To była niespójność: **bez wybranego łańcucha nie istnieje frame, względem którego `T_chain_tip_tcp` cokolwiek znaczy.**

Kontrakt:

```
setTcp bez wybranego łańcucha  →  KinematicChainNotSelected, stan bez zmian
```

TCP wolno ustawić **po `selectChain`, przed lub po `buildForwardKinematics`** — obie kolejności są poprawne:

```
loadRobotModel → selectChain → setTcp → buildForwardKinematics → buildTcpForwardKinematics
loadRobotModel → selectChain → buildForwardKinematics → setTcp → buildTcpForwardKinematics
```

To wynika wprost z grafu z §7: `tcp` zależy od `kinematicChain`, **nie** od `forwardKinematics`.

### 5.3 Osobne `buildTcpForwardKinematics`

| Wariant | Ocena |
|---|---|
| `buildForwardKinematics()` automatycznie uwzględnia TCP | **odrzucony** |
| osobne `buildTcpForwardKinematics()` | **przyjęty** |

Wariant automatyczny sprawiłby, że `forwardKinematics()` znaczy raz „do końca łańcucha", raz „do TCP", **zależnie od niewidocznego stanu** — wołający trzymający wskaźnik nie ma jak stwierdzić, którą wielkość dostał. To ten sam wzorzec, który projekt odrzucał konsekwentnie. Dodatkowo `T_base_tip` jest potrzebne samo w sobie: to ono jest zweryfikowane numerycznie w Fazie 1.

### 5.4 Co jest publiczne

| Wielkość | Dostępna? |
|---|---|
| `T_base_chain_tip` | tak — istniejące `forwardKinematics()` |
| `T_base_tcp` | tak — `tcpForwardKinematics()` |
| `T_chain_tip_tcp` jako `SymbolicTransform` | **nie** — tania funkcja konfiguracji; składowanie dodałoby czwarte pole i czwarty wiersz unieważniania bez zysku |
| konfiguracja TCP | tak — `tcp()` |

### 5.5 Brak TCP ≠ TCP tożsamościowy

`tcp() == nullptr` znaczy **„nie ustawiono"** → `buildTcpForwardKinematics` zwraca `TcpNotSet`. Jawne `setTcp(FixedRigidTransform{})` jest **poprawnym TCP** równym tożsamości.

„Zapomniałem ustawić narzędzie" i „narzędzie jest w punkcie mocowania" to różne sytuacje; milcząca zamiana pierwszej w drugą dałaby wiarygodny, błędny TCP. Dzięki fast pathowi z §2.2 jawna tożsamość **nie kosztuje ani jednego węzła**.

---

## 6. Odpowiedzialności komponentów

### 6.1 Kierunek zależności — ustalenie mocniejsze niż zgłoszone

Review zwróciło uwagę, że umieszczenie `makeRigidTransform(const FixedRigidTransform&, ...)` w `SymbolicTransform.hpp` dałoby krawędź `symbolic → model`. **Sprawdziłem: warstwa symboliczna ma dziś zero zależności od `model/`** — `grep` po `model/` w `src/ik_equations/symbolic/` nie daje ani jednego trafienia. (Trafienia „Vector3" w tej warstwie to `SymbolicVector3`, inna nazwa.)

Byłaby to więc **pierwsza taka krawędź w projekcie**, a nie kolejna. Uwaga review jest ostrzejsza, niż została sformułowana, i przyjmuję ją w mocniejszej postaci: **warstwa symboliczna nie dowiaduje się o `model/` niczego.**

### 6.2 Rozkład na pliki

| Element | Gdzie | Uzasadnienie |
|---|---|---|
| `assembleTransform(SymbolicRotation, SymbolicVector3)` | **`symbolic/SymbolicTransform.hpp` — upublicznione** | operacja czysto symboliczna, bez fabryki i bez `model/`; **usuwa istniejącą duplikację** (dwie kopie → jedna) |
| `makeRpyRotation(const Vector3&, const ExpressionFactory&)` | `builders/RigidTransformConstruction.hpp` | bierze `Vector3` z `model/`, więc należy do `builders/`, które od `model/` już zależy |
| `buildFixedRigidTransform(const FixedRigidTransform&, const ExpressionFactory&)` | j.w. | wolna funkcja, nie klasa |
| `PrincipalAxis`, `makePrincipalRotation` | **`builders/detail/PrincipalRotation.hpp`** | detal współdzielony przez dwie jednostki translacji, nigdy w nagłówku, po który sięgnąłby konsument |

Kierunek: `model → builders → symbolic`. Krawędź `symbolic → model` **nie powstaje**.

### 6.3 Dlaczego nie klasa buildera i nie `JointTransformBuilder`

| Wariant | Ocena |
|---|---|
| `TcpTransformBuilder` / `ToolTransformBuilder` | **odrzucony** — klasa bez stanu poza fabryką, opakowująca jedno wywołanie |
| rozszerzenie `JointTransformBuilder` | **odrzucony** — wymagałoby sfabrykowania `KinematicJoint` z nazwą, indeksem i typem `Fixed`, czyli **udawania, że TCP jest jointem**; dane byłyby kłamstwem |
| wolna funkcja w `builders/` | **przyjęty** |

Granica: **`builders/` umie zbudować stałą transformację z konfiguracji; `symbolic/` umie ją złożyć i sprawdzić; fasada decyduje, czym ona jest i kiedy przestaje być aktualna.** Pojęcie „TCP" nie pojawia się ani w `symbolic/`, ani w `builders/` — tam jest wyłącznie „stała transformacja sztywna".

### 6.5 Skąd fasada bierze `ExpressionFactory` — blocker z review

`buildFixedRigidTransform` i `multiplyTransforms` wymagają fabryki, a **`IkEquationBuilder` nie ma dziś do żadnej dostępu**: trzyma `JointTransformBuilder` i `ForwardKinematicsBuilder`, których fabryki są prywatnymi polami. v1 i v2 tego nie zauważyły.

| Wariant | Ocena |
|---|---|
| nowe pole `ExpressionFactory expressionFactory_` w fasadzie | **odrzucony** |
| getter na fabrykę w istniejących builderach | **odrzucony** — poszerza zatwierdzone API o akcesor istniejący wyłącznie dla wygody kogoś innego |
| **lokalna fabryka w `buildTcpForwardKinematics`** | **przyjęty** |

```cpp
const ExpressionFactory factory;                     // lokalna, bezstanowa
const auto fixed = buildFixedRigidTransform(*tcp_, factory);
tcpForwardKinematics_ = multiplyTransforms(*forwardKinematics_, fixed, factory);
```

Uzasadnienie odrzucenia pola: `ExpressionFactory` **nie ma ani jednego pola**, więc dodatkowa składowa fasady niczego nie współdzieli — dokładnie ta obserwacja, przez którą `STATUS.md` niesie wpis „Factory ownership is value-semantic". Pole sugerowałoby współdzielenie, którego nie ma.

Gdyby fabryka kiedyś zyskała stan, **wszystkie trzy warianty** trzeba będzie przemyśleć naraz, razem z dwiema kopiami trzymanymi już przez buildery. Zapisanie tego teraz jest uczciwsze niż udawanie, że wybór pola dziś rozwiązuje coś na przyszłość.

### 6.4 Ekstrakcja konwersji RPY — wariant B, doprecyzowany

`makeRpyRotation` staje się jedyną produkcyjną implementacją konwencji RPY; `JointTransformBuilder.cpp` woła ją zamiast lokalnego `buildRpyRotation`.

Powód nie jest estetyczny. `STATUS.md` zapisuje jako znaną lukę, że **konwencja RPY opiera się na przekonaniu wspólnym dla obu implementacji, nie na źródle zewnętrznym**. Przy takim stanie dwie produkcyjne implementacje tej samej konwencji są jakościowo gorsze niż jedna: rozjazd byłby rozjazdem w konwencji, czyli dokładnie tam, gdzie nie mamy niezależnej weryfikacji.

Ryzyko ograniczone: `JointTransformBuilder` ma 22 testy, walidacja numeryczna 17, w tym dwa ręczne oracle i `NumericReferenceUsesCorrectRpyOrder`.

**Zmiana wobec v1:** `PrincipalAxis` i `makePrincipalRotation` **nie trafiają do publicznego API**. v1 twierdziła, że muszą, bo potrzebuje ich fast path osi w `JointTransformBuilder`. To nieprawda — wystarczy nagłówek `detail/`, widziany przez dwie jednostki translacji, które faktycznie ich potrzebują. Publiczne API nie ma powodu znać `bool negated`.

---

## 7. Graf zależności stanu i unieważnianie

### 7.1 Blocker z review: stan **nie jest** liniowym łańcuchem

v1 zapisała regułę „stan jest łańcuchem liniowym, każdy krok czyści wszystko poniżej". **Jej własna tabela temu przeczyła**: `setTcp` nie ruszało `forwardKinematics_`, i słusznie.

Tabela była poprawna. **Błędna była reguła, którą z niej wyprowadziłem** — i to jest gorszy rodzaj pomyłki, bo tabela dotyczy dzisiejszych pięciu pól, a reguła miała rządzić wszystkim, co dojdzie w Fazie 2.

Prawidłowy model to **graf zależności**:

```
        RobotDescription
               │
               ▼
         KinematicChain
          │          │
          ▼          ▼
  ForwardKinematics  TCP configuration
          │          │
          └────┬─────┘
               ▼
      TcpForwardKinematics
```

```
kinematicChain     → forwardKinematics
kinematicChain     → tcp
forwardKinematics  ┐
                   ├→ tcpForwardKinematics
tcp                ┘
```

`tcp` **nie zależy** od `forwardKinematics` — oba są dziećmi `kinematicChain`. Dlatego §5.2 dopuszcza obie kolejności.

To nie jest poprawka diagramu. `IkEquationSystem` z etapu F2.4 będzie zależeć od **`tcpForwardKinematics` + `target`** — znowu dwóch rodziców. Model liniowy nie umiałby tego wyrazić i pierwszy węzeł o dwóch rodzicach wymusiłby przeprojektowanie.

**Reguła ogólna, tym razem poprawna:** udany krok ustawia swój węzeł i unieważnia **wszystkich jego potomków w grafie** — nie „wszystko poniżej w liście".

### 7.2 Tabela unieważniania

| Operacja (sukces) | `robotDescription_` | `kinematicChain_` | `forwardKinematics_` | `tcp_` | `tcpForwardKinematics_` |
|---|---|---|---|---|---|
| `loadRobotModel` | **ustaw** | reset | reset | reset | reset |
| `selectChain` | — | **ustaw** | reset | **reset** | reset |
| `buildForwardKinematics` | — | — | **ustaw** | **zachowaj** | **reset** |
| `setTcp` | — | — | **zachowaj** | **ustaw** | **reset** |
| `clearTcp` | — | — | zachowaj | **reset** | **reset** |
| `buildTcpForwardKinematics` | — | — | — | — | **ustaw** |

Dwa wiersze warte komentarza:

**`buildForwardKinematics` zachowuje TCP, czyści `tcpForwardKinematics_`.** Nie dlatego, że wynik byłby matematycznie inny, tylko dlatego, że przebudowa FK tworzy **nowe węzły** — zachowany `T_base_tcp` wskazywałby na poprzednie drzewo, zostawiając obiekt, w którym dwa wyniki pochodzą z dwóch różnych przebiegów.

**`setTcp` zachowuje FK.** TCP nie jest potomkiem FK w grafie.

### 7.3 Czyją własnością jest TCP

Rozstrzygający scenariusz: użytkownik wybiera `base_link → tool0`, ustawia TCP `(0, 0, 0.1)`, po czym wybiera `base_link → flange`. Te same trzy liczby oznaczają teraz offset względem **innego fizycznego frame'u**, czyli **inny punkt w przestrzeni**. Zachowanie TCP dałoby wynik wiarygodny i błędny; wyczyszczenie daje głośne `TcpNotSet`.

**Zmiana łańcucha czyści TCP.** Zasada projektu jest tu jednoznaczna: głośny brak jest lepszy niż wiarygodna nieprawda.

Koszt przyjmuję świadomie: ponowny wybór **tego samego** łańcucha także wyczyści TCP. Alternatywą byłoby porównywanie nazw linków, czyli heurystyka udająca wiedzę.

### 7.4 Gwarancja transakcyjna

`setTcp` sprawdza precondition i waliduje **przed** zapisem — przy błędzie poprzedni TCP i poprzedni `T_base_tcp` zostają nietknięte. `buildTcpForwardKinematics` sprawdza preconditions przed jakimkolwiek budowaniem. `clearTcp` nie może zawieść (`void`, `noexcept`).

---

## 8. Model błędów

**Rozszerzamy istniejący enum** — jeden przepływ i jedna fasada; dwa kształty błędu zmuszałyby wołającego do obsługi dwóch rodzajów wyniku w jednej sekwencji.

```cpp
enum class IkEquationBuilderErrorCode
{
    RobotModelNotLoaded,
    KinematicChainNotSelected,
    UrdfLoadFailed,
    ChainBuildFailed,
    ForwardKinematicsNotBuilt,   // nowe
    TcpNotSet,                   // nowe
    InvalidTcpTransform          // nowe
};
```

| Sytuacja | Kod |
|---|---|
| `setTcp` bez wybranego łańcucha | `KinematicChainNotSelected` |
| `setTcp` z wartością nieskończoną w którejkolwiek z sześciu liczb | `InvalidTcpTransform` |
| `buildTcpForwardKinematics` bez łańcucha / FK / TCP | patrz §8.1 |

### 8.1 Deterministyczna kolejność brakujących prerekwizytów — **wymagana przez review**

Przy `loadRobotModel → selectChain → buildTcpForwardKinematics` brakuje jednocześnie FK i TCP. v1 nie ustalała, który błąd wygrywa.

**Kolejność sprawdzeń odpowiada kolejności prerekwizytów:**

```
1.  !kinematicChain_      →  KinematicChainNotSelected
2.  !forwardKinematics_   →  ForwardKinematicsNotBuilt
3.  !tcp_                 →  TcpNotSet
```

Daje to wołającemu **najbardziej użyteczną instrukcję naprawczą**: zgłasza pierwszy brakujący krok, a nie ostatni.

### 8.2 Struktura błędu bez zmian

`IkEquationBuilderError` zostaje. `message` nazywa składową, która zawiodła (np. `"tcp translation z is not finite"`). Nie dokładam typowanego pola analogicznego do `chainError` — nie ma dla niego istniejącego enuma i powstałoby dla jednego przypadku.

Inwariant bez zmian: `chainError` ma wartość wtedy i tylko wtedy, gdy `code == ChainBuildFailed`. Trzy nowe kody go nie ustawiają.

**Brak walidacji ortogonalności i normy** — konsekwencja wyboru RPY (§4.3). Kod w rodzaju `InvalidTcpRotation` byłby dziś martwy.

---

## 9. API i własność

- `[[nodiscard]]` na `setTcp`, `buildTcpForwardKinematics` i obu nowych akcesorach.
- `clearTcp()` — `void`, `noexcept`; nie może zawieść.
- `FixedRigidTransform` przechowywany **przez wartość** w `std::optional` (sześć `double`); nie współdzielony, nie przenoszony.
- Kopiowanie i przenoszenie fasady: **bez zmian**, domyślne.

### 9.1 Kontrakt unieważniania wskaźników — wynikający z grafu, nie zbiorczy

v2 powtarzała istniejące sformułowanie „unieważniane przez **każdą** udaną operację zmieniającą stan". Review słusznie zauważa, że to **przeczy proponowanym testom**: jeżeli każde wywołanie może unieważnić każdy wskaźnik, to `SettingTcpPreservesForwardKinematics` sprawdza coś, czego kontrakt nie obiecuje.

Graf z §7.1 pozwala zapisać to dokładnie. **Operacja, która nie modyfikuje danego węzła, nie zmienia znaczenia wskaźnika do niego.**

| Akcesor | Po tych operacjach wcześniej pobrany wskaźnik jest **nieaktualny** |
|---|---|
| `kinematicChain()` | `loadRobotModel`, `selectChain` |
| `forwardKinematics()` | `loadRobotModel`, `selectChain`, `buildForwardKinematics` |
| `tcp()` | `loadRobotModel`, `selectChain`, `setTcp`, `clearTcp` |
| `tcpForwardKinematics()` | wszystkie sześć operacji |

Wprost: `setTcp` **nie** dezaktualizuje `forwardKinematics()`, a `buildForwardKinematics` **nie** dezaktualizuje `tcp()`. Pinują to dwa testy z §10.2.

### 9.1.1 „Nieaktualny", nie „unieważniony" — i dlaczego to nie jest czepianie się słowa

Review zwraca uwagę, że „unieważniony" sugeruje wiszący wskaźnik, a tak być nie musi. To rozróżnienie jest **istotniejsze, niż wygląda**.

Przypisanie do **już aktywnego** `std::optional<SymbolicTransform>` konstruuje nową wartość w tym samym miejscu — adres obiektu **nie zmienia się**. Wcześniej pobrany wskaźnik pozostaje więc formalnie poprawny i **nie wywoła awarii**: będzie cicho pokazywał **nowy wynik**, podczas gdy wołający sądzi, że trzyma stary.

To jest gorsze niż wiszący wskaźnik. Wiszący wskaźnik daje szansę na crash pod sanitizerem; ten daje **wiarygodną, błędną odpowiedź** — dokładnie ta klasa błędu, przed którą projekt broni się wszędzie indziej (brak domyślnego zera w evaluatorze, `optional` w fasadzie, klucz `Expression` w cache'u).

Dlatego kontrakt jest **semantyczny, nie adresowy**:

> Po operacji wymienionej w tabeli wcześniej pobranego wskaźnika **nie wolno traktować jako dostępu do poprzedniego wyniku** — niezależnie od tego, czy adres pozostał ten sam.

Dla węzłów nietkniętych obietnica jest odwrotna i **mocniejsza**: wskaźnik nadal wskazuje ten sam, niezmieniony wynik. To da się sprawdzić porównaniem tożsamości wskaźników i właśnie tak jest testowane.

**Zmiana obejmuje też istniejący komentarz w `IkEquationBuilder.hpp`** — dziś jest bezpieczny, ale nadmiernie konserwatywny; po tej zmianie ma mówić to samo, co tabela. To jedyna modyfikacja dokumentacji już zatwierdzonego API.

Wszystkie wskaźniki pozostają nieposiadające i giną wraz z fasadą.

### 9.2 `clearTcp` jest idempotentne

```
clearTcp() wolno wywołać przed wyborem łańcucha, bez ustawionego TCP i wielokrotnie.
Po każdym wywołaniu:  tcp() == nullptr  oraz  tcpForwardKinematics() == nullptr.
```

Zapisuję to jawnie, żeby implementacja nie uzależniła tej metody od stanu łańcucha — `void`/`noexcept` samo w sobie tego nie gwarantuje. Nie wymaga osobnego testu: `ClearingTcpInvalidatesTcpForwardKinematics` obejmie podwójne wywołanie.

---

## 10. Plan testów — 21 pozycji

### 10.1 Poprawność złożenia (6)

| Test | Co pinuje |
|---|---|
| `IdentityTcpLeavesForwardKinematicsUnchanged` | jawny TCP zerowy → **`sameNode`** na 16 komórkach — §10.6 |
| `AppliesTranslationOnlyTcp` | sama translacja |
| `AppliesRotationOnlyTcp` | sama rotacja |
| `AppliesCombinedTcp` | oba naraz |
| **`AppliesTcpTranslationInToolFrame`** | §10.4 — najważniejszy test etapu |
| `PreservesCanonicalHomogeneousLastRow` | `[0 0 0 1]` przez `isZero`/`isOne` po złożeniu |

### 10.2 Graf stanu (6)

| Test | Co pinuje |
|---|---|
| `ChangingTcpInvalidatesTcpForwardKinematics` | `setTcp` czyści wynik |
| `ClearingTcpInvalidatesTcpForwardKinematics` | **nowy** — `clearTcp` był w tabeli, nic go nie sprawdzało |
| `RebuildingForwardKinematicsInvalidatesTcpForwardKinematicsButPreservesTcp` | **nowy** — asercje `tcp() != nullptr` **i** `tcpForwardKinematics() == nullptr`; pinuje, że TCP nie jest potomkiem FK |
| `ChangingChainInvalidatesTcpAndTcpForwardKinematics` | §7.3 |
| `LoadingNewRobotInvalidatesTcpAndTcpForwardKinematics` | j.w. tranzytywnie |
| `SettingTcpPreservesForwardKinematics` | druga strona grafu — `setTcp` nie rusza FK |

### 10.3 Błędy (7)

| Test | Kod |
|---|---|
| `RejectsTcpBeforeChainSelection` | `KinematicChainNotSelected` — dotyczy **`setTcp`** (§5.2) |
| `RejectsTcpForwardKinematicsBeforeChainSelection` | **nowy** — `KinematicChainNotSelected` z **`buildTcpForwardKinematics`**; §10.7 |
| `RejectsTcpForwardKinematicsBeforeForwardKinematics` | `ForwardKinematicsNotBuilt` |
| `RejectsTcpForwardKinematicsWithoutTcp` | `TcpNotSet` |
| `RejectsNonFiniteTcpTranslation` | `InvalidTcpTransform` |
| `RejectsNonFiniteTcpRotation` | `InvalidTcpTransform` |
| `FailedTcpUpdatePreservesPreviousState` | porównanie **tożsamości wskaźników** przed i po nieudanym `setTcp` |

`FailedTcpUpdatePreservesPreviousState` zostaje osobnym testem, nie asercją doklejoną do testu non-finite: **walidacja i gwarancja transakcyjna to różne kontrakty**, a łączenie ich znaczyłoby, że usunięcie jednego cicho przestaje pilnować drugiego.

### 10.4 Test kolejności składania — ręczny oracle

KR640, `q1 = π/2`, reszta zero. Z Fazy 1 (policzone ręcznie, przetestowane):

```
R_base_tip = Rz(π/2)          p_base_tip = (0, 1.600, 2.335)
```

TCP = translacja `(0.1, 0, 0)`, bez rotacji:

```
p_base_tcp = (0, 1.600, 2.335) + Rz(π/2)·(0.1, 0, 0)
           = (0, 1.600, 2.335) + (0, 0.1, 0)
           = (0, 1.700, 2.335)
```

Implementacja dodająca TCP w bazie dałaby `(0.1, 1.600, 2.335)` — **różnica w innej osi**, nie do przejścia przypadkiem. Oracle nie zależy od żadnej z dwóch implementacji FK.

### 10.5 Roboty i referencja (2)

`BuildsKr4TcpForwardKinematics`, `BuildsKr640TcpForwardKinematics` — pełna ścieżka przez fasadę z niezerowym TCP, porównana z referencją kwaternionową. Pokrywają jednocześnie `MatchesQuaternionReferenceForKr4/Kr640`; rozdzielanie dałoby dwa testy o tym samym przebiegu.

### 10.6 Tożsamościowy TCP — `sameNode`, nie `structurallyEqual`

v2 proponowała `structurallyEqual`. **Za mało.** Deklaracja brzmi „nie doda ani jednego węzła", a drzewo przebudowane od zera w identycznym kształcie przeszłoby `structurallyEqual` bez zastrzeżeń. Fast path zwraca dosłownie `return lhs;`, więc właściwym kontraktem jest **tożsamość węzła**:

```cpp
EXPECT_TRUE(sameNode((*builder.forwardKinematics())(row, column),
                     (*builder.tcpForwardKinematics())(row, column)));
```

`sameNode` jest publiczne w `Expression.hpp` i jest O(1). To jedyna asercja, która faktycznie pinuje „zero nowych węzłów"; `structurallyEqual` można zostawić obok jako słabszą, ale sama nie wystarcza.

### 10.7 Dlaczego priorytet łańcucha wymaga własnego testu

§8.1 ustala kolejność `chain → FK → TCP`. Bez `RejectsTcpForwardKinematicsBeforeChainSelection` implementacja mogłaby sprawdzać **najpierw FK** i na świeżej fasadzie zwracać `ForwardKinematicsNotBuilt` — a **wszystkie pozostałe testy nadal by przechodziły**. Ustalona kolejność bez testu jest deklaracją, nie kontraktem.

`RejectsTcpBeforeChainSelection` tego nie pokrywa: dotyczy `setTcp`, czyli innej metody.

**Razem: 21 nowych. Oczekiwany stan: 221 + 21 = 242.**

*(6 poprawności złożenia + 6 grafu stanu + 7 błędów + 2 roboty. W v2 podałem 19 — zwykły błąd rachunkowy, faktyczna suma tamtej listy wynosiła 20.)*

---

## 11. Plan walidacji numerycznej

**Rozszerzamy istniejącą referencję, nie tworzymy trzeciego modelu FK:**

```
reference = compose( numericForwardKinematics(chain, configuration),
                     RigidTransform{ fromRollPitchYaw(tcp.rpy), toVector3d(tcp.translation) } )
```

`compose` implementuje `p = p_a + rotate(q_a, p_b)`, czyli **dokładnie semantykę frame'u narzędzia** — więc referencja weryfikuje kolejność składania niezależnie, kwaternionami zamiast macierzy.

Zakres: **te same dziewięć konfiguracji co w Fazie 1** — zerowa, sześć jednojointowych, mieszana, blisko limitów — dla obu robotów, z TCP mającym **i translację, i rotację** (czysto translacyjny nie sprawdziłby złożenia rotacji). Daje to **18 porównań macierzy**, tyle samo co walidacja FK, realizowane pętlą wewnątrz dwóch testów GTest.

Tolerancja: **`1e-12` bez zmian.** TCP dokłada jedno złożenie do siedmiu już wykonywanych; wzrost błędu rzędu pojedynczych ULP wobec zmierzonych `5.55e-16`. Reguła ta sama: przekroczenie to ustalenie do review, nie podniesienie progu.

Zastrzeżenie: TCP **nie zmienia zakresu dowodu z Fazy 1**. Walidacja nadal zaczyna się od `KinematicChain`, a konwencja RPY — teraz używana także dla TCP — nadal opiera się na przekonaniu wspólnym dla obu implementacji. Ekstrakcja z §6.4 tego nie naprawia; sprawia jedynie, że przekonanie jest zapisane **w jednym miejscu**.

---

## 12. Plan zmian w plikach

**Dodane:**

| Plik | Zawartość |
|---|---|
| `src/ik_equations/model/Vector3.hpp` | wydzielony agregat (§4.4) |
| `src/ik_equations/model/FixedRigidTransform.hpp` | `{translation, rpy}` + kontrakt `T_parent_child`, metry, radiany |
| `src/ik_equations/builders/RigidTransformConstruction.hpp/.cpp` | `makeRpyRotation`, `buildFixedRigidTransform` |
| `src/ik_equations/builders/detail/PrincipalRotation.hpp` | `PrincipalAxis`, `makePrincipalRotation` — detal; funkcje **`inline`** albo osobny `.cpp`, bo nagłówek włączają dwie jednostki translacji (uwaga z review — do rozstrzygnięcia w proposalu implementacyjnym) |
| `tests/test_tcp_transform.cpp` | 21 testów |

**Zmienione:**

| Plik | Zmiana |
|---|---|
| `src/ik_equations/model/UrdfJoint.hpp` | usunięcie `Vector3`, include `Vector3.hpp` |
| `src/ik_equations/symbolic/SymbolicTransform.hpp/.cpp` | upublicznienie `assembleTransform`, usunięcie lokalnej kopii |
| `src/ik_equations/builders/JointTransformBuilder.cpp` | użycie wspólnych `makeRpyRotation`, `makePrincipalRotation`, `assembleTransform` |
| `src/ik_equations/IkEquationBuilder.hpp/.cpp` | trzy kody błędu, dwa pola stanu, cztery metody, **doprecyzowany komentarz o unieważnianiu wskaźników** (§9.1) |
| `tests/support/NumericForwardKinematics.hpp/.cpp` | `numericTcpForwardKinematics` |
| `CMakeLists.txt` | **dwie nowe linie** — `builders/RigidTransformConstruction.cpp`, `builders/detail/PrincipalRotation.cpp`; główny plik wymienia źródła jawnie, więc bez tego nie zlinkują się |
| `tests/CMakeLists.txt` | jedna linia — `test_tcp_transform.cpp` |
| `STATUS.md` | nowy komponent, **graf zależności z §7.1** |
| `README.md` | wzmianka w sekcji komponentów; przykład bez zmian, bo TCP jest opcjonalne |

**Bez zmian — jawnie:** `UrdfModelLoader`, `KinematicChainBuilder`, `ForwardKinematicsBuilder`, `Expression`, `ExpressionFactory`, `SymbolicMatrix`, `ExpressionEvaluator`, `main.cpp`.

---

## 13. Non-goals

Jawnie **nie** projektujemy: `ConstraintBuilder`, `PositionTarget`, `PoseTarget`, `IkEquationSystem`, `EquationSimplifier`, `EquationSolver`, `IkPatternDetector`, `CodeGenerator`, YAML, konfiguracji MotionBridge, kalibracji TCP, dynamicznej zmiany TCP w trakcie ewaluacji, pozycji wymuszonej, kierunku dyszy, wielu jednocześnie zdefiniowanych TCP.

Jedyny ukłon w stronę przyszłości to **graf zależności z §7.1** — nie projektuje żadnego z powyższych, tylko zapewnia, że węzeł o dwóch rodzicach (`tcpFK + target → equationSystem`) nie wymusi przeprojektowania fasady.

---

## 14. Ryzyka

| Ryzyko | Ocena | Reakcja |
|---|---|---|
| ekstrakcja RPY psuje `JointTransformBuilder` | **średnie** | 22 + 17 testów, w tym dwa ręczne oracle i test kolejności RPY |
| upublicznienie `assembleTransform` zachęci do budowania niekanonicznych transformacji | niskie | funkcja **wymusza** kanoniczny ostatni wiersz — startuje od `identity()`; jej upublicznienie zmniejsza ryzyko, bo dziś każdy pisze własną |
| ekstrakcja `Vector3` psuje include'y | niskie | `UrdfJoint.hpp` włączany przez dwa pliki, oba tranzytywnie |
| czyszczenie TCP przy ponownym wyborze tego samego łańcucha irytuje | niskie, świadome | alternatywa to heurystyka po nazwach linków |
| `FixedRigidTransform` używany w złym kierunku (`T_tcp_tip`) | **średnie** | kontrakt z §4.2 w komentarzu typu; test `AppliesTcpTranslationInToolFrame` wykryje odwrócenie |
| przyszłe wejście kwaternionowe | niskie | konwersja na granicy, nie drugi kształt stanu |

---

## 15. Otwarte decyzje wymagające review

1. **§6.4 — ekstrakcja konwencji RPY (wariant B).** Rekomendacja: **tak**, z `PrincipalAxis` w `detail/`, nie w publicznym API. Jedyna zmiana dotykająca kodu zatwierdzonego wcześniej.
2. **§6.2 — upublicznienie `assembleTransform`.** Rekomendacja: **tak**; usuwa istniejącą duplikację i nie wprowadza żadnej zależności.
3. ~~**§12 — `detail/PrincipalRotation.hpp` jako `inline` czy z osobnym `.cpp`.**~~ **Rozstrzygnięte w review: osobny `.cpp`.** Nagłówek niesie wyłącznie `enum class PrincipalAxis` i deklarację `makePrincipalRotation`; definicja w `PrincipalRotation.cpp`. Spójne z resztą projektu, gdzie nagłówki nie zawierają definicji poza szablonami, i eliminuje ryzyko ODR przy dwóch jednostkach translacji.

**Wszystkie trzy decyzje z §15 zostały rozstrzygnięte w review** — dokument nie zostawia otwartych kwestii blokujących proposal implementacyjny.

Rozstrzygnięte i **niewymagające** decyzji: `FixedRigidTransform` z kontraktem `T_parent_child`/metry/radiany, translacja + RPY, ekstrakcja `Vector3`, wolna funkcja w `builders/` zamiast klasy, `symbolic` bez wiedzy o `model/`, **lokalna bezstanowa fabryka zamiast pola fasady (§6.5)**, `setTcp` wymaga łańcucha, osobne `buildTcpForwardKinematics`, brak akcesora dla `T_chain_tip_tcp`, brak TCP ≠ TCP tożsamościowy, graf zależności zamiast łańcucha, **kontrakt wskaźników per akcesor (§9.1)**, **idempotencja `clearTcp` (§9.2)**, kolejność raportowania prerekwizytów, `sameNode` dla tożsamościowego TCP, `FailedTcpUpdatePreservesPreviousState` jako osobny test, trzy nowe kody błędu, rozszerzenie referencji kwaternionowej, tolerancja `1e-12`.

---

## 16. Co zmieniła rewizja v3

| # review | Zarzut | Werdykt | Gdzie |
|---|---|---|---|
| 1 | nie rozstrzygnięto, skąd fasada bierze `ExpressionFactory` | **przyjęty — blocker, luka v1 i v2** | §6.5 |
| 2 | zbiorczy kontrakt wskaźników przeczy grafowi i własnym testom | **przyjęty** | §9.1 |
| 3 | brak testu priorytetu łańcucha w `buildTcpForwardKinematics` | **przyjęty** | §10.3, §10.7 |
| 4 | liczba testów błędna (19 zamiast 20) | **przyjęty — błąd rachunkowy** | §10 |
| 5 | tożsamościowy TCP wymaga `sameNode`, nie `structurallyEqual` | **przyjęty** | §10.6 |
| 6 | idempotencja `clearTcp` niezapisana | **przyjęty** | §9.2 |

Blocker §1 jest wart odnotowania: przez dwie wersje projektowałem wywołania `buildFixedRigidTransform(..., factory)` i `multiplyTransforms(..., factory)`, **nie sprawdziwszy, czy fasada ma skąd wziąć fabrykę**. Nie ma — buildery trzymają swoje prywatnie. Pisałem sygnatury, nie sprawdzając, czy wołający jest w stanie je spełnić.

Zarzuty 2 i 5 mają wspólny kształt z błędami z v1: **kontrakt był słabszy niż to, co testy miały sprawdzać.** Przy wskaźnikach obiecywałem mniej, niż testowałem (więc test wykraczał poza kontrakt); przy tożsamościowym TCP deklarowałem więcej, niż test sprawdzał („zero nowych węzłów" pilnowane przez `structurallyEqual`, które przepuszcza przebudowę). W obu przypadkach zdanie i asercja mówiły co innego.

---

## 17. Co zmieniła rewizja v2

| # review | Zarzut | Werdykt | Gdzie |
|---|---|---|---|
| 1 | stan nie jest liniowym łańcuchem | **przyjęty — blocker, błąd v1** | §7.1 |
| 2 | `setTcp` bez preconditionu łańcucha | **przyjęty — blocker, niespójność v1** | §5.2 |
| 3 | `Vector3` wiąże model TCP z `UrdfJoint.hpp` | **przyjęty** | §4.4 |
| 4 | brak semantyki kierunku i jednostek | **przyjęty** | §4.2 |
| 5 | nie eksportować `PrincipalAxis` publicznie | **przyjęty** | §6.2, §6.4 |
| 6 | kierunek zależności `symbolic → model` | **przyjęty w mocniejszej postaci** — §6.1 | §6.1, §6.2 |
| 7 | brak testów `clearTcp` i przebudowy FK | **przyjęty** | §10.2 |
| 8 | niedeterministyczna kolejność błędów | **przyjęty** | §8.1 |

Dwa zarzuty to realne błędy v1, i warto nazwać, co je łączy. Zarzut 1: **tabela była poprawna, a reguła, którą z niej wyprowadziłem, jej przeczyła.** Zarzut 2: **tekst deklarował powiązanie TCP z końcem łańcucha, a sygnatura go nie wymuszała.** W obu przypadkach szczegół był dobry, a uogólnienie nad nim — nie. To ten sam wzorzec co przy `x·0` w evaluatorze i przy oracle'u `q = 0`: opis szedł dalej niż to, co faktycznie zostało ustalone.

Zarzut 6 sprawdziłem, zamiast przyjąć na słowo, i okazał się mocniejszy: `symbolic` nie ma **żadnej** zależności od `model/`, więc chodziło o pierwszą taką krawędź, nie o kolejną. Stąd rozwiązanie ostrzejsze niż zaproponowane — konwersja w `builders/`, a nie w `symbolic/`.

---

```
APPROVE-READY
```
