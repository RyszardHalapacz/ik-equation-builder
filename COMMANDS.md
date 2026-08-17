# Commands — KinemaForge / IkEquationBuilder

Day-to-day cheat sheet. Everything is run from the project root folder in PowerShell.

## Building the project

First-time configure (only once, or after changes to CMakeLists.txt):
```powershell
cmake -B build -G Ninja
```

Build after code changes:
```powershell
cmake --build build
```

## Running tests

```powershell
ctest --test-dir build --output-on-failure
```

## Verifying the warning policy (`-Werror`)

`-Wall -Wextra -Wswitch` are always on for KinemaForge targets; `-Werror` is
opt-in via `KINEMAFORGE_WERROR`, which CI sets. Verify it in a **separate build
directory** — reconfiguring `build/` would cache `KINEMAFORGE_WERROR=ON` and turn
every later local build into a `-Werror` build:

```powershell
cmake -B build-werror -G Ninja -DKINEMAFORGE_WERROR=ON `
      -DCMAKE_CXX_COMPILER="C:/Program Files/JetBrains/CLion 2026.1/bin/mingw/bin/g++.exe" `
      -DCMAKE_MAKE_PROGRAM="C:/PROGRA~1/JETBRA~1/CLION2~1.1/bin/ninja/win/x64/ninja.exe"
cmake --build build-werror
ctest --test-dir build-werror --output-on-failure
```

The two compiler/generator paths are required because **`ninja` is not on PATH**
— it ships with CLion, and the existing `build/` only works because its cache
already holds the path. Without them CMake fails with
`CMAKE_CXX_COMPILER not set, after EnableLanguage`.

Note the two configurations differ in more than warnings: `build/` is
`Release` (`-DNDEBUG`, assertions compiled out), `build-werror/` has an empty
build type (assertions live).

## PowerShell shortcuts (profile)

The following functions are defined in `$PROFILE` (edit: `notepad $PROFILE`, reload: `. $PROFILE`):

```powershell
function iebbuild {
    cmake --build "C:\Users\Komputer\CLionProjects\builder_rownan\build"
}

function iebtest {
    ctest --test-dir "C:\Users\Komputer\CLionProjects\builder_rownan\build" --output-on-failure
}
```

Usage from any directory in any PowerShell session:
- `iebbuild` — builds the project
- `iebtest` — runs the tests

## Note: MinGW / PATH

The project is statically linked (`-static-libgcc -static-libstdc++ -static` in `CMakeLists.txt` for MinGW) to avoid conflicts with other MinGW installs on PATH (e.g. Anaconda), which could cause a `0xc0000139` crash when running tests.
