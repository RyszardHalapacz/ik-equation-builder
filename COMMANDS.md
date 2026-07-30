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
