cmake_minimum_required(VERSION 3.21)

project(KinemaForge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (MINGW)
    # Statically link the MinGW runtime so binaries don't depend on
    # libstdc++-6.dll / libgcc_s_seh-1.dll / libwinpthread-1.dll being
    # found on PATH (other MinGW installs, e.g. Anaconda, can shadow
    # these with incompatible versions and crash with 0xc0000139).
    add_link_options(-static-libgcc -static-libstdc++ -static)
endif()

# --- Warning policy -----------------------------------------------------
#
# Carried by an INTERFACE target rather than add_compile_options, so it reaches
# exactly the targets that opt in. Directory-scoped options would also land on
# FetchContent dependencies -- googletest is made available inside
# tests/CMakeLists.txt, which add_subdirectory(tests) pulls into this
# directory's option scope. Their warnings are not ours to fix, and a target
# whitelist makes that a property of the design rather than of line order.
#
# -Werror is opt-in through KINEMAFORGE_WERROR, which CI sets. Unconditionally
# would tie the ability to build this project to the compiler version installed
# on a developer machine: a newer GCC adding one new warning would stop all work
# until someone cleans up. In CI that tradeoff is correct, locally it is not.
option(KINEMAFORGE_WERROR "Treat compiler warnings as errors (used by CI)" OFF)

add_library(kinemaforge_warnings INTERFACE)

if (MSVC)
    # /w14062 is REQUIRED, not decoration. C4062 ("enumerator not handled in
    # switch, and there is no default label") is a level-4 warning that
    # Microsoft ships OFF BY DEFAULT, so /W4 alone does not enable it -- and
    # C4062 is precisely the diagnostic the switch-based protections in this
    # codebase rely on. Without it an MSVC build would silently accept a new
    # enumerator with no case, which is the exact regression those protections
    # exist to prevent. /w1<nnnn> promotes the warning to level 1 so it is
    # reported at any /W level.
    #
    # C4061 is the same diagnostic for a switch that DOES have a default and
    # still misses enumerators, also off by default. This codebase's house
    # style is "no default", so C4061 fires only where that style is already
    # broken -- which makes it worth having.
    #
    # NOTE: this branch is unverified. The project builds with MinGW g++; no
    # MSVC toolchain was available when it was written.
    target_compile_options(kinemaforge_warnings INTERFACE
        /W4
        /w14062
        /w14061
    )
    if (KINEMAFORGE_WERROR)
        target_compile_options(kinemaforge_warnings INTERFACE /WX)
    endif()
else()
    # -Wswitch is listed explicitly even though -Wall already implies it. This
    # particular diagnostic is part of an architectural contract stated in
    # source comments (IkEquationSystem.cpp, UrdfModelLoader.cpp), so it is
    # named here rather than left to arrive as a side effect of a bundle whose
    # contents are a compiler-version detail.
    target_compile_options(kinemaforge_warnings INTERFACE
        -Wall
        -Wextra
        -Wswitch
    )
    if (KINEMAFORGE_WERROR)
        target_compile_options(kinemaforge_warnings INTERFACE -Werror)
    endif()
endif()

include(FetchContent)

# --- pugixml (URDF parsing) ---
FetchContent_Declare(
    pugixml
    GIT_REPOSITORY https://github.com/zeux/pugixml.git
    GIT_TAG        v1.14
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(pugixml)

add_library(kinemaforge_ik STATIC
    src/Kinematics.cpp
    src/kinematics/robot_model.cpp
    src/kinematics/robot_model_loader.cpp
    src/ik_equations/IkEquationBuilder.cpp
    src/ik_equations/UrdfModelLoader.cpp
    src/ik_equations/builders/KinematicChainBuilder.cpp
    src/ik_equations/builders/JointTransformBuilder.cpp
    src/ik_equations/builders/ForwardKinematicsBuilder.cpp
    src/ik_equations/builders/ConstraintBuilder.cpp
    src/ik_equations/builders/RigidTransformConstruction.cpp
    src/ik_equations/builders/detail/PrincipalRotation.cpp
    src/ik_equations/model/IkEquationSystem.cpp
    src/ik_equations/model/TargetValidation.cpp
    src/ik_equations/model/QuaternionTarget.cpp
    src/ik_equations/symbolic/Expression.cpp
    src/ik_equations/symbolic/ExpressionFactory.cpp
    src/ik_equations/symbolic/ExpressionEvaluator.cpp
    src/ik_equations/symbolic/RewriteEngine.cpp
    src/ik_equations/symbolic/SymbolicTransform.cpp
)
target_include_directories(kinemaforge_ik PUBLIC src)
target_link_libraries(kinemaforge_ik PUBLIC pugixml::pugixml)
# PRIVATE: our warning policy is not part of this library's contract. A consumer
# such as MotionBridge brings its own.
target_link_libraries(kinemaforge_ik PRIVATE kinemaforge_warnings)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE kinemaforge_ik kinemaforge_warnings)

enable_testing()
add_subdirectory(tests)
