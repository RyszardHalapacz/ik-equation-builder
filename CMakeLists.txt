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
    src/ik_equations/symbolic/Expression.cpp
    src/ik_equations/symbolic/ExpressionFactory.cpp
    src/ik_equations/symbolic/ExpressionEvaluator.cpp
    src/ik_equations/symbolic/SymbolicTransform.cpp
)
target_include_directories(kinemaforge_ik PUBLIC src)
target_link_libraries(kinemaforge_ik PUBLIC pugixml::pugixml)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE kinemaforge_ik)

enable_testing()
add_subdirectory(tests)
