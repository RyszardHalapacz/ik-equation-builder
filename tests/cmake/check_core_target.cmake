# The solver core must be consumable on its own (MotionBridge report 2026-11-21, 5.1).
#
# Run as a CTest case (see tests/CMakeLists.txt):
#   cmake -DKINEMAFORGE_SOURCE_DIR=... -DWORK_DIR=... -DGENERATOR=... -DMAKE_PROGRAM=...
#         -DCXX_COMPILER=... -DPUGIXML_SOURCE_DIR=... -DGOOGLETEST_SOURCE_DIR=...
#         -P check_core_target.cmake
#
# kinemaforge_ik_core is the analytic solver and nothing else. It must
#   1. exist, with the alias kinemaforge::ik_core;
#   2. not compile the legacy translation units (Kinematics.cpp, robot_model*,
#      UrdfModelLoader.cpp) nor the symbolic layer / code generator;
#   3. not put a top-level common/ or kinematics/ directory on its consumers'
#      include path: a host project with its own common/diagnostic.hpp or
#      kinematics/robot_model.hpp would otherwise get the wrong one, or an ODR
#      violation, depending on include order;
#   4. link, and solve, in a consumer that defines its OWN mt::kinematics
#      types, without pugixml.

foreach(var KINEMAFORGE_SOURCE_DIR WORK_DIR GENERATOR MAKE_PROGRAM CXX_COMPILER
            PUGIXML_SOURCE_DIR GOOGLETEST_SOURCE_DIR)
    if(NOT DEFINED ${var})
        message(FATAL_ERROR "${var} is not set")
    endif()
endforeach()

set(consumer_src "${WORK_DIR}/consumer")
file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${consumer_src}/host/common" "${consumer_src}/host/kinematics")

# The host's own headers, under the same relative paths KinemaForge's legacy
# code used to publish.
file(WRITE "${consumer_src}/host/common/diagnostic.hpp"
    "#pragma once\nnamespace mt { struct HostDiagnostic { int host_marker; }; }\n")
file(WRITE "${consumer_src}/host/kinematics/robot_model.hpp"
    "#pragma once\nnamespace mt::kinematics { struct RobotModel { double host_only; }; }\n")

set(consumer_cmake [=[
cmake_minimum_required(VERSION 3.21)
project(KinemaForgeCoreConsumer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)
set(BUILD_TESTING OFF)
add_subdirectory("@KINEMAFORGE_SOURCE_DIR@" kinemaforge)

if(NOT TARGET kinemaforge_ik_core)
    message(FATAL_ERROR "target kinemaforge_ik_core does not exist")
endif()
if(NOT TARGET kinemaforge::ik_core)
    message(FATAL_ERROR "alias kinemaforge::ik_core does not exist")
endif()

get_target_property(core_sources kinemaforge_ik_core SOURCES)
foreach(source IN LISTS core_sources)
    if(source MATCHES "(Kinematics\.cpp|robot_model|UrdfModelLoader|/symbolic/|/codegen/|IkEquationBuilder|ConstraintBuilder)")
        message(FATAL_ERROR "kinemaforge_ik_core must not compile ${source}")
    endif()
endforeach()
list(LENGTH core_sources core_source_count)
if(core_source_count LESS 5)
    message(FATAL_ERROR "kinemaforge_ik_core has suspiciously few sources: ${core_sources}")
endif()

get_target_property(core_includes kinemaforge_ik_core INTERFACE_INCLUDE_DIRECTORIES)
foreach(dir IN LISTS core_includes)
    string(REPLACE "$<BUILD_INTERFACE:" "" dir "${dir}")
    string(REPLACE ">" "" dir "${dir}")
    foreach(forbidden common kinematics)
        if(IS_DIRECTORY "${dir}/${forbidden}")
            message(FATAL_ERROR "kinemaforge_ik_core exposes ${dir}/${forbidden} to its consumers")
        endif()
    endforeach()
endforeach()

get_target_property(core_links kinemaforge_ik_core INTERFACE_LINK_LIBRARIES)
if(core_links MATCHES "pugixml")
    message(FATAL_ERROR "kinemaforge_ik_core must not depend on pugixml: ${core_links}")
endif()

add_executable(consumer main.cpp)
target_include_directories(consumer PRIVATE host)
target_link_libraries(consumer PRIVATE kinemaforge::ik_core)
]=])
string(REPLACE "@KINEMAFORGE_SOURCE_DIR@" "${KINEMAFORGE_SOURCE_DIR}" consumer_cmake "${consumer_cmake}")
file(WRITE "${consumer_src}/CMakeLists.txt" "${consumer_cmake}")

file(WRITE "${consumer_src}/main.cpp" "
#include \"common/diagnostic.hpp\"
#include \"kinematics/robot_model.hpp\"
#include \"ik_equations/builders/KinematicChainBuilder.hpp\"
#include \"ik_equations/solver/AnalyticIkSolver.hpp\"

// The host's definitions are the ones this translation unit sees.
static_assert(sizeof(mt::HostDiagnostic) == sizeof(int));
static_assert(sizeof(mt::kinematics::RobotModel) == sizeof(double));

int main()
{
    // An empty chain is rejected by the solver itself; reaching that answer
    // links the solver and the geometry/pattern code it uses.
    const kinemaforge::ik::KinematicChain chain;
    const kinemaforge::ik::PoseTarget target{{0.0, 0.0, 0.0}, {{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}}};
    const auto result = kinemaforge::ik::AnalyticIkSolver{}.solve(chain, target);
    return !result && result.error().code == kinemaforge::ik::AnalyticIkSolverErrorCode::UnsupportedKinematicStructure
               ? 0 : 1;
}
")

set(build_dir "${WORK_DIR}/build")
execute_process(
    COMMAND "${CMAKE_COMMAND}" -S "${consumer_src}" -B "${build_dir}"
            -G "${GENERATOR}"
            "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}"
            "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
            "-DFETCHCONTENT_SOURCE_DIR_PUGIXML=${PUGIXML_SOURCE_DIR}"
            "-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=${GOOGLETEST_SOURCE_DIR}"
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE output)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "Core consumer configure failed:\n${output}")
endif()

execute_process(
    COMMAND "${CMAKE_COMMAND}" --build "${build_dir}" --target consumer
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE output)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "Core consumer build failed:\n${output}")
endif()

# Run it. The exe path differs by generator; Ninja puts it in the build root.
execute_process(COMMAND "${build_dir}/consumer${CMAKE_EXECUTABLE_SUFFIX}"
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE output)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "Core consumer ran but reported ${result}:\n${output}")
endif()
