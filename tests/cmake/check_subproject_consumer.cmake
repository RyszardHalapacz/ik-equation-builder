# Verifies that KinemaForge can be consumed through add_subdirectory().
#
# Run as a CTest case (see tests/CMakeLists.txt):
#   cmake -DKINEMAFORGE_SOURCE_DIR=... -DWORK_DIR=... -DGENERATOR=... -DMAKE_PROGRAM=...
#         -DCXX_COMPILER=... -DPUGIXML_SOURCE_DIR=... -DGOOGLETEST_SOURCE_DIR=...
#         -P check_subproject_consumer.cmake
#
# Two scenarios:
#   1. BUILD_TESTING=OFF: the library must not add its tests (no googletest,
#      no fixture generator target).
#   2. KINEMAFORGE_BUILD_TESTS=ON inside a parent project: the test data paths
#      must resolve relative to KinemaForge, not to the parent. A Ninja dry run
#      (-n) fails immediately on a dependency file that does not exist, so it
#      catches a wrong data path without compiling anything.
#
# Dependencies are taken from local source trees (FETCHCONTENT_SOURCE_DIR_*),
# so the check needs no network access.

foreach(var KINEMAFORGE_SOURCE_DIR WORK_DIR GENERATOR MAKE_PROGRAM CXX_COMPILER
            PUGIXML_SOURCE_DIR GOOGLETEST_SOURCE_DIR)
    if(NOT DEFINED ${var})
        message(FATAL_ERROR "${var} is not set")
    endif()
endforeach()

set(consumer_src "${WORK_DIR}/consumer")
file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${consumer_src}")

file(WRITE "${consumer_src}/CMakeLists.txt" "
cmake_minimum_required(VERSION 3.21)
project(KinemaForgeConsumer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)
add_subdirectory(\"${KINEMAFORGE_SOURCE_DIR}\" kinemaforge)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE kinemaforge_ik)
")
file(WRITE "${consumer_src}/main.cpp" "int main() { return 0; }\n")

function(configure_consumer build_dir)
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -S "${consumer_src}" -B "${build_dir}"
                -G "${GENERATOR}"
                "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}"
                "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
                "-DFETCHCONTENT_SOURCE_DIR_PUGIXML=${PUGIXML_SOURCE_DIR}"
                "-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=${GOOGLETEST_SOURCE_DIR}"
                ${ARGN}
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE output)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Consumer configure failed (${ARGN}):\n${output}")
    endif()
endfunction()

function(dry_run_build build_dir)
    execute_process(
        COMMAND "${CMAKE_COMMAND}" --build "${build_dir}" -- -n
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE output)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Consumer dry-run build failed:\n${output}")
    endif()
endfunction()

# --- 1. BUILD_TESTING=OFF ----------------------------------------------------
set(off_dir "${WORK_DIR}/build-testing-off")
configure_consumer("${off_dir}" -DBUILD_TESTING=OFF)
if(EXISTS "${off_dir}/_deps/googletest-src" OR
   EXISTS "${off_dir}/kinemaforge/tests")
    message(FATAL_ERROR
        "BUILD_TESTING=OFF still added KinemaForge tests to the parent project")
endif()
dry_run_build("${off_dir}")

# --- 2. Tests explicitly enabled inside a parent project ---------------------
set(on_dir "${WORK_DIR}/build-tests-on")
configure_consumer("${on_dir}" -DKINEMAFORGE_BUILD_TESTS=ON)
if(NOT EXISTS "${on_dir}/kinemaforge/tests")
    message(FATAL_ERROR "KINEMAFORGE_BUILD_TESTS=ON did not add the tests")
endif()
dry_run_build("${on_dir}")

message(STATUS "KinemaForge add_subdirectory consumer check passed")
