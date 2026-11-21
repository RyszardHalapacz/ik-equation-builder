include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)
# Windows: use the same dynamic CRT as the rest of the project
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(kinemaforge_generate_test_fixtures generate_ik_test_fixtures.cpp)
target_link_libraries(kinemaforge_generate_test_fixtures PRIVATE
    kinemaforge_ik kinemaforge_warnings)
target_compile_definitions(kinemaforge_generate_test_fixtures PRIVATE
    KINEMAFORGE_URDF_DATA_DIR="${PROJECT_SOURCE_DIR}/data/urdf"
    KINEMAFORGE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
set(ik_fixture_dir "${CMAKE_CURRENT_BINARY_DIR}/fresh_generated")
set(ik_fixture_headers
    "${ik_fixture_dir}/kr640_fresh.hpp"
    "${ik_fixture_dir}/kr4_fresh.hpp"
    "${ik_fixture_dir}/kr640_tcp_fresh.hpp"
    "${ik_fixture_dir}/equal_links_fresh.hpp")
add_custom_command(OUTPUT ${ik_fixture_headers}
    COMMAND $<TARGET_FILE:kinemaforge_generate_test_fixtures> "${ik_fixture_dir}"
    DEPENDS kinemaforge_generate_test_fixtures
        "${PROJECT_SOURCE_DIR}/data/urdf/kr640.urdf"
        "${PROJECT_SOURCE_DIR}/data/urdf/kr4_r600.urdf"
        "${CMAKE_CURRENT_SOURCE_DIR}/data/equal_links_6r.urdf"
    VERBATIM)

add_executable(kinemaforge_tests
    ${ik_fixture_headers}
    test_kinematics.cpp
    test_ik_equation_builder.cpp
    test_urdf_model_loader.cpp
    test_kinematic_chain_builder.cpp
    test_joint_transform_builder.cpp
    test_forward_kinematics_builder.cpp
    test_constraint_builder.cpp
    test_directional_constraints.cpp
    test_numeric_fk_validation.cpp
    test_tcp_transform.cpp
    test_ik_equation_model.cpp
    test_target_validation.cpp
    test_quaternion_target.cpp
    test_symbolic_expression.cpp
    test_expression_factory.cpp
    test_expression_evaluator.cpp
    test_rewrite_engine.cpp
    test_expression_canonicalizer.cpp
    test_trigonometric_simplifier.cpp
    test_kinematic_geometry.cpp
    test_ik_pattern_detector.cpp
    test_primitive_solvers.cpp
    test_analytic_ik_solver.cpp
    test_analytic_ik_regressions.cpp
    test_ik_tolerances.cpp
    test_analytic_ik_kernels.cpp
    test_analytic_ik_compact.cpp
    test_analytic_ik_positional_families.cpp
    test_code_generator.cpp
    test_symbolic_matrix.cpp
    test_symbolic_transform.cpp
    support/NumericForwardKinematics.cpp
    support/TransformComparison.cpp
)
target_link_libraries(kinemaforge_tests PRIVATE
    kinemaforge_ik GTest::gtest_main kinemaforge_warnings)
target_include_directories(kinemaforge_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
target_compile_definitions(kinemaforge_tests PRIVATE
    KINEMAFORGE_URDF_DATA_DIR="${PROJECT_SOURCE_DIR}/data/urdf"
    KINEMAFORGE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)

include(GoogleTest)
gtest_discover_tests(kinemaforge_tests)

# Consuming KinemaForge through add_subdirectory() (as MotionBridge does).
# Ninja only: the check relies on `ninja -n` failing on a missing input file.
if(CMAKE_GENERATOR MATCHES "Ninja")
    add_test(NAME cmake_subproject_consumer
        COMMAND "${CMAKE_COMMAND}"
            "-DKINEMAFORGE_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
            "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/subproject_consumer"
            "-DGENERATOR=${CMAKE_GENERATOR}"
            "-DMAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}"
            "-DCXX_COMPILER=${CMAKE_CXX_COMPILER}"
            "-DPUGIXML_SOURCE_DIR=${pugixml_SOURCE_DIR}"
            "-DGOOGLETEST_SOURCE_DIR=${googletest_SOURCE_DIR}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_subproject_consumer.cmake")
endif()
