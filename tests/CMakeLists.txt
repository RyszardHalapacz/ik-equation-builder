include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)
# Windows: use the same dynamic CRT as the rest of the project
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(kinemaforge_tests
    test_kinematics.cpp
    test_urdf_model_loader.cpp
    test_kinematic_chain_builder.cpp
    test_joint_transform_builder.cpp
    test_forward_kinematics_builder.cpp
    test_symbolic_expression.cpp
    test_expression_factory.cpp
    test_expression_evaluator.cpp
    test_symbolic_matrix.cpp
    test_symbolic_transform.cpp
)
target_link_libraries(kinemaforge_tests PRIVATE kinemaforge_ik GTest::gtest_main)
target_compile_definitions(kinemaforge_tests PRIVATE
    KINEMAFORGE_URDF_DATA_DIR="${CMAKE_SOURCE_DIR}/data/urdf"
)

include(GoogleTest)
gtest_discover_tests(kinemaforge_tests)
