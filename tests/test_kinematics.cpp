#include <gtest/gtest.h>

#include "legacy/Kinematics.h"

TEST(KinematicsTest, ExampleForwardAtZero) {
    EXPECT_DOUBLE_EQ(kinemaforge::example_forward(0.0), 0.0);
}
