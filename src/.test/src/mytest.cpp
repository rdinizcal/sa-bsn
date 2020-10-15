#include <gtest/gtest.h>
#include <gmock/gmock.h>

TEST(TestSuite, testCase1) {
    EXPECT_EQ(5, 5);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
