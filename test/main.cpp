#include <gtest/gtest.h>

TEST(DavidsTester, basicTest){
  EXPECT_TRUE(true);
}

int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
