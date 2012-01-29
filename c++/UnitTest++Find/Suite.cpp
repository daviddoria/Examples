#include <iostream>
#include <UnitTest++.h>

SUITE(YourSuiteName)
{
  TEST(YourTestName)
  {
    CHECK(false);
  }

  TEST(YourOtherTestName)
  {
  }
}

int main()
{
  return UnitTest::RunAllTests();
}

