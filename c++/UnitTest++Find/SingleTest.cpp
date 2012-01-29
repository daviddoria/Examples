#include <iostream>
#include <UnitTest++.h>

TEST(FailSpectacularly)
{
  CHECK(false);
}

int main()
{
  return UnitTest::RunAllTests();
}
