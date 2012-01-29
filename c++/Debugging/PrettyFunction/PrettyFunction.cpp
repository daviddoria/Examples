#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>

void Test();
void Test(const int a);

int main(int argc, char *argv[])
{
  Test();
  Test(2);
  return 0;
}

void Test()
{
  std::cout << __PRETTY_FUNCTION__ << '\n';

}

void Test(const int a)
{
  std::cout << __PRETTY_FUNCTION__ << '\n';

}
