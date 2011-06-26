#include <iostream>

int main(int argc, char *argv[])
{
#ifdef TESTFLAG
std::cout << "TESTFLAG defined" << std::endl;
#else
std::cout << "TESTFLAG not defined" << std::endl;
#endif
  return 0;
}
