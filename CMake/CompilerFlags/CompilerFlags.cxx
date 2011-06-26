#include <iostream>

int main()
{
#ifdef UNIX
  std::cout << "UNIX was defined." << std::endl;
#else
  std::cout << "UNIX was NOT defined." << std::endl;
#endif

#ifdef DAVID
  std::cout << "DAVID was defined." << std::endl;
#else
  std::cout << "DAVID was NOT defined." << std::endl;
#endif

#if defined(UNIX) || defined(DAVID)
  std::cout << "Working." << std::endl;
#else
  #error "Broken!"
#endif

return 0;
}
