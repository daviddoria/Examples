#include <iostream>

namespace Namespaces
{
  double add(const double a, const double b)
  {
	  return a + b;
  }
}

int main(int argc, char *argv[])
{

  //cout << add(3.4, 4.2); //does not work - "add was not declared in this scope"

  std::cout << Namespaces::add(3.4, 4.2);

  std::cout << ::Namespaces::add(3.4, 4.2); // This is the same as Namespaces::add . The leading :: is like '/' preceeding a filename - it specifies "start from the root/global namespace"

  return 0;
}
