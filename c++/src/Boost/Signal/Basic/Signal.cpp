#include <boost/signal.hpp>
#include <iostream>

void func()
{
  std::cout << "Hello, world!" << std::endl;
}

int main()
{
  boost::signal<void ()> s;
  s.connect(func);
  s();
}