#include <boost/signals2/signal.hpp>
#include <iostream>

void func()
{
  std::cout << "Hello, world!" << std::endl;
}

int main()
{
  boost::signals2::signal<void ()> s;
  s.connect(func);
  s();
}