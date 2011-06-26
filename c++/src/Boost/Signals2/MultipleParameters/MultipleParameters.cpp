#include <boost/signals2/signal.hpp>
#include <iostream>

void func(float value1, int value2)
{
  std::cout << "value1 is " << value1 << std::endl;
  std::cout << "value2 is " << value2 << std::endl;
}

int main()
{
  boost::signals2::signal<void (float, int)> s;
  s.connect(func);
  s(5.1, 2);
}