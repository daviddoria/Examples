#include <boost/signals2/signal.hpp>
#include <iostream>

void func(float value)
{
  std::cout << "value is " << value << std::endl;
}

int main()
{
  boost::signals2::signal<void (float)> s;
  s.connect(func);
  s(5.1);
}