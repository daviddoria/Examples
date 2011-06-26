#include <boost/signal.hpp>
#include <boost/bind.hpp>

#include <iostream>

class A
{
public:
  boost::signal<void ()> MySignal;
};

class B
{
public:
  void MySlot() {std::cout << "Slot called." << std::endl;}
};

int main()
{
  A a;
  B b;

  a.MySignal.connect(boost::bind(&B::MySlot, &b));
  a.MySignal();
}