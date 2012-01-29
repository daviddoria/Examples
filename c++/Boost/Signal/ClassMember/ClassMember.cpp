#include <boost/signal.hpp>
#include <boost/bind.hpp>

#include <iostream>

class Dog
{
public:
  boost::signal<void ()> Bark;
};

class Person
{
private:
  Dog Lassy;
public:
  Person()
  {
    Lassy.Bark.connect(boost::bind(&Person::HearBark, this));
  }

  void HearBark()
  {
    std::cout << "I heard you bark!" << std::endl;
  }

  void CauseDogToBark()
  {
    Lassy.Bark();
  }

};

int main()
{
  Person david;
  david.CauseDogToBark();
}