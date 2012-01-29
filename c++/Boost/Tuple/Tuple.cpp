#include <boost/tuple/tuple.hpp>

#include <iostream>

int main ()
{
  boost::tuple<float, std::string, bool> myTupleTest;
  boost::tuple<float, std::string, bool> myTuple(2.0, "hello", false);
  // std::cout << myTuple << std::endl; // Can't do this
  
  float a = myTuple.get<0>();
  std::cout << a << std::endl;
  
  std::string b = myTuple.get<1>();
  std::cout << b << std::endl;
  
  bool c = myTuple.get<2>();
  std::cout << c << std::endl;
  
  return 0;
}
