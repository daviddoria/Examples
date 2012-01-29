#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <iostream>

double Update1(int val)
{
  std::cout << "Update1 " << val << std::endl;
}

double Update2(int val)
{
  std::cout << "Update2 " << val << std::endl;
}
  
int main () 
{
  boost::function<double(int)> f;
  
  f = &Update1;
  f(2.0);
  
  f = &Update2;
  f(3.0);
  
  return 0;
}
