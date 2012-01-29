#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <iostream>


struct F
{
    int operator()(int a, int b) { return a - b; }
    bool operator()(long a, long b) { return a == b; }
};

int main ()
{
F f;

int x = 104;

  boost::bind<int>(f, _1, _1)(x);                // f(x, x), i.e. zero
  return 0;
}