#include <iostream>
#include <vector>
#include <cstdlib>

#include <cmath>

#define BOOST_TEST_MODULE MyTest
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

unsigned int SUM(const unsigned int a, const unsigned int b)
{
  return a+b;
}

BOOST_AUTO_TEST_CASE( my_test )
{
  BOOST_CHECK( SUM(2,3)==5 );
  BOOST_CHECK( SUM(2,3)==6 );
}

/*
int main()
{
  return 0;
}
*/