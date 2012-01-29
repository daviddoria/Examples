#include <iostream>
#include <vector>
#include <cstdlib>

#include <cmath>

#define BOOST_TEST_MODULE MyTest
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test.hpp>
unsigned int sum(const unsigned int a, const unsigned int b)
{
  return a+b;
}

test_suite*
init_unit_test_suite( int argc, char* argv[] )
{
    test_suite* test = BOOST_TEST_SUITE( "Master test suite" );

    test->add( BOOST_TEST_CASE( &sum ) );

    return test;
}
