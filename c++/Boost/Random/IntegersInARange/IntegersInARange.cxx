#include "boost/random/mersenne_twister.hpp"
#include "boost/random/uniform_int.hpp"
#include "boost/random/variate_generator.hpp"

#include <iostream>

boost::mt19937 generator;

int distribution( int LowerBound, int UpperBound )
{
  boost::uniform_int<> distribution( LowerBound, UpperBound );
  boost::variate_generator< boost::mt19937&, boost::uniform_int<> > randomizer( generator, distribution );
  return randomizer();
}

int main( int , char * [] )
{
  int LowerBound = 1;
  int UpperBound = 100;

  int NumberOfIterations = 10;

  for( int i = 0; i < NumberOfIterations; i++ )
    {
    std::cout << i <<" ** " << distribution( LowerBound, UpperBound ) << std::endl;
    }

  return EXIT_SUCCESS;
}

