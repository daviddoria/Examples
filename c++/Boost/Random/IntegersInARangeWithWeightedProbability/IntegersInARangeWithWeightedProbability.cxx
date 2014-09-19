#include "boost/random/mersenne_twister.hpp"
#include "boost/random/uniform_real.hpp"
#include "boost/random/variate_generator.hpp"

#include <vector>
#include <algorithm>
#include <numeric>
#include <iostream>

boost::mt19937 generator;

std::vector< double > ProbabilityGenerator( int LowerBound, int UpperBound,
                                            std::vector< int > SpecialValues )
{
  size_t N = static_cast< size_t >( UpperBound - LowerBound + 1 );
  double value = 1. / static_cast< double >( N + 19 * SpecialValues.size() );

  std::vector< double > probabilities( N, value );

  for( std::vector< int >::iterator it = SpecialValues.begin();
        it != SpecialValues.end();
       ++it )
    {
    probabilities[ *it - LowerBound ] = 20. * value;
    }

  return probabilities;
}

int distribution( std::vector< double > probability )
{
  std::vector< double > cumulative;

  std::partial_sum( probability.begin(), probability.end(), std::back_inserter( cumulative ) );
  boost::uniform_real<> distribution( 0., cumulative.back() );
  boost::variate_generator< boost::mt19937&, boost::uniform_real<> > randomizer( generator, distribution );

  double value = randomizer();

  return ( std::lower_bound( cumulative.begin(), cumulative.end(), value ) - cumulative.begin() );
}

int main( int , char * [] )
{
  int LowerBound = 1;
  int UpperBound = 100;

  std::vector< int > SpecialValues;
  SpecialValues.push_back( 8 );
  SpecialValues.push_back( 24 );
  SpecialValues.push_back( 36 );

  std::vector< double > probability = ProbabilityGenerator( LowerBound, UpperBound, SpecialValues );

  int NumberOfIterations = 25;

  for( int i = 0; i < NumberOfIterations; i++ )
    {
    std::cout << i <<" ** " << LowerBound + distribution( probability ) << std::endl;
    }

  return EXIT_SUCCESS;
}

