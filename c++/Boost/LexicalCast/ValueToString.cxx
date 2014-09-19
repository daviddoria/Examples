#include <iostream>
#include <string>
#include "boost/lexical_cast.hpp"

int main( int argc, char* argv[] )
{
  int   value   = 24;

  std::string integer = boost::lexical_cast< std::string >( value );

  float pi      = 3.14;

  std::string pi_string = boost::lexical_cast< std::string >( pi );


  return EXIT_SUCCESS;
}
