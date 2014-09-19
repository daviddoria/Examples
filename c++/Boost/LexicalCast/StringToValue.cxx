#include <iostream>
#include <string>
#include "boost/lexical_cast.hpp"

int main( int argc, char* argv[] )
{
  std::string integer   = "24";
  std::string floating  = "3.14";
  std::string error     = "error";

  try
    {
    int val0 = boost::lexical_cast< int >( integer );
    std::cout << val0 << std::endl;
    }
  catch( boost::bad_lexical_cast & )
    {
    std::cout << "Wrong conversion" << std::endl;
    }

  try
    {
    double pi = boost::lexical_cast< double >( floating );
    std::cout << pi << std::endl;
    }
  catch( boost::bad_lexical_cast & )
    {
    std::cout << "Wrong conversion" << std::endl;
    }

  try
    {
    double val = boost::lexical_cast< double >( error );
    }
  catch( boost::bad_lexical_cast & )
    {
    std::cout << "Wrong conversion" << std::endl;
    }

  return EXIT_SUCCESS;
}
