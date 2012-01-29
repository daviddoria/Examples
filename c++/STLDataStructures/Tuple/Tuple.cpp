#include <iostream>
#include <tuple>

int main (int argc, char *argv[]) 
{
  std::tuple<int,char,std::string> tup( 4, 'c', std::string( "Hello World" ) );
  std::cout << std::get<0>(tup) << std::endl;
  std::cout << std::get<1>(tup) << std::endl;
  std::cout << std::get<2>(tup) << std::endl;

  return 0;
}

