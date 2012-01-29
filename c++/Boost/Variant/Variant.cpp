#include <iostream>

#include <boost/variant.hpp>

void Streaming();
void Get();

int main(int, char* [])
{
  //Streaming();
  Get();
  return 0;
}


void Streaming()
{
  boost::variant< int, std::string > v;
  v = "hello";
  std::cout << v << std::endl;
  v = 21;
  std::cout << v << std::endl;

}

void Get()
{
  boost::variant< int, std::string > v;
  v = "hello";
  std::string mystring = boost::get<std::string>(v);
  std::cout << mystring << std::endl;
  //int myint = boost::get<int>(v); // this would fail
  //std::cout << myint << std::endl;

  v = 21;
  int myint = boost::get<int>(v);
  std::cout << myint << std::endl;
}
