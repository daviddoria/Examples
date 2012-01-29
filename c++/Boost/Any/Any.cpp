#include <iostream>
#include <vector>
#include <string>

#include <boost/any.hpp>

int main()
{
  std::vector<boost::any> v;

  v.push_back(1);
  v.push_back(std::string("test"));

  for(unsigned int i = 0; i < v.size(); ++i)
    {
    std::cout << boost::any_cast<std::string>(v[i]) << std::endl;
    }
  return 0;
}