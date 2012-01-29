#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/bind.hpp>

#include <iostream>

namespace qi = boost::spirit::qi;

int main()
{
  using boost::spirit::qi::float_;
  using boost::spirit::qi::parse;

  std::string stringToParse = "(1.2,3.45,6,78)";
  std::string::iterator iter = stringToParse.begin();

  std::vector<float> v;
  parse(iter, stringToParse.end(), '(' >> qi::float_[boost::bind(&std::vector<float>::push_back, &v, _1)] >> *(',' >> float_)[boost::bind(&std::vector<float>::push_back, &v, _1)] >> ')'); // works

  for (unsigned int i = 0; i < v.size(); ++i)
    {
    std::cout << v[i] << std::endl;
    }
  return 0;
}
