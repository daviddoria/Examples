#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/bind.hpp>

#include <iostream>

namespace qi = boost::spirit::qi;

void Test(int a)
{
  std::cout << a << std::endl;
}

int main()
{
  using boost::spirit::qi::float_;
  using boost::spirit::qi::parse;

  std::string stringToParse = "(45)";
  std::string::iterator iter = stringToParse.begin();

  parse(iter, stringToParse.end(), '(' >> *float_[&Test] >> ')');

  return 0;
}
