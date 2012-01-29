#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/bind.hpp>

#include <iostream>

namespace qi = boost::spirit::qi;

struct Point
{
  Point() : x(0), y(0) {}
  void SetX(float x_in) {x = x_in;}
  void SetY(float y_in) {y = y_in;}
  float x;
  float y;
};

int main()
{
  using boost::spirit::qi::float_;
  using boost::spirit::qi::parse;

  std::string stringToParse = "(1.2, 3.45)";
  std::string::iterator iter = stringToParse.begin();
  Point p;

  parse(iter, stringToParse.end(), '(' >> *qi::space >> float_[boost::bind(&Point::SetX, &p, _1)] >> *qi::space >> ',' >> *qi::space >> float_[boost::bind(&Point::SetY, &p, _1)] >> *qi::space >> ')');

  std::cout << p.x << std::endl;
  std::cout << p.y << std::endl;
  return 0;
}
