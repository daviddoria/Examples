#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/bind.hpp>

#include <iostream>

namespace qi = boost::spirit::qi;

// A plain function
void print(int const& i)
{
    std::cout << i << std::endl;
}

int main()
{
    using boost::spirit::qi::int_;
    using boost::spirit::qi::parse;

    std::string stringToParse = "test {42}";
    // parse(stringToParse.begin(), stringToParse.end(), '{' >> int_[&print] >> '}'); // Can't use a temp iterator
    std::string::iterator iter = stringToParse.begin();
    parse(iter, stringToParse.end(), '{' >> int_[&print] >> '}');

    return 0;
}
