//  This sample demontrates a parser for a comma separated list of numbers.

#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>

#include <iostream>
#include <string>
#include <vector>

namespace client
{
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;

    ///////////////////////////////////////////////////////////////////////////
    //  Our number list parser
    ///////////////////////////////////////////////////////////////////////////
    //[tutorial_numlist1
    template <typename Iterator>
    bool parse_numbers(Iterator first, Iterator last)
    {
        using qi::double_;
        using qi::phrase_parse;
        using ascii::space;

        bool r = phrase_parse(
            first,                          /*< start iterator >*/
            last,                           /*< end iterator >*/
            double_ >> *(',' >> double_),   /*< the parser >*/
            space                           /*< the skip-parser >*/
        );
        if (first != last) // fail if we did not get a full match
            return false;
        return r;
    }
    //]
}

////////////////////////////////////////////////////////////////////////////
//  Main program
////////////////////////////////////////////////////////////////////////////
int
main()
{
    std::string str = "1,2, 3";
    if (client::parse_numbers(str.begin(), str.end()))
      {
          std::cout << "-------------------------\n";
          std::cout << "Parsing succeeded\n";
          std::cout << str << " Parses OK: " << std::endl;
      }
    else
      {
          std::cout << "-------------------------\n";
          std::cout << "Parsing failed\n";
          std::cout << "-------------------------\n";
      }

    return 0;
}