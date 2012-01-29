#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/fusion/include/io.hpp>

#include <iostream>
#include <string>

struct employee
{
    int age;
    std::string name;
    double salary;
};

BOOST_FUSION_ADAPT_STRUCT(
    employee,
    (int, age)
    (std::string, name)
    (double, salary)
)

int main()
{

  return 0;
}
