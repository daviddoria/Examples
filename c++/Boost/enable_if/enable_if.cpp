#include <iostream>

#include <boost/utility/enable_if.hpp>

template <std::size_t Dim>
typename boost::enable_if_c< (Dim),
void >::type loop() { }

int main (int argc, char *argv[])
{

  return 0;
}
