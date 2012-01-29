#include <boost/concept_check.hpp>

template <typename T>
void Test()
{
  BOOST_CONCEPT_ASSERT((boost::EqualityComparable<T>));
}

int main()
{
  Test<double>();
  
  return 0;
}
