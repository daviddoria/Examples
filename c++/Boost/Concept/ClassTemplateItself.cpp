#include <boost/concept_check.hpp>

template <typename T>
class Test
{
  // BOOST_CONCEPT_ASSERT((boost::EqualityComparable<Test<T> >)); // Can't do this - doesn't make sense
};

int main()
{
  Test<double>();
  
  return 0;
}
