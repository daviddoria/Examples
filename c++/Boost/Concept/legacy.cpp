#include <boost/concept_check.hpp>

// This was the proposed syntax
// template <typename T>
// where EqualityComparable<T>
// void Test()
// {
//   
// }

template <typename T>
void Test()
{
  BOOST_CONCEPT_ASSERT((EqualityComparable<T>));
}

int main()
{
  Test<double>();
  
  return 0;
}