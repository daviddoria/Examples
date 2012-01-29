#include <boost/concept_check.hpp>

struct GoodTestStruct
{
  int boundary_point;
};

struct BadTestStruct
{

};

template <typename T>
struct HasBoundaryPointConcept
{
  T object;
  BOOST_CONCEPT_USAGE(HasBoundaryPointConcept)
  {
    int bp = object.boundary_point;
  };

};

template <typename T>
void Test()
{
  BOOST_CONCEPT_ASSERT((HasBoundaryPointConcept<T>));
}

int main()
{
  // Test<BadTestStruct>(); // Fails concept check
  Test<GoodTestStruct>(); // Passes concept check

  return 0;
}
