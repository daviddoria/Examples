#include <iostream>
#include <vector>

#include <boost/utility/enable_if.hpp>

typedef std::vector<std::vector<int> > GridType;

template <std::size_t GridDimensions>
struct TestClass
{
  template <std::size_t Dim>
  typename boost::enable_if_c< (GridDimensions == Dim),
  void >::type loop(GridType& grid) { };

  template <std::size_t Dim>
  typename boost::disable_if_c< (GridDimensions == Dim),
  void >::type loop(GridType& grid)
  {
    for(std::size_t i = 0; i < grid.size(); ++i)
    {
      // loop in the nested dimension:
      loop< Dim + 1 >(grid);
    };

  }; // end loop()

}; // end TestClass

int main(int argc, char *argv[])
{
  std::vector<std::vector<int> > values;
  for(unsigned int i = 0; i < 10; ++i)
    {
    std::vector<int> vec;
    for(unsigned int j = 0; j < 5; ++j)
      {
      vec.push_back(j);
      }
    values.push_back(vec);
    }
  TestClass<2> test;
  test.loop<0>(values);
  return 0;
}
