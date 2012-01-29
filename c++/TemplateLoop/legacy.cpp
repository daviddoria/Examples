#include <iostream>
#include <vector>

#include <boost/utility/enable_if.hpp>
/*
The point of the enable_if and disable_if in this context is that the implementation is compile-time recursion.
The idea here is that you have GridDimensions number of dimensions, and you want to nest GridDimensions
number of for-loops to traverse all the elements (just like you would nest 2 for-loops to traverse a 2D grid). So,
at first, from the main inpainting function, the loop<0>() function is called to start the first highest-level for-loop
(traversing the 0th dimension). In the for-loop within loop(), it recursively calls loop< Dim + 1 >(), such that another
for-loop is done on the next nested dimension. At the end, when "Dim = GridDimensions - 1", you call loop< Dim + 1 >(),
which switches to the implementation that is enabled with Dim == GridDimensions, which does nothing and thus, terminates
the recursion. This is a basic technique to create a static recursion (which you might need when dealing with static-arrays,
tuples, type-lists, etc.).
*/

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
