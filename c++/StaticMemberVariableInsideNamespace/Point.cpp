#include "Point.h"
#include <iostream>

namespace David
{

  Point::Point()
  {
    std::cout << Point::X << std::endl;
  }

  Subpoint::Subpoint()
  {
    std::cout << Point::X << std::endl;
  }

  Subpoint2::Subpoint2()
  {
    std::cout << Point::X << std::endl;
  }

  void Subpoint::MyFunction()
  {
    std::cout << Point::X << std::endl;
  }

  void Subpoint2::MyFunction()
  {
    std::cout << Point::X << std::endl;
  }
}
