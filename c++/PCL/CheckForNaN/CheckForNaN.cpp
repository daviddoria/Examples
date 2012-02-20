#include <iostream>
#include <limits>

#include <pcl/point_types.h>

int main (int argc, char** argv)
{
  typedef pcl::PointXYZ PointType;

  PointType p;
  p.x = 1; p.y = 2; p.z = 3;
  std::cout << pcl_isnan(p);
  
  PointType p_full_nan;
  p_full_nan.x = std::numeric_limits<float>::quiet_NaN(); p_full_nan.y = 2; p_full_nan.z = 3;
  std::cout << pcl_isnan(p_full_nan);
  
  PointType p_partial_nan;
  p_partial_nan.x = std::numeric_limits<float>::quiet_NaN(); p_partial_nan.y = 2; p_partial_nan.z = 3;
  std::cout << pcl_isnan(p_partial_nan);
  
  return (0);
}
