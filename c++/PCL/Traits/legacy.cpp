#include <iostream>

#include <pcl/point_types.h>

int main (int argc, char** argv)
{
  typedef pcl::PointXYZ PointType;
  
  typedef typename pcl::traits::fieldList<PointType>::type FieldListT;
  typedef typename pcl::traits::fieldList<PointType>::value FieldListValueT;
  
  FieldListT fieldList = PointType::;
  
  //std::cout << fieldList << std::endl;
  //std::cout << fieldList[0] << std::endl;

  return (0);
}
