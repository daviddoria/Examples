// STL
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int 
main (int argc, char** argv)
{
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> CloudType;
  CloudType::Ptr cloud (new CloudType);
  cloud->height = 10;
  cloud->width = 10;
  cloud->is_dense = true;
  cloud->resize(cloud->height * cloud->width);

  std::cout << (*cloud)(0,0) << std::endl;

  PointType p; p.x = 1; p.y = 2; p.z = 3;

  (*cloud)(0,0) = p;

  std::cout << (*cloud)(0,0) << std::endl;
  return (0);
}
