// STL
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{
  typedef pcl::PointXYZRGB PointType;
  typedef pcl::PointCloud<PointType> CloudType;
  CloudType::Ptr cloud (new CloudType);

  CloudType::PointType p;
  p.x = 1; p.y = 2; p.z = 3; p.r = 5; p.g = 10; p.b = 15;
  std::cout << p << std::endl;
  std::cout << p.x << " "    << p.y << " "    << p.z << " " << static_cast<int>(p.r) << " " << static_cast<int>(p.g) << " " << static_cast<int>(p.b) << std::endl;

  std::cout << "size: " << cloud->points.size () << std::endl;

  cloud->push_back(p);

  return 0;
}
