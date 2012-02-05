// STL
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{
  typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
  CloudType::Ptr cloud (new CloudType);
  
  CloudType::PointType p;
  std::cout << "    " << p.x << " "    << p.y << " "    << p.z << std::endl;
  p.x = 1.2;
  std::cout << "    " << p.x << " "    << p.y << " "    << p.z << std::endl;

  std::cout << "size: " << cloud->points.size () << std::endl;

  cloud->push_back(p);

  std::cout << "size: " << cloud->points.size () << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    std::cout << "    " << cloud->points[i].x << " "    << cloud->points[i].y << " "    << cloud->points[i].z << std::endl;
  }

  return 0;
}
