#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  for(unsigned int i = 0; i < 10; ++i)
  {
    pcl::PointXYZ p; p.x = i; p.y = i; p.z = i;
    cloud->points.push_back(p);
  }

  pcl::PointXYZ p; p.x = 5.1; p.y = 5.1; p.z = 5.1;
  cloud->points.push_back(p);

  std::cout << "Number of points before: " << cloud->points.size() << std::endl;

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud (cloud);
  voxel_grid.setLeafSize (0.5, 0.5, 0.5);
  voxel_grid.filter(*cloud);

  std::cout << "Number of points after: " << cloud->points.size() << std::endl;

  return (0);
}
