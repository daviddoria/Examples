#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/point_traits.h>
#include <pcl/filters/voxel_grid.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter(*cloud);
  
  return (0);
}
