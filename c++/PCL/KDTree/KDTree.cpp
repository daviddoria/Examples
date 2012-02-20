#include <iostream>

#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudType;
  PointCloudType::Ptr cloud (new PointCloudType);

  typedef pcl::search::KdTree<PointCloudType::PointType> TreeType;
  TreeType::Ptr tree (new TreeType);

  return 0;
}
