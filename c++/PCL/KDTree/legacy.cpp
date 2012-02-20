#include <iostream>

#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>

static void Standard();
static void WithNormals();

int main (int argc, char** argv)
{
  Standard();
  WithNormals();
  return 0;
}

void Standard()
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudType;
  PointCloudType::Ptr cloud (new PointCloudType);

  typedef pcl::search::KdTree<PointCloudType::PointType> TreeType;
  TreeType::Ptr tree (new TreeType);

}

void WithNormals()
{
  typedef pcl::PointCloud<pcl::Normal> PointCloudWithNormalsType;

  PointCloudWithNormalsType::Ptr cloudWithNormals (new PointCloudWithNormalsType);

  // Create a tree
  typedef pcl::search::KdTree<PointCloudWithNormalsType::PointType> TreeType;
  TreeType::Ptr tree (new TreeType);

}

