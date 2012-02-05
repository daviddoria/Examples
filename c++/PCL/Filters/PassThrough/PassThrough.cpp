// STL
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main (int argc, char** argv)
{
  typedef pcl::PointCloud<pcl::PointXYZ> InputCloudType;
  InputCloudType::Ptr cloud (new InputCloudType);
  cloud->is_dense = false;

  InputCloudType::PointType p; p.x = 1; p.y = 2; p.z = 3;
  cloud->push_back(p);

  std::cout << "size: " << cloud->points.size () << std::endl;

  //typedef pcl::PointCloud<pcl::PointXYZ> OutputCloudType;
  typedef InputCloudType OutputCloudType;
  OutputCloudType::Ptr outputCloud (new OutputCloudType);

  pcl::PassThrough<OutputCloudType::PointType> passThroughFilter;
  passThroughFilter.setInputCloud (cloud);
  passThroughFilter.filter (*outputCloud);

  return 0;
}
