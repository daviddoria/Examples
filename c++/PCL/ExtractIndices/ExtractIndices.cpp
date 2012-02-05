#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

int main (int argc, char** argv)
{
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> CloudType;
  CloudType::Ptr cloud (new CloudType);
  cloud->is_dense = false;
  PointType p;
  for(unsigned int i = 0; i < 5; ++i)
  {
    p.x = i; p.y = i; p.z = i;
    cloud->push_back(p);
  }

  std::cout << "Cloud has " << cloud->points.size() << " points." << std::endl;

  pcl::PointIndices indices;
  indices.indices.push_back(0);
  indices.indices.push_back(2);

  pcl::ExtractIndices<PointType> extractIndices;
  extractIndices.setIndices(boost::make_shared<const pcl::PointIndices> (indices));
  extractIndices.setInputCloud(cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  extractIndices.filter(*output);

  std::cout << "Output has " << output->points.size() << " points." << std::endl;
  return (0);
}
