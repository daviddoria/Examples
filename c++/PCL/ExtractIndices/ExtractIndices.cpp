#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

int main (int argc, char** argv)
{
  std::string fileName = argv[1];

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
  {
    std::string errorString = std::string("Couldn't read file ") + fileName;
    PCL_ERROR (errorString.c_str());
    return (-1);
  }
  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  pcl::PointIndices indices;
  indices.indices.push_back(0);
  indices.indices.push_back(1);
  indices.indices.push_back(2);

  pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
  extractIndices.setIndices(boost::make_shared<const pcl::PointIndices> (indices));
  extractIndices.setInputCloud(cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  extractIndices.filter(*output);

  std::cout << "Output has " << output->points.size() << " points." << std::endl;
  return (0);
}
