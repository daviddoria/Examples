#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/narf.h>
#include <pcl/features/narf_descriptor.h>

int main (int argc, char** argv)
{
  //  Needs a range image
  std::string fileName = argv[1];
  std::cout << "Reading " << fileName << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile <pcl::PointXYZ> (fileName.c_str(), *cloud) == -1)
      // load the file
      {
      PCL_ERROR ("Couldn't read file");
      return (-1);
      }
  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  pcl::RangeImage range_image;
  // Setup the NARF computation
  pcl::NarfDescriptor narfEstimation(&range_image); // optional second paramter of indices at which to compute features
  
  pcl::PointCloud<pcl::Narf36> narf_descriptors;
  narfEstimation.compute (narf_descriptors);
  return 0;
}
