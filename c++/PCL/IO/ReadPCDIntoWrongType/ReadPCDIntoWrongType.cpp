#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> OutputCloudType;
typedef pcl::PointCloud<pcl::Normal> InputCloudType;

static void OutputTestFile();

int
main (int argc, char** argv)
{
  OutputTestFile();

  InputCloudType::Ptr cloud (new InputCloudType);

  //if (pcl::io::loadPCDFile<InputCloudType::PointType> ("test.pcd", *cloud) == -1)
  if (pcl::io::loadPCDFile("test.pcd", *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file test.pcd \n");
    return (-1);
  }
  
  return (0);
}

void OutputTestFile()
{
  OutputCloudType::Ptr cloud (new OutputCloudType);
  cloud->height = 1;
  cloud->is_dense = false;

  for(unsigned int i = 0; i < 20; ++i)
  {
    OutputCloudType::PointType p;
    cloud->push_back(p);
  }
  pcl::io::savePCDFileASCII ("test.pcd", *cloud);

}
