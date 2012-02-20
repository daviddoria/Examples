// STL
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

int main (int argc, char** argv)
{
  sensor_msgs::PointCloud2 blob; 
  //pcl::io::loadPCDFile ("file.pcd", blob); 
  // pcl::fromROSMsg (cloud_blob, *cloud); //* convert from sensor_msgs/PointCloud2 to pcl::PointCloud<T>
  std::cout << "There are " << blob.fields.size() << " fields." << std::endl;
  for(unsigned int i = 0; i < blob.fields.size(); ++i)
  {
    std::cout << "Field " << i << " : " << blob.fields[i] << std::endl;
  }

  return 0;
}
/*
  // Setup points only
  typedef pcl::PointCloud<pcl::PointXYZ> CloudPointType;
  CloudPointType::Ptr cloudPoints(new CloudPointType);

  CloudPointType::PointType p;
  p.x = 1; p.y = 2; p.z = 3;
  cloudPoints->push_back(p);
  
  // Setup normals only
  typedef pcl::PointCloud<pcl::Normal> CloudNormalType;
  CloudNormalType::Ptr cloudNormals(new CloudNormalType);
  
  CloudNormalType::PointType n;
  n.normal_x = 4; n.normal_y = 5; n.normal_z = 6;
  cloudNormals->push_back(n);

  // Concatenate
  typedef pcl::PointCloud<pcl::PointNormal> CloudPointNormalType;
  CloudPointNormalType::Ptr cloudPointNormals(new CloudPointNormalType);
  pcl::concatenateFields (*cloudPoints, *cloudNormals, *cloudPointNormals);
  
  CloudPointNormalType::PointType p_retrieved = cloudPointNormals->points[0];
  std::cout << p_retrieved.x << " " << p_retrieved.y << " " << p_retrieved.z << std::endl;
  std::cout << p_retrieved.normal_x << " " << p_retrieved.normal_y << " " << p_retrieved.normal_z << std::endl;*/