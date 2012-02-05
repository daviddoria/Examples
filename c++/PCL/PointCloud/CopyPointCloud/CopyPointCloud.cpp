// STL
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

static void SameType();
static void DifferenceType();
// static void BadConversion();

int main (int argc, char** argv)
{
  SameType();
  DifferenceType();
  // BadConversion();
  return 0;
}

void SameType()
{
  typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
  CloudType::Ptr cloud (new CloudType);
  
  CloudType::PointType p;
  p.x = 1; p.y = 2; p.z = 3;
  cloud->push_back(p);
  std::cout << p.x << " " << p.y << " " << p.z << std::endl;

  CloudType::Ptr cloud2(new CloudType);
  copyPointCloud(*cloud, *cloud2);
  
  CloudType::PointType p_retrieved = cloud2->points[0];
  std::cout << p_retrieved.x << " " << p_retrieved.y << " " << p_retrieved.z << std::endl;
}

void DifferenceType()
{
  typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
  CloudType::Ptr cloud (new CloudType);
  
  CloudType::PointType p;
  p.x = 1; p.y = 2; p.z = 3;
  cloud->push_back(p);
  std::cout << p.x << " " << p.y << " " << p.z << std::endl;

  typedef pcl::PointCloud<pcl::PointNormal> CloudType2;
  CloudType2::Ptr cloud2(new CloudType2);
  copyPointCloud(*cloud, *cloud2);
  
  CloudType2::PointType p_retrieved = cloud2->points[0];
  std::cout << p_retrieved.x << " " << p_retrieved.y << " " << p_retrieved.z << std::endl;
}

/*
// Of course you can't do this, because pcl::Normal does not have x,y,z members:
error: ‘pcl::PointCloud<pcl::Normal>::PointType’ has no member named ‘x’

void BadConversion()
{
  typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
  CloudType::Ptr cloud (new CloudType);
  
  CloudType::PointType p;
  p.x = 1; p.y = 2; p.z = 3;
  cloud->push_back(p);
  std::cout << p.x << " " << p.y << " " << p.z << std::endl;

  typedef pcl::PointCloud<pcl::Normal> CloudType2;
  CloudType2::Ptr cloud2(new CloudType2);
  copyPointCloud(*cloud, *cloud2);
  
  CloudType2::PointType p_retrieved = cloud2->points[0];
  std::cout << p_retrieved.x << " " << p_retrieved.y << " " << p_retrieved.z << std::endl;
}

*/
