// STL
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/make_shared.hpp>

int main (int argc, char** argv)
{
  typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
  CloudType::Ptr cloud(new CloudType);

  // These are not valid syntax
  //cloud = boost::make_shared<CloudType>(new CloudType);
  //cloud = boost::make_shared<const CloudType>(CloudType);
  //cloud = new CloudType;

  // This works
  cloud = boost::make_shared<CloudType>(*(new CloudType));

  return 0;
}
