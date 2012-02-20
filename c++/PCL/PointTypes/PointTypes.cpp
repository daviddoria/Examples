#include <iostream>

#include <pcl/point_types.h>

int main (int argc, char** argv)
{
  pcl::PointXYZ p;
  p.getVector3fMap();
  
  pcl::Normal n;
  // n.getVector3fMap(); // Can't do this
  return 0;
}
