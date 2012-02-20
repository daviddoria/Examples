// STL
#include <iostream>

// PCL
#include <pcl/PointIndices.h>

int main (int argc, char** argv)
{
  pcl::PointIndices indices;
  indices.indices.push_back(0);
  indices.indices.push_back(2);

  return 0;
}
