#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/point_cloud.h>

// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php#how-to-add-a-new-pointt-type

struct MyPointType : public pcl::PointXYZ
{
  std::string name;
};
// POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType); // I don't think this is necessary

int main (int argc, char** argv)
{
  pcl::PointCloud<MyPointType>::Ptr cloud (new pcl::PointCloud<MyPointType>);

  if (pcl::io::loadPCDFile<MyPointType> ("test_pcd.pcd", *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
//   std::cout << "Loaded "
//             << cloud->width * cloud->height
//             << " data points from test_pcd.pcd with the following fields: "
//             << std::endl;
//   for (size_t i = 0; i < cloud->points.size (); ++i)
//     std::cout << "    " << cloud->points[i].x
//               << " "    << cloud->points[i].y
//               << " "    << cloud->points[i].z << std::endl;

  return (0);
}
