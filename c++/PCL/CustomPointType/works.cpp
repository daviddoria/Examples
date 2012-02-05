#include <pcl/point_types.h>
 #include <pcl/point_cloud.h>
 #include <pcl/io/pcd_io.h>

 struct MyPointType
 {
   PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
   float test;
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
 } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

//  POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,           // here we assume a XYZ + "test" (as fields)
//                                     (float, x, x)
//                                     (float, y, y)
//                                     (float, z, z)
//                                     (float, test, test)
//  )


 int
 main (int argc, char** argv)
 {
   pcl::PointCloud<MyPointType> cloud;
   cloud.points.resize (2);
   cloud.width = 2;
   cloud.height = 1;

   cloud.points[0].test = 1;
   cloud.points[1].test = 2;
   cloud.points[0].x = cloud.points[0].y = cloud.points[0].z = 0;
   cloud.points[1].x = cloud.points[1].y = cloud.points[1].z = 3;

   pcl::io::savePCDFile ("test.pcd", cloud);
 }