#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>

#include <boost/make_shared.hpp>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // ... fill point cloud...

  cloud.width = 640;
  cloud.height = 480;
  cloud.points.resize (cloud.width * cloud.height);

  for (int ri = 0; ri < cloud.height; ++ri)
  {
    for (int ci = 0; ci < cloud.width; ++ci)
    {
      const float depth = 0.2f*static_cast<float> (rand ()) / static_cast<float>(RAND_MAX) + 1.0f;
      cloud(ri, ci).x = (ci - 320) * depth;
      cloud(ri, ci).y = (ri - 240) * depth;
      cloud(ri, ci).z = depth;
    }
  }

  // Estimate normals
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

  pcl::PointCloud<pcl::Normal> normals;

  ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setInputCloud(boost::make_shared<const pcl::PointCloud<pcl::PointXYZ> >(cloud));
  ne.compute(normals);

  return (0);
}
