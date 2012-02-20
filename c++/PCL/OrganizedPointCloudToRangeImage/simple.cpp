// STL
#include <iostream>

// PCL
#include <pcl/range_image/range_image.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

static float ComputeAngularResolutionWidth(CloudType::ConstPtr cloud);
static float ComputeAngularResolutionHeight(CloudType::ConstPtr cloud);
static float ComputeMaxAngleWidth(CloudType::ConstPtr cloud);
static float ComputeMaxAngleHeight(CloudType::ConstPtr cloud);

template <typename TPoint>
static float AngleBetweenPointVectors(const TPoint& p0, const TPoint& p1)
{
  Eigen::Vector4f v0;
  v0[0] = p0.x; v0[1] = p0.y; v0[2] = p0.z; v0[3] = 1;
  Eigen::Vector4f v1;
  v1[0] = p1.x; v1[1] = p1.y; v1[2] = p1.z; v1[3] = 1;
  return pcl::getAngle3D (v0, v1);
}

static void CreateFromPointCloud(CloudType::ConstPtr cloud, pcl::RangeImage& range_image);

int main (int argc, char** argv)
{
  // Create an organized point cloud
  CloudType::Ptr cloud (new CloudType);
  cloud->height = 100;
  cloud->width = 100;
  cloud->is_dense = true;
  cloud->resize(cloud->height * cloud->width);

  // Make a scan of a square
  for(unsigned int i = 0; i < cloud->width; ++i)
  {
    for(unsigned int j = 0; j < cloud->width; ++j)
    {
      PointType p;
      p.x = i; p.y = j; p.z = 10;
      (*cloud)(i,j) = p;
    }
  }

  pcl::RangeImage range_image;

  CreateFromPointCloud(cloud, range_image);
  return 0;
}

float ComputeAngularResolutionWidth(CloudType::ConstPtr cloud)
{
  PointType p0 = (*cloud)(0,0);
  PointType p1 = (*cloud)(1,0);
  return AngleBetweenPointVectors(p0, p1);
}

float ComputeAngularResolutionHeight(CloudType::ConstPtr cloud)
{
  PointType p0 = (*cloud)(0,0);
  PointType p1 = (*cloud)(0,1);
  return AngleBetweenPointVectors(p0, p1);
}

float ComputeMaxAngleWidth(CloudType::ConstPtr cloud)
{
  PointType p0 = (*cloud)(0,0);
  PointType p1 = (*cloud)(cloud->width - 1,0);
  return AngleBetweenPointVectors(p0, p1);
}

float ComputeMaxAngleHeight(CloudType::ConstPtr cloud)
{
  PointType p0 = (*cloud)(0,0);
  PointType p1 = (*cloud)(0,cloud->height - 1);
  return AngleBetweenPointVectors(p0, p1);
}

void CreateFromPointCloud(CloudType::ConstPtr cloud, pcl::RangeImage& range_image)
{
  float angular_resolution_width = ComputeAngularResolutionWidth(cloud);
  //float angular_resolution_height = ComputeAngularResolutionHeight(cloud);
  float max_angle_width = ComputeMaxAngleWidth(cloud);
  float max_angle_height = ComputeMaxAngleHeight(cloud);

  Eigen::Affine3f sensor_pose;
  range_image.createFromPointCloud (*cloud, angular_resolution_width, max_angle_width, max_angle_height, sensor_pose);
}
