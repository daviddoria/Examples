// STL
#include <iostream>

// PCL
#include <pcl/range_image/range_image.h>

// NOTE: operator() is (column, row) (clarified by the fix to issue #586)

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

namespace pcl
{

class RangeImageNew : public pcl::RangeImage
{
public:
  // Main new function
  void CreateFromPointCloud(CloudType::ConstPtr cloud);

private:
  // Workers
  float ComputeAngularResolutionWidth(CloudType::ConstPtr cloud);
  float ComputeAngularResolutionHeight(CloudType::ConstPtr cloud);
  float ComputeMaxAngleWidth(CloudType::ConstPtr cloud);
  float ComputeMaxAngleHeight(CloudType::ConstPtr cloud);

  // Helpers
  //float AngleBetweenPointVectors(const PointWithRange& p0, const PointWithRange& p1);
  //bool IsNan(const PointWithRange& p);
  float AngleBetweenPointVectors(const CloudType::PointType& p0, const CloudType::PointType& p1);
  bool IsNan(const CloudType::PointType& p);

  Eigen::Vector3f origin_;
};
}

// float pcl::RangeImageNew::AngleBetweenPointVectors(const PointWithRange& p0, const PointWithRange& p1)
// {
//   // Assuming origin is (0,0,0)
// //   Eigen::Vector4f v0;
// //   v0[0] = p0.x; v0[1] = p0.y; v0[2] = p0.z; v0[3] = 1;
// //   Eigen::Vector4f v1;
// //   v1[0] = p1.x; v1[1] = p1.y; v1[2] = p1.z; v1[3] = 1;
// //   return pcl::getAngle3D (v0, v1);
// 
//   Eigen::Vector4f v0;
//   v0[0] = p0.x - origin_[0]; v0[1] = p0.y - origin_[1]; v0[2] = p0.z - origin_[2]; v0[3] = 1;
//   Eigen::Vector4f v1;
//   v1[0] = p1.x - origin_[0]; v1[1] = p1.y - origin_[1]; v1[2] = p1.z - origin_[2]; v1[3] = 1;
//   return pcl::getAngle3D (v0, v1);
// }
// 
// bool pcl::RangeImageNew::IsNan(const PointWithRange& p)
// {
//   if(p.x == std::numeric_limits<float>::quiet_NaN() ||
//      p.y == std::numeric_limits<float>::quiet_NaN() ||
//      p.z == std::numeric_limits<float>::quiet_NaN())
//   {
//     return true;
//   }
// 
//   return false;
// }

float pcl::RangeImageNew::AngleBetweenPointVectors(const CloudType::PointType& p0, const CloudType::PointType& p1)
{
  // Assuming origin is (0,0,0)
//   Eigen::Vector4f v0;
//   v0[0] = p0.x; v0[1] = p0.y; v0[2] = p0.z; v0[3] = 1;
//   Eigen::Vector4f v1;
//   v1[0] = p1.x; v1[1] = p1.y; v1[2] = p1.z; v1[3] = 1;
//   return pcl::getAngle3D (v0, v1);

  Eigen::Vector4f v0;
  v0[0] = p0.x - origin_[0]; v0[1] = p0.y - origin_[1]; v0[2] = p0.z - origin_[2]; v0[3] = 1;
  Eigen::Vector4f v1;
  v1[0] = p1.x - origin_[0]; v1[1] = p1.y - origin_[1]; v1[2] = p1.z - origin_[2]; v1[3] = 1;
  return pcl::getAngle3D (v0, v1);
}

bool pcl::RangeImageNew::IsNan(const CloudType::PointType& p)
{
  if(p.x == std::numeric_limits<float>::quiet_NaN() ||
     p.y == std::numeric_limits<float>::quiet_NaN() ||
     p.z == std::numeric_limits<float>::quiet_NaN())
  {
    return true;
  }

  return false;
}

float pcl::RangeImageNew::ComputeAngularResolutionWidth(CloudType::ConstPtr cloud)
{
  float total_angle = 0.0f;
  unsigned int valid_angle_counter = 0;
  // Traverse the grid in pairs of a point and the point to the right of it.
  for(unsigned int col = 0; col < cloud->width - 1; ++col) // (width - 1) because there is no point to the right of the last column!
  {
    for(unsigned int row = 0; row < cloud->height; ++row) 
    {
      CloudType::PointType p0 = (*cloud)(col, row);
      CloudType::PointType p1 = (*cloud)(col + 1, row);
      if(IsNan(p0) || IsNan(p1))
        {
        continue;
        }
      else
      {
        total_angle = AngleBetweenPointVectors(p0, p1);
        valid_angle_counter++;
      }
    }
  }
  float average_angle = total_angle/static_cast<float>(valid_angle_counter);
  return average_angle;
}

float pcl::RangeImageNew::ComputeAngularResolutionHeight(CloudType::ConstPtr cloud)
{
  float total_angle = 0.0f;
  unsigned int valid_angle_counter = 0;
  // Traverse the grid in pairs of a point and the point above it.
  for(unsigned int col = 0; col < cloud->width; ++col)
  {
    for(unsigned int row = 0; row < cloud->height - 1; ++row) // (height -1) because there is no point above the top row!
    {
      CloudType::PointType p0 = (*cloud)(col, row);
      CloudType::PointType p1 = (*cloud)(col, row + 1);
      if(IsNan(p0) || IsNan(p1))
        {
        continue;
        }
      else
      {
        total_angle = AngleBetweenPointVectors(p0, p1);
        valid_angle_counter++;
      }
    }
  }
  float average_angle = total_angle/static_cast<float>(valid_angle_counter);
  return average_angle;
}

float pcl::RangeImageNew::ComputeMaxAngleWidth(CloudType::ConstPtr cloud)
{
  // Compute the average of the angle between extreme points
  float total_angle = 0.0f;
  unsigned int valid_angle_counter = 0;
  for(unsigned int row = 0; row < cloud->height; ++row)
  {
    CloudType::PointType p0 = (*cloud)(0, row);
    CloudType::PointType p1 = (*cloud)(cloud->width - 1, row);
    if(IsNan(p0) || IsNan(p1))
      {
      continue;
      }
    else
    {
      total_angle = AngleBetweenPointVectors(p0, p1);
      valid_angle_counter++;
    }
  }
  float average_angle = total_angle/static_cast<float>(valid_angle_counter);
  return average_angle;
}

float pcl::RangeImageNew::ComputeMaxAngleHeight(CloudType::ConstPtr cloud)
{
  // Compute the average of the angle between extreme points
  float total_angle = 0.0f;
  unsigned int valid_angle_counter = 0;
  for(unsigned int col = 0; col < cloud->width; ++col)
  {
    CloudType::PointType p0 = (*cloud)(col, 0);
    CloudType::PointType p1 = (*cloud)(col, cloud->height - 1);
    if(IsNan(p0) || IsNan(p1))
      {
      continue;
      }
    else
    {
      total_angle = AngleBetweenPointVectors(p0, p1);
      valid_angle_counter++;
    }
  }
  float average_angle = total_angle/static_cast<float>(valid_angle_counter);
  return average_angle;
}

void pcl::RangeImageNew::CreateFromPointCloud(CloudType::ConstPtr cloud)
{
  Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();
  Eigen::Vector3f zero;
  zero[0] = 0; zero[1] = 0; zero[2] = 0;
  origin_ = sensor_pose * zero;

  float angular_resolution_width = ComputeAngularResolutionWidth(cloud);
  std::cout << "angular_resolution_width: " << angular_resolution_width << std::endl;
  //float angular_resolution_height = ComputeAngularResolutionHeight(cloud);
  float max_angle_width = ComputeMaxAngleWidth(cloud);
  std::cout << "max_angle_width: " << max_angle_width << std::endl;
  float max_angle_height = ComputeMaxAngleHeight(cloud);
  std::cout << "max_angle_height: " << max_angle_height << std::endl;

  createFromPointCloud (*cloud, angular_resolution_width, max_angle_width, max_angle_height, sensor_pose);
}


int main (int argc, char** argv)
{
  // Create an organized point cloud
  CloudType::Ptr cloud (new CloudType);
  cloud->height = 10;
  cloud->width = 10;
  cloud->is_dense = true;
  cloud->resize(cloud->height * cloud->width);

  // Make a scan of a square
  for(unsigned int row = 0; row < cloud->height; ++row)
  {
    for(unsigned int col = 0; col < cloud->width; ++col)
    {
      PointType p;
      p.x = col; p.y = row; p.z = 10;
      (*cloud)(col, row) = p;
    }
  }

  pcl::RangeImageNew range_image;

  range_image.CreateFromPointCloud(cloud);
  return 0;
}
