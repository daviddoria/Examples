#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/normal_3d.h>

main (int argc, char** argv)
{
  std::string fileName = argv[1];
  std::cout << "Reading " << fileName << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.03);

  normalEstimation.compute (*cloudWithNormals);

  // Setup the shape context computation
  typedef pcl::SHOT ShapeContext; // Signature of Histograms of OrienTations (SHOT).
  pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, ShapeContext > shapeContext;
  // Provide the original point cloud (without normals)
  shapeContext.setInputCloud (cloud);
  // Provide the point cloud with normals
  shapeContext.setInputNormals(cloudWithNormals);
  // Use the same KdTree from the normal estimation
  shapeContext.setSearchMethod (tree);

  pcl::PointCloud<ShapeContext>::Ptr shapeContextFeatures(new pcl::PointCloud<ShapeContext>);

  // The search radius must be set to above the minimal search radius
  std::cout << "min radius: " << shapeContext.getMinimalRadius() << std::endl;
  shapeContext.setRadiusSearch (0.2);

  // Actually compute the shape contexts
  shapeContext.compute (*shapeContextFeatures);

  std::cout << "output points.size (): " << shapeContextFeatures->points.size () << std::endl;

  // Display and retrieve the shape context descriptor vector for the 0th point.
  std::cout << shapeContextFeatures->points[0] << std::endl;
  std::vector<float> descriptor = shapeContextFeatures->points[0].descriptor;

  return 0;
}
