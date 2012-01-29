#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>

int main (int argc, char** argv)
{
    std::string fileName = argv[1];
    std::cout << "Reading " << fileName << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile <pcl::PointXYZ> (fileName.c_str(), *cloud) == -1)
        // load the file
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

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud< pcl::Normal>);
    normalEstimation.setRadiusSearch (0.03);
    normalEstimation.compute (*normals);

    // Setup the SHOT features
    //typedef pcl::FPFHSignature33 ShotFeature; // Can't use this, even despite: http://docs.pointclouds.org/trunk/structpcl_1_1_f_p_f_h_signature33.html
    typedef pcl::SHOT ShotFeature;
    pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, ShotFeature> shotEstimation;
    shotEstimation.setInputCloud(cloud);
    shotEstimation.setInputNormals(normals);

    // Use the same KdTree from the normal estimation
    shotEstimation.setSearchMethod (tree);
    pcl::PointCloud<ShotFeature>::Ptr shotFeatures(new pcl::PointCloud<ShotFeature>);
    //spinImageEstimation.setRadiusSearch (0.2);
    shotEstimation.setKSearch(10);

    // Actually compute the spin images
    shotEstimation.compute (*shotFeatures);
    std::cout << "SHOT output points.size (): " << shotFeatures->points.size () << std::endl;

    // Display and retrieve the SHOT descriptor for the first point.
    ShotFeature descriptor = shotFeatures->points[0];
    std::cout << descriptor << std::endl;

    return 0;
}
