#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/normal_3d.h>

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

    // Setup the spin images computation
    typedef pcl::Histogram<153> SpinImage;
    pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> spinImageEstimation(8, 0.5, 16);
    spinImageEstimation.setInputWithNormals(cloud, normals);

    // Use the same KdTree from the normal estimation
    spinImageEstimation.setSearchMethod (tree);
    pcl::PointCloud<SpinImage>::Ptr spinImages(new pcl::PointCloud<SpinImage>);
    //spinImageEstimation.setRadiusSearch (0.2);
    spinImageEstimation.setKSearch(10);

    // Actually compute the spin images
    spinImageEstimation.compute (*spinImages);
    std::cout << "SI output points.size (): " << spinImages->points.size () << std::endl;

    // Display and retrieve the spin image descriptor vector for the first point.
    SpinImage descriptor = spinImages->points[0];
    std::cout << descriptor << std::endl;

    return 0;
}
