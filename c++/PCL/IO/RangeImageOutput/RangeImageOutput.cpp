// STL
#include <iostream>

// PCL
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/vtk_lib_io.h>

int 
main (int argc, char** argv)
{
  pcl::RangeImagePlanar::Ptr range_image(new pcl::RangeImagePlanar);
  const unsigned int side_length = 10;
  range_image->height = side_length;
  range_image->width = side_length;
  range_image->resize(range_image->height * range_image->width);

  for(unsigned int row = 0; row < side_length; ++row)
  {
    for(unsigned int col = 0; col < side_length; ++col)
    {
    pcl::PointWithRange p; p.x = drand48(); p.y = drand48(); p.z = drand48(); p.range = p.getVector3fMap().norm();
    std::cout << "Range " << p.range << std::endl;
    (*range_image)(col, row) = p;

    }
  }
  
  pcl::io::saveRangeImagePlanarFilePNG ("/home/doriad/test.png", *range_image);
  return (0);
}
