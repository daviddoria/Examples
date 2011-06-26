#include "ImageCamera.h"

ImageCamera::ImageCamera()
{

  
}

vnl_double_2 ImageCamera::GetTopLeftPixel()
{
  vnl_double_2 pt2d;
  
  return pt2d;
}

vnl_double_2 ImageCamera::GetTopRightPixel()
{
  vnl_double_2 pt2d;
  
  return pt2d;
}

vnl_double_2 ImageCamera::GetBottomLeftPixel()
{
  vnl_double_2 pt2d;
  return pt2d;
}
    
vnl_double_2 ImageCamera::GetBottomRightPixel()
{
  
  vnl_double_2 pt2d;
  
  return pt2d;
}
  
void ImageCamera::DisplayParameters()
{

  
}

//*****************************************************************************

//this function gets a ray from the camera center through a pixel (pUV) specified in image (pixel) coordinates
bool ImageCamera::GetRay(vnl_double_2 pUV,
                         vnl_double_3& pXYZ) const
{

  return true;
}

//*****************************************************************************

bool ImageCamera::project(vnl_double_3 const& pXYZ,
                          vnl_double_2& pUV,
                          double* dist2) const
{
  // Project point into camera coordinate system.
 return true;
}

//*****************************************************************************

vnl_double_3 ImageCamera::get_origin() const
{
  return origin;
}

//*****************************************************************************


double ImageCamera::getDist2FromCenter(const vnl_double_2 &imgpt) const
{
return 0;
}