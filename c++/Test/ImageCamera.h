#ifndef IMAGECAMERA_H_
#define IMAGECAMERA_H_

#include "Camera.h"
#include <vnl/vnl_double_3.h>
#include <vector>

class ImageCamera : public Camera
{
  public:
    ImageCamera();
    ~ImageCamera() {}

    bool GetRay(vnl_double_2 pUV, vnl_double_3& pXYZ) const;
    bool project(vnl_double_3 const& pXYZ, vnl_double_2& pUV, double* dist2) const;
    vnl_double_3 get_origin() const;
    double getDist2FromCenter(const vnl_double_2 &imgpt) const;
  
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    vnl_double_2 getImageCenter() const { return vnl_double_2(u0, v0); }
  
    vnl_double_3 get_axis() const {return R.get_column(2);}

    std::vector<std::string> GetFileList(const std::string &directory, const std::string &ext);
    std::vector<std::string> GetImageFileList(const std::string &directory);
  
    virtual vnl_double_2 GetTopLeftPixel();
    virtual vnl_double_2 GetTopRightPixel();
    virtual vnl_double_2 GetBottomLeftPixel();
    virtual vnl_double_2 GetBottomRightPixel();
  
    virtual void DisplayParameters();
  
  protected:

    int rows, columns;   
    double f;
    int width, height;
    double u0, v0; //image center
    
    std::string ImageFileName;
    
};

#endif
