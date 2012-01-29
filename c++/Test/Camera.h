#ifndef CAMERA_H_
#define CAMERA_H_

#include <vnl/vnl_matrix_fixed.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_double_2.h>

class Camera
{
  public:

    Camera() {}

    virtual bool read_parameters() = 0;
    virtual bool get_xyz(vnl_double_2 pUV, vnl_double_3& pXYZ) const = 0;
    virtual bool project(vnl_double_3 const& pXYZ, vnl_double_2& pUV, double* dist2) const = 0;
    virtual vnl_double_3 get_origin() const = 0;
    virtual double getDist2FromCenter(const vnl_double_2 &imgpt) const = 0;
    virtual vnl_double_3 get_axis() const = 0;
    vnl_matrix_fixed<double, 3, 3> get_R() const { return R; }
    double getFocalLength() const { return f; }
  
  protected:

    vnl_matrix_fixed<double, 3, 3> R;
    vnl_double_3 t, origin;     //origin is equal to -R^T * t
    double f; //focal length
};
#endif
