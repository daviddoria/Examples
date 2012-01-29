#include <iostream>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/Dense> // for Vector
#include <Eigen/StdVector> // Required (http://eigen.tuxfamily.org/dox-devel/TopicStlContainers.html)

typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Point2DVector;
typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > Point3DVector;

Point2DVector Generate2DPoints();
Point3DVector Generate3DPoints();

Eigen::MatrixXd ReadP(const std::string& filename);

int main(int argc, char *argv[])
{
  // Create the known projection matrix
  /*
  Eigen::MatrixXd = ReadP(argv[1]);

  Point2DVector Points2D = Generate2DPoints();
  Point3DVector Points3D = Generate3DPoints();
  
  for(unsigned int i = 0; i < Points3D.size(); ++i)
  {
    Eigen::Vector3d projected = P * Points3D[i].homogeneous();
    std::cout << "Projected: " << projected.hnormalized() << " actual: " << Points2D[i] << std::endl;
    //std::cout << "Projected: " << (P * Points3D[i].homogeneous()).hnormalized() << " actual: " << Points2D[i] << std::endl;
  }
*/
  return 0;
}

Eigen::MatrixXd ReadP(const std::string& filename)
{
  Eigen::MatrixXd P(3,4);
  P(0,0) = -2.8058e-01;
  P(1,0) = -6.8326e-02;
  P(2,0) = 5.1458e-07;

  P(0,1) = 2.0045e-02;
  P(1,1) = -3.1718e-01;
  P(2,1) = 4.5840e-06;

  P(0,2) = 1.8102e-01;
  P(1,2) = -7.2974e-02;
  P(2,2) = 2.6699e-06;

  P(0,3) = 6.6062e-01;
  P(1,3) = 5.8402e-01;
  P(2,3) = 1.5590e-03;
}

Point2DVector Generate2DPoints()
{
  Point2DVector points;

  float x,y;

  x=282;y=274;
  points.push_back(Eigen::Vector2d(x,y));

  x=397;y=227;
  points.push_back(Eigen::Vector2d(x,y));

  x=577;y=271;
  points.push_back(Eigen::Vector2d(x,y));

  x=462;y=318;
  points.push_back(Eigen::Vector2d(x,y));

  x=270;y=479;
  points.push_back(Eigen::Vector2d(x,y));

  x=450;y=523;
  points.push_back(Eigen::Vector2d(x,y));

  x=566;y=475;
  points.push_back(Eigen::Vector2d(x,y));

  return points;
}


Point3DVector Generate3DPoints()
{
  Point3DVector points;

  float x,y,z;

  x=.5;y=.5;z=-.5;
  points.push_back(Eigen::Vector3d(x,y,z));

  x=.5;y=.5;z=.5;
  points.push_back(Eigen::Vector3d(x,y,z));

  x=-.5;y=.5;z=.5;
  points.push_back(Eigen::Vector3d(x,y,z));

  x=-.5;y=.5;z=-.5;
  points.push_back(Eigen::Vector3d(x,y,z));

  x=.5;y=-.5;z=-.5;
  points.push_back(Eigen::Vector3d(x,y,z));

  x=-.5;y=-.5;z=-.5;
  points.push_back(Eigen::Vector3d(x,y,z));

  x=-.5;y=-.5;z=.5;
  points.push_back(Eigen::Vector3d(x,y,z));

  return points;
}
