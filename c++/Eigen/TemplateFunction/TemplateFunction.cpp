#include <iostream>
#include <vector>

#include <Eigen/Dense> // for Vector

typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Point2DVector;

template<typename T>
T Centroid(const typename std::vector<T,Eigen::aligned_allocator<T> >& points)
{
  T centroid;
  centroid.fill(0.0f);

  for(unsigned int i = 0; i < points.size(); ++i)
    {
    centroid += points[i];
    }

  float numberOfPoints = static_cast<float>(points.size());
  centroid /= numberOfPoints;

  return centroid;
}

int main(int argc, char *argv[])
{

  Point2DVector points;

  Eigen::Vector2d centroid = Centroid(points);
  
  return 0;
}
