#include "vtkMath.h"

int main(int argc, char *argv[])
{
  double p0[3] = {0.0, 0.0, 0.0};
  //double p1[3] = {1.0, 1.0, 1.0};
  double p1[3] = {2.0, 0.0, 0.0};
  
  double squaredDistance = vtkMath::Distance2BetweenPoints(p0, p1);
  double distance = sqrt(squaredDistance);
  
  cout << "SquaredDistance = " << squaredDistance << endl;
  cout << "Distance = " << distance << endl;
  return EXIT_SUCCESS;
}
