#include <vtkSmartPointer.h>
#include <vtkVector.h>

#include <iostream>

void AddVectors();

int main(int, char *[])
{
  vtkVector3d v(1.0, 2.0, 3.0);
  std::cout << v.X() << " " << v.Y() << " " << v.Z() << std::endl;
    
  double* vals = v.GetData();
  
  std::cout << vals[0] << " " << vals[1] << " " << vals[2] << std::endl;

  AddVectors();
  
  return EXIT_SUCCESS;
}

void AddVectors()
{
  vtkVector3d v1(1.0, 2.0, 3.0);
  vtkVector3d v2(4.0, 5.0, 6.0);
  
  // Cant do this!
  //vtkVector3d v3 = v1.Add(v2);
  
  
}