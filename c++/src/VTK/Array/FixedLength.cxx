#include <vtkSmartPointer.h>
#include <vtkFloatArray.h>

#include <iostream>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkFloatArray> Distances = vtkSmartPointer<vtkFloatArray>::New();
  Distances->SetNumberOfComponents(1);
  Distances->SetNumberOfValues(10);
  Distances->SetName("Distances");

  //set values
  for(unsigned int i = 0; i < Distances->GetNumberOfTuples(); i++)
  {	
    Distances->SetValue(i, drand48());
  }
  
  //get values
  for(unsigned int i = 0; i < Distances->GetNumberOfTuples(); i++)
  {
    double d = Distances->GetValue(i);
    vtkstd::cout << d << vtkstd::endl;
  }

  return 0;
}
