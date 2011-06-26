#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

#include "vtkTestReader.h"

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkTestReader> reader = 
    vtkSmartPointer<vtkTestReader>::New();
  reader->Update();
  
  vtkPolyData* polydata = reader->GetOutput();

  std::cout << "Number of points: " << polydata->GetNumberOfPoints() << std::endl;
  
  return EXIT_SUCCESS;
}
