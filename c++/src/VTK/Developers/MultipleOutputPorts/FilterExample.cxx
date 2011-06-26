#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>

#include "vtkTestFilter.h"

int main (int, char *[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource =
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();

  vtkSmartPointer<vtkTestFilter> filter =
      vtkSmartPointer<vtkTestFilter>::New();
  filter->SetInputConnection(sphereSource->GetOutputPort());
  filter->Update();

  std::cout << "Output0 has " << filter->GetOutput(0)->GetNumberOfPoints() << " points." << std::endl;
  std::cout << "Output1 has " << filter->GetOutput(1)->GetNumberOfPoints() << " points." << std::endl;

  return EXIT_SUCCESS;
}
