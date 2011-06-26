#include <vtkSmartPointer.h>
#include <vtkPoints.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  vtkstd::cout << "Points class name: " << Points->GetClassName() << vtkstd::endl;  
  return 0;
}

