#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPCAStatistics.h>

int main(int argc, char *argv[])
{
  //create a set of points
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint ( 1.0, 0.0, 0.0 );
  points->InsertNextPoint ( 0.0, 0.0, 0.0 );
  points->InsertNextPoint ( 0.0, 1.0, 0.0 );

  vtkSmartPointer<vtkPCAStatistics> pcaStatistics = 
      vtkSmartPointer<vtkPCAStatistics>::New();

  return EXIT_SUCCESS;
}
