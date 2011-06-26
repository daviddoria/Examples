#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

int main(int, char *[])
{
  //Create a point cloud
  vtkSmartPointer<vtkPointSource> pointSource =
    vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetCenter(0.0, 0.0, 0.0);
  pointSource->SetNumberOfPoints(50);
  pointSource->SetRadius(5.0);
  pointSource->Update();

  vtkPoints* points = pointSource->GetOutput()->GetPoints();

  double p[3];
  points->GetPoint(0,p);

  std::cout << "p: " << p[0] << " " << p[1] << " " << p[2] << std::endl;

  double* point = points->GetPoint(0);
  std::cout << "point: " << point[0] << " " << point[1] << " " << point[2] << std::endl;

  point[0] = 2;

  std::cout << "point: " << point[0] << " " << point[1] << " " << point[2] << std::endl;


  points->SetPoint(0, point);

  double a[3];
  points->GetPoint(0,a);
  std::cout << "a: " << a[0] << " " << a[1] << " " << a[2] << std::endl;

  return EXIT_SUCCESS;
}