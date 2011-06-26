#include <vtkSmartPointer.h>
#include <vtkPlane.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPlane> plane = 
      vtkSmartPointer<vtkPlane>::New();
  plane->SetOrigin(0.0, 0.0, 0.0);
  plane->SetNormal(0.0, 0.0, 1.0);
  
  double p[3] = {23.1, 54.6, 9.2};
  double projected[3];
  plane->ProjectPoint(p, projected);
  
  cout << "Projected: " << projected[0] << " " << projected[1] << " " << projected[2] << endl;
  
  return EXIT_SUCCESS;
}