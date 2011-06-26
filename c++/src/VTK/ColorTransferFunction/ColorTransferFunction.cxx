#include <vtkSmartPointer.h>
#include <vtkColorTransferFunction.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkColorTransferFunction> colorTransferFunction = 
      vtkSmartPointer<vtkColorTransferFunction>::New();
  
  colorTransferFunction->AddRGBPoint(0.0, 1, 0, 0);
  colorTransferFunction->AddRGBPoint(10.0, 0, 1, 0);
  
  double color[3];
  colorTransferFunction->GetColor(1.0, color);
  cout << color[0] << " " << color[1] << " " << color[2] << endl;
  
  colorTransferFunction->GetColor(5.0, color);
  cout << color[0] << " " << color[1] << " " << color[2] << endl;
  
  return EXIT_SUCCESS;
}
