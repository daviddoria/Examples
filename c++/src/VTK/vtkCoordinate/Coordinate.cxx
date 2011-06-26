#include <vtkSmartPointer.h>
#include <vtkCoordinate.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkRenderWindow> rendererWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  
  vtkSmartPointer<vtkRenderer> renderer =
      vtkSmartPointer<vtkRenderer>::New();
  rendererWindow->AddRenderer(renderer);
  rendererWindow->Render();
  
  vtkSmartPointer<vtkCoordinate> coordinate = 
      vtkSmartPointer<vtkCoordinate>::New();
  coordinate->SetCoordinateSystemToNormalizedDisplay();
  coordinate->SetValue(.5,.5,0);
  cout << *coordinate << endl;
  cout << coordinate->GetCoordinateSystemAsString() << endl;
  
  int* val = new int[3];
  val = coordinate->GetComputedDisplayValue(renderer);
  cout << "Val: " << val[0] << " " << val[1] << endl;
  
  return EXIT_SUCCESS;
}
