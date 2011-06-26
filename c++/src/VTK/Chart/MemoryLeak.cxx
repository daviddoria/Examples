#include "vtkContextView.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkContextView> view = vtkSmartPointer<vtkContextView>::New();
  view->Render();
  view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
  view->GetRenderWindow()->SetSize(800, 600);
  view->Render();
  view->GetRenderWindow()->SetMultiSamples(0);  
  return 0;
}
