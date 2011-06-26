#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkSphereWidget.h>
#include <vtkSphereRepresentation.h>
#include <vtkBoxWidget2.h>
#include <vtkCommand.h>

int main(int argc, char *argv[])
{
  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  
  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkSmartPointer<vtkSphereWidget> sphereWidget1 = 
      vtkSmartPointer<vtkSphereWidget>::New();
  sphereWidget1->SetInteractor(renderWindowInteractor);
  sphereWidget1->SetRepresentationToSurface();
  
   vtkSmartPointer<vtkSphereWidget> sphereWidget2 = 
      vtkSmartPointer<vtkSphereWidget>::New();
  sphereWidget2->SetInteractor(renderWindowInteractor);
  sphereWidget2->SetRepresentationToSurface();
  sphereWidget2->SetCenter(2.0, 0.0, 0.0);
 
  sphereWidget1->On();
  sphereWidget2->On();
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
