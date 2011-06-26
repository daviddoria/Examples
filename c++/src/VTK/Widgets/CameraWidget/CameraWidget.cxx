#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCameraRepresentation.h>
#include <vtkCameraWidget.h>
 
int main (int argc, char *argv[])
{
  // A sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput(sphereSource->GetOutput());
 
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  // A renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
 
  // An interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  // Add the actors to the scene
  renderer->AddActor(actor);
 
  // Render an image (lights and cameras are created automatically)
  renderWindow->Render();
  
  vtkSmartPointer<vtkCameraRepresentation> cameraRep = 
      vtkSmartPointer<vtkCameraRepresentation>::New();
  cameraRep->SetNumberOfFrames(2400);
  
  vtkSmartPointer<vtkCameraWidget> cameraWidget = 
      vtkSmartPointer<vtkCameraWidget>::New();
  cameraWidget->SetInteractor(renderWindowInteractor);
  cameraWidget->SetRepresentation(cameraRep);

  //cameraWidget->EnabledOn();
  cameraWidget->On();
 
  renderWindow->Render();
 
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}