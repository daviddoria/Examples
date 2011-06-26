#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkPolyData.h"
#include "vtkSmartPointer.h"
#include "vtkSphereSource.h"
#include "vtkBalloonWidget.h"

int main ()
{

  //sphere 1
  vtkSmartPointer<vtkSphereSource> SphereSource = vtkSmartPointer<vtkSphereSource>::New();
  SphereSource->SetCenter(0.0, 0.0, 0.0);
  SphereSource->SetRadius(4.0);
    
  vtkSmartPointer<vtkPolyDataMapper> Mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  Mapper->SetInput(SphereSource->GetOutput());

  vtkSmartPointer<vtkActor> Actor = vtkSmartPointer<vtkActor>::New();
  Actor->SetMapper(Mapper);
  
  // a renderer and render window
  vtkSmartPointer<vtkRenderer> Renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> RenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  RenderWindow->AddRenderer(Renderer);



  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  RenderWindowInteractor->SetRenderWindow(RenderWindow);

  vtkSmartPointer<vtkBalloonWidget> BalloonWidget = vtkSmartPointer<vtkBalloonWidget>::New();
  BalloonWidget->SetInteractor(RenderWindowInteractor);
  BalloonWidget->CreateDefaultRepresentation();
  BalloonWidget->AddBalloon(Actor,"This is a sphere",NULL);
  
  // add the actors to the scene
  Renderer->AddActor(Actor);
  Renderer->SetBackground(1,1,1); // Background color white

  // render an image (lights and cameras are created automatically)
  RenderWindow->Render();

  RenderWindowInteractor->Initialize();
  RenderWindow->Render();
  BalloonWidget->On();
  
  // begin mouse interaction
  RenderWindowInteractor->Start();
  
  return 0;
}
