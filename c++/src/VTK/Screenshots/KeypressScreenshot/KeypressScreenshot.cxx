#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkSmartPointer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkPolyData.h"
#include "vtkSphereSource.h"

#include "KeyPressInteractor.h"

int main ()
{

  vtkSmartPointer<vtkSphereSource> Sphere = vtkSmartPointer<vtkSphereSource>::New();
  Sphere->SetCenter(0.0, 0.0, 0.0);
  Sphere->SetRadius(5.0);
  vtkPolyData* Polydata = Sphere->GetOutput();

  //create a mapper
  vtkSmartPointer<vtkPolyDataMapper> Mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  Mapper->SetInput(Polydata);

  // create an actor
  vtkSmartPointer<vtkActor> Actor = vtkSmartPointer<vtkActor>::New();
  Actor->SetMapper(Mapper);

  // a renderer and render window
  vtkSmartPointer<vtkRenderer> Renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> RenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  RenderWindow->AddRenderer(Renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  RenderWindowInteractor->SetRenderWindow(RenderWindow);

  // Create my interactor style
  vtkSmartPointer<KeyPressInteractorStyle> style = vtkSmartPointer<KeyPressInteractorStyle>::New();
  style->SetActor(Actor);
  //style->SetSource(Polydata);
  RenderWindowInteractor->SetInteractorStyle( style );
  
  // add the actors to the scene
  Renderer->AddActor(Actor);
  Renderer->SetBackground(1,1,1); // Background color white

  // render an image (lights and cameras are created automatically)
  RenderWindow->Render();
  

  
  // begin mouse interaction
  RenderWindowInteractor->Start();

  return 0;
}


