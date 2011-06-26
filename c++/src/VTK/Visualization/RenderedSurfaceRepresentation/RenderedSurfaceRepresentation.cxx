#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkPolyData.h"
#include "vtkSphereSource.h"

int main ()
{

  vtkSphereSource* Sphere = vtkSphereSource::New();
  Sphere->SetCenter(0.0, 0.0, 0.0);
  Sphere->SetRadius(5.0);
  vtkPolyData* Polydata = Sphere->GetOutput();

  //create a mapper
  vtkPolyDataMapper *Mapper = vtkPolyDataMapper::New();
  Mapper->SetInput(Polydata);

  // create an actor
  vtkActor *Actor = vtkActor::New();
  Actor->SetMapper(Mapper);

  // a renderer and render window
  vtkRenderer* Renderer = vtkRenderer::New();
  vtkRenderWindow* RenderWindow = vtkRenderWindow::New();
  RenderWindow->AddRenderer(Renderer);

  // an interactor
  vtkRenderWindowInteractor *RenderWindowInteractor = vtkRenderWindowInteractor::New();
  RenderWindowInteractor->SetRenderWindow(RenderWindow);

  // add the actors to the scene
  Renderer->AddActor(Actor);
  Renderer->SetBackground(1,1,1); // Background color white

  // render an image (lights and cameras are created automatically)
  RenderWindow->Render();

  // begin mouse interaction
  RenderWindowInteractor->Start();
  
  //cleanup
  Sphere->Delete();
  Mapper->Delete();
  Actor->Delete();
  RenderWindow->Delete();
  Renderer->Delete();
  RenderWindowInteractor->Delete();

  return 0;
}


