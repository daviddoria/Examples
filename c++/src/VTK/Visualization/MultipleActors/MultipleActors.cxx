#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkPolyData.h"
#include "vtkSmartPointer.h"
#include "vtkSphereSource.h"

#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkInteractorStyleTrackball.h"

int main ()
{

  //sphere 1
  vtkSmartPointer<vtkSphereSource> Sphere1 = vtkSmartPointer<vtkSphereSource>::New();
  Sphere1->SetCenter(0.0, 0.0, 0.0);
  Sphere1->SetRadius(4.0);
    
  vtkSmartPointer<vtkPolyDataMapper> Mapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
  Mapper1->SetInput(Sphere1->GetOutput());

  
  vtkSmartPointer<vtkActor> Actor1 = vtkSmartPointer<vtkActor>::New();
  Actor1->SetMapper(Mapper1);
  
  //sphere 2
  vtkSmartPointer<vtkSphereSource> Sphere2 = vtkSmartPointer<vtkSphereSource>::New();
  Sphere2->SetCenter(10.0, 0.0, 0.0);
  Sphere2->SetRadius(3.0);
  
  //create a mapper
  vtkSmartPointer<vtkPolyDataMapper> Mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
  Mapper2->SetInput(Sphere2->GetOutput());

  // create an actor
  vtkSmartPointer<vtkActor> Actor2 = vtkSmartPointer<vtkActor>::New();
  Actor2->SetMapper(Mapper2);

  // a renderer and render window
  vtkSmartPointer<vtkRenderer> Renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> RenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  RenderWindow->AddRenderer(Renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  RenderWindowInteractor->SetRenderWindow(RenderWindow);

  // add the actors to the scene
  Renderer->AddActor(Actor1);
  Renderer->AddActor(Actor2);
  Renderer->SetBackground(1,1,1); // Background color white

  // render an image (lights and cameras are created automatically)
  RenderWindow->Render();

  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New(); //like paraview
  
  RenderWindowInteractor->SetInteractorStyle( style );
  
  // begin mouse interaction
  RenderWindowInteractor->Start();
  
  return 0;
}
