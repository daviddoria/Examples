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

#include <vtkstd/vector>

int main ()
{
  vtkstd::vector<vtkSmartPointer<vtkActor> > actors;
  
  for(unsigned int i = 0; i < 10; i++)
    {
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetCenter(i, 0.0, 0.0);
    sphereSource->SetRadius(.2);
    
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInput(sphereSource->GetOutput());
  
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
  
    actors.push_back(actor);
    }
  

  // a renderer and render window
  vtkSmartPointer<vtkRenderer> Renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> RenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  RenderWindow->AddRenderer(Renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  RenderWindowInteractor->SetRenderWindow(RenderWindow);

  // add the actors to the scene
  for(unsigned int i = 0; i < actors.size(); i++)
    {
    Renderer->AddActor(actors[i]);
    }
    
  Renderer->SetBackground(1,1,1); // Background color white

  // render an image (lights and cameras are created automatically)
  RenderWindow->Render();

  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New(); //like paraview
  
  RenderWindowInteractor->SetInteractorStyle( style );
  
  // begin mouse interaction
  RenderWindowInteractor->Start();
  
  return 0;
}
