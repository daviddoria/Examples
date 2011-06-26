#include "vtkSmartPointer.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkPolyData.h"
#include "vtkSphereSource.h"

#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkInteractorStyleTrackball.h"

int main ()
{
  double multiplier = 0.1;
  for(unsigned int r = 0; r < 11; r++)
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
  
    // add the actors to the scene
    Renderer->AddActor(Actor);
    //Renderer->SetBackground(1,1,1); // Background color white
    double currentr = multiplier * r;
    vtkstd::cout << currentr << vtkstd::endl;
    //Renderer->SetBackground(currentr,1,1);
    Renderer->SetBackground(currentr, currentr, currentr);
  
    // render an image (lights and cameras are created automatically)
    RenderWindow->Render();
  
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New(); //like paraview
    
    RenderWindowInteractor->SetInteractorStyle( style );
    
    // begin mouse interaction
    RenderWindowInteractor->Start();
    
    //vtkstd::cout << "end of program" << vtkstd::endl;
    }  
  return 0;
}


