#include "EasyRenderer.h"

#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkRenderWindowInteractor.h"


void EasyRenderer::AddObject(vtkSmartPointer<vtkPolyData> Object)
{
  this->Objects.push_back(Object); 
}

void EasyRenderer::Render()
{
  // a renderer and render window
  vtkSmartPointer<vtkRenderer> Renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> RenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  RenderWindow->AddRenderer(Renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  RenderWindowInteractor->SetRenderWindow(RenderWindow);

  for(unsigned int i = 0; i < this->Objects.size(); i++)
  {
    //create a mapper
    vtkSmartPointer<vtkPolyDataMapper> Mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    Mapper->SetInput(this->Objects[i]);
  
    // create an actor
    vtkSmartPointer<vtkActor> Actor = vtkSmartPointer<vtkActor>::New();
    Actor->SetMapper(Mapper);
  
    // add the actors to the scene
    Renderer->AddActor(Actor);
  }
  
  Renderer->SetBackground(1,1,1); // Background color white

  // render an image (lights and cameras are created automatically)
  RenderWindow->Render();

  // begin mouse interaction
  RenderWindowInteractor->Start();

}