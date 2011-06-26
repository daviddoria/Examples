#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAxisActor2D.h>
#include <vtkSmartPointer.h>

int main (int argc, char *argv[])
{
  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  //renderer->SetBackground(1,1,1);
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
    
  // add the axis
  vtkSmartPointer<vtkAxisActor2D> axisActor = 
      vtkSmartPointer<vtkAxisActor2D>::New();
  axisActor->SetPoint2(.75, .75);
  renderer->AddActor(axisActor);
  
  //render and interact
  renderer->ResetCamera();
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}