#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkScalarBarActor.h>

int main (int argc, char *argv[])
{

  //sphere 1
  vtkSmartPointer<vtkSphereSource> sphere1 = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphere1->SetCenter(0.0, 0.0, 0.0);
  sphere1->SetRadius(4.0);
  sphere1->Update();
    
  vtkSmartPointer<vtkPolyDataMapper> mapper1 = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper1->SetInput(sphere1->GetOutput());

  vtkSmartPointer<vtkActor> actor1 = 
      vtkSmartPointer<vtkActor>::New();
  actor1->SetMapper(mapper1);
  
  //sphere 2
  vtkSmartPointer<vtkSphereSource> sphere2 = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphere2->SetCenter(10.0, 0.0, 0.0);
  sphere2->SetRadius(3.0);
  sphere2->Update();
  
  //create a mapper
  vtkSmartPointer<vtkPolyDataMapper> mapper2 = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper2->SetInput(sphere2->GetOutput());

  // create an actor
  vtkSmartPointer<vtkActor> actor2 = 
      vtkSmartPointer<vtkActor>::New();
  actor2->SetMapper(mapper2);

  vtkSmartPointer<vtkScalarBarActor> scalarBar = 
      vtkSmartPointer<vtkScalarBarActor>::New();
  scalarBar->SetLookupTable(mapper1->GetLookupTable());
  scalarBar->SetTitle("Title");
  
  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor2D(scalarBar);
  
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // add the actors to the scene
  renderer->AddActor(actor1);
  renderer->AddActor(actor2);

  // render an image (lights and cameras are created automatically)
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
