#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPropCollection.h>
#include <vtkInteractorStyleTrackballCamera.h>

int main(int argc, char *argv[])
{
  //sphere 1
  vtkSmartPointer<vtkSphereSource> sphereSource1 = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource1->SetCenter(0.0, 0.0, 0.0);
  sphereSource1->SetRadius(4.0);
  sphereSource1->Update();
    
  vtkSmartPointer<vtkPolyDataMapper> mapper1 = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper1->SetInputConnection(sphereSource1->GetOutputPort());
  
  vtkSmartPointer<vtkActor> actor1 = 
      vtkSmartPointer<vtkActor>::New();
  actor1->SetMapper(mapper1);
  
  //sphere 2
  vtkSmartPointer<vtkSphereSource> sphereSource2 = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource2->SetCenter(10.0, 0.0, 0.0);
  sphereSource2->SetRadius(3.0);
  sphereSource2->Update();
  
  //create a mapper
  vtkSmartPointer<vtkPolyDataMapper> mapper2 = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper2->SetInputConnection(sphereSource2->GetOutputPort());

  // create an actor
  vtkSmartPointer<vtkActor> actor2 = 
      vtkSmartPointer<vtkActor>::New();
  actor2->SetMapper(mapper2);

  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
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
  renderer->SetBackground(1,1,1); // Background color white

  // render an image (lights and cameras are created automatically)
  renderWindow->Render();

  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = 
      vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
  
  renderWindowInteractor->SetInteractorStyle( style );
  
  renderWindowInteractor->Start();
  
  //set the background to red so we know we are done with the original two sphere display
  renderer->SetBackground(1,0,0);
  
  actor2->VisibilityOff();
  renderWindow->Render();
    
  renderWindowInteractor->Start();
    
  return EXIT_SUCCESS;
}
