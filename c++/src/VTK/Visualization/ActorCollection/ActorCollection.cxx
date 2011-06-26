#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>

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
    
  vtkSmartPointer<vtkPolyDataMapper> mapper2 = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper2->SetInputConnection(sphereSource2->GetOutputPort());
    
  vtkSmartPointer<vtkActor> actor2 = 
      vtkSmartPointer<vtkActor>::New();
  actor2->SetMapper(mapper2);

  //create actor collection
  vtkSmartPointer<vtkActorCollection> actorCollection = 
      vtkSmartPointer<vtkActorCollection>::New();
  actorCollection->AddItem(actor1);
  actorCollection->AddItem(actor2);
  
  cout << "There are " << actorCollection->GetNumberOfItems() << " actors in the collection." << endl;
  
  // create a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  actorCollection->InitTraversal();
  // add the actors to the scene
  for(unsigned int i = 0; i < actorCollection->GetNumberOfItems(); i++)
    {
    vtkActor* actor = actorCollection->GetNextActor();
    renderer->AddActor(actor);
    }
 
  // render an image (lights and cameras are created automatically)
  renderWindow->Render();

  // begin mouse interaction
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
