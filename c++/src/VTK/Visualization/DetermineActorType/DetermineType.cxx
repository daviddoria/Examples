#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkCubeAxesActor.h>
#include <vtkSphereSource.h>

int main(int argc, char *argv[])
{
  //sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());
  
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  
  //cube axes
  vtkSmartPointer<vtkCubeAxesActor> cubeAxesActor = 
      vtkSmartPointer<vtkCubeAxesActor>::New();
  
  // create a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  cubeAxesActor->SetCamera(renderer->GetActiveCamera());
  
  renderer->AddActor(actor);
  renderer->AddActor(cubeAxesActor);
  
  //Determine the types of the actors - method 1
  {
  cout << "Method 1:" << endl;
  vtkActorCollection* actorCollection = renderer->GetActors();
  actorCollection->InitTraversal();

  for(unsigned int i = 0; i < actorCollection->GetNumberOfItems(); i++)
    {
    vtkActor* actor = actorCollection->GetNextActor();
    cout << "actor " << i << " : " << actor->GetClassName() << endl;
    vtkstd::string className = actor->GetClassName();
    vtkstd::string wantedClass = "vtkCubeAxesActor";
    if(className.compare(wantedClass) == 0)
      {
      cout << "actor " << i << " is a vtkCubeAxesActor!" << endl;
      }
    else
      {
      cout << "actor " << i << " is NOT a vtkCubeAxesActor!" << endl;
      }
    }
  }
  
  //Determine the types of the actors - method 2
  {
  cout << "Method 2:" << endl;
  vtkActorCollection* actorCollection = renderer->GetActors();
  actorCollection->InitTraversal();

  for(unsigned int i = 0; i < actorCollection->GetNumberOfItems(); i++)
    {
    vtkActor* actor = actorCollection->GetNextActor();
    cout << "actor " << i << " : " << actor->GetClassName() << endl;
    if(actor->IsA("vtkCubeAxesActor"))
      {
      cout << "actor " << i << " is a vtkCubeAxesActor!" << endl;
      }
    else
      {
      cout << "actor " << i << " is NOT a vtkCubeAxesActor!" << endl;
      }
    }
  }
  
  //Determine the types of the actors - method 3
  {
  cout << "Method 3:" << endl;
  vtkActorCollection* actorCollection = renderer->GetActors();
  actorCollection->InitTraversal();

  for(unsigned int i = 0; i < actorCollection->GetNumberOfItems(); i++)
    {
    vtkActor* actor = actorCollection->GetNextActor();
    cout << "actor " << i << " : " << actor->GetClassName() << endl;

    if(vtkCubeAxesActor::SafeDownCast(actor) != 0)
      {
      cout << "actor " << i << " is a vtkCubeAxesActor!" << endl;
      }
    else
      {
      cout << "actor " << i << " is NOT a vtkCubeAxesActor!" << endl;
      }
    }
  }
  
  //Determine the types of the actors - method 4
  {
  cout << "Method 4:" << endl;
  vtkActorCollection* actorCollection = renderer->GetActors();
  actorCollection->InitTraversal();

  for(unsigned int i = 0; i < actorCollection->GetNumberOfItems(); i++)
    {
    vtkActor* actor = actorCollection->GetNextActor();
    cout << "actor " << i << " : " << actor->GetClassName() << endl;

    if(dynamic_cast<vtkCubeAxesActor*>(actor) != 0)
      {
      cout << "actor " << i << " is a vtkCubeAxesActor!" << endl;
      }
    else
      {
      cout << "actor " << i << " is NOT a vtkCubeAxesActor!" << endl;
      }
    }
  }
 
  //render the scene
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
