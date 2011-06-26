#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkInteractorStyleTerrain.h>
#include <vtkObjectFactory.h>

class MyInteractorStyle : public vtkInteractorStyleTerrain
{
  public:
    static MyInteractorStyle* New();
 
    virtual void OnMouseWheelForward() 
    {
      cout << "Wheel forward" << endl;
 
        vtkRenderWindowInteractor *rwi = this->Interactor;
        
      this->FindPokedRenderer(rwi->GetEventPosition()[0],
                              rwi->GetEventPosition()[1]);
      this->CreateLatLong();
      if (this->LatLongLines) 
        {
        this->LatLongLinesOff();
        }
      else 
        {
        double bounds[6];
        this->CurrentRenderer->ComputeVisiblePropBounds( bounds );
        double radius = sqrt((bounds[1]-bounds[0])*(bounds[1]-bounds[0]) +
                             (bounds[3]-bounds[2])*(bounds[3]-bounds[2]) +
                             (bounds[5]-bounds[4])*(bounds[5]-bounds[4])) /2.0;
        this->LatLongSphere->SetRadius( radius );
        this->LatLongSphere->SetCenter((bounds[0]+bounds[1])/2.0,
                                       (bounds[2]+bounds[3])/2.0,
                                       (bounds[4]+bounds[5])/2.0);        
        this->LatLongLinesOn();
        }
      this->SelectRepresentation();
      rwi->Render();
      
      // forward events
      vtkInteractorStyleTerrain::OnMouseWheelForward();
    }
    
    virtual void OnMouseWheelBackward() 
    {
      cout << "Wheel backward" << endl;
 
      // forward events
      vtkInteractorStyleTerrain::OnMouseWheelBackward();
    }
 
};
vtkStandardNewMacro(MyInteractorStyle);

int main (int argc, char *argv[])
{

  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  //create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

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
  renderer->AddActor(actor);
  renderer->SetBackground(1,1,1); // Background color white
  
  renderWindow->Render();

 // vtkSmartPointer<vtkInteractorStyleTerrain> style = 
   //   vtkSmartPointer<vtkInteractorStyleTerrain>::New();
  
  vtkSmartPointer<MyInteractorStyle> style = 
    vtkSmartPointer<MyInteractorStyle>::New();
 
  renderWindowInteractor->SetInteractorStyle( style );
  
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}