#include <vtkObjectFactory.h>

#include <vtkPolyDataMapper.h>
#include <vtkTransform.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkConeSource.h>
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballActor.h>

// Define interaction style
class MouseInteractorStyle : public vtkInteractorStyleTrackballActor
{
  public:
    static MouseInteractorStyle* New();
 
    void OnLeftButtonUp()
    {
      vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
      if(this->InteractionProp)
        {
        this->InteractionProp->GetMatrix(m);
        cout << *m << endl;
        }
      
      // forward events
      vtkInteractorStyleTrackballActor::OnLeftButtonUp();
    }

    void OnMiddleButtonUp()
    {
      vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
      if(this->InteractionProp)
      {
        this->InteractionProp->GetMatrix(m);
        cout << *m << endl;
      }
      
      // forward events
      vtkInteractorStyleTrackballActor::OnMiddleButtonUp();
    }
};
vtkStandardNewMacro(MouseInteractorStyle);

int main (int argc, char *argv[])
{
  
  vtkSmartPointer<vtkConeSource> coneSource = vtkSmartPointer<vtkConeSource>::New();
  vtkPolyData* cone = coneSource->GetOutput();

  //create a mapper
  vtkSmartPointer<vtkPolyDataMapper> coneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  coneMapper->SetInput(cone);

  // create an actor
  vtkSmartPointer<vtkActor> coneActor = vtkSmartPointer<vtkActor>::New();
  coneActor->SetMapper(coneMapper);

  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // add the actors to the scene
  renderer->AddActor(coneActor);
  renderer->SetBackground(1,1,1); // Background color white
  
  renderer->ResetCamera();
  renderWindow->Render();
  
  /*
  vtkSmartPointer<vtkInteractorStyleTrackballActor> style = 
      vtkSmartPointer<vtkInteractorStyleTrackballActor>::New();
 */
  
  vtkSmartPointer<MouseInteractorStyle> style = 
      vtkSmartPointer<MouseInteractorStyle>::New();
 
  renderWindowInteractor->SetInteractorStyle( style );

  // begin mouse interaction
  renderWindowInteractor->Start();


  
  return 0;
}