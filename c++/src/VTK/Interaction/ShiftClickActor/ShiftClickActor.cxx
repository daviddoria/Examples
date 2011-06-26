#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkObjectFactory.h>

class MouseInteractorStyle : public vtkInteractorStyleTrackballActor
{
  public:
    static MouseInteractorStyle* New();
 
    virtual void OnLeftButtonDown() 
    {
      if(this->Interactor->GetShiftKey())
        {
        cout << "Shift held. ";
        }
        
      if(this->Interactor->GetControlKey())
        {
        cout << "Control held. ";
        }
      
        /* this doesn't work ??? */
      if(this->Interactor->GetAltKey())
        {
        cout << "Alt held. ";
        }
        
      cout << "Pressed left mouse button." << endl;
      
      // forward events
      vtkInteractorStyleTrackballActor::OnLeftButtonDown();
    }
 
};
 
vtkStandardNewMacro(MouseInteractorStyle);
 
int main (int argc, char *argv[])
{

  vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
  
  //create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // add the actors to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(1,1,1); // Background color white

  // render an image (lights and cameras are created automatically)
  renderWindow->Render();

  vtkSmartPointer<MouseInteractorStyle> style = 
      vtkSmartPointer<MouseInteractorStyle>::New();
  
  renderWindowInteractor->SetInteractorStyle( style );
  
  // begin mouse interaction
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}