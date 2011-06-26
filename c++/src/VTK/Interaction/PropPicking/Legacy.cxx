#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPlaneSource.h>
#include <vtkPropPicker.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkProperty.h>
#include <vtkEvent.h>

// Trap mouse events here and do some stuff.
class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static MouseInteractorStyle* New()
    {
      return new MouseInteractorStyle();
    }

    virtual void OnLeftButtonDown()
    {
      cout << "Pressed left mouse button." << endl;

      int* pos = this->GetInteractor()->GetEventPosition();

      // Pick from this location.
      this->Picker->Pick(pos[0], pos[1], 0, this->GetDefaultRenderer());

      // forward events
      vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }
    
    void SetPicker(vtkSmartPointer<vtkPropPicker> picker) {this->Picker = picker;}
  private:
    vtkSmartPointer<vtkPropPicker>  Picker;

};

// Callback thats get called for a picking event.
class PickCallback : public vtkCommand
{
  public:
    static PickCallback* New()
    {
      return new PickCallback();
    }

    void Execute(vtkObject* caller, unsigned long eventId, void* callData)
    {
      vtkPropPicker* picker = static_cast<vtkPropPicker*>(caller);
      double* pos = picker->GetPickPosition();
      cout << "Pick position is: " << pos[0] << " " << pos[1]
          << " " << pos[2] << endl;
    }

};


// Execute application. 
int main (int argc, char *argv[])
{

  vtkSmartPointer<vtkPlaneSource> planeSource = 
      vtkSmartPointer<vtkPlaneSource>::New();
  planeSource->Update();

  //create a polydata object
  vtkPolyData* polydata = planeSource->GetOutput();

  //create a mapper
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput ( polydata );

  // create an actor
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper ( mapper );

  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer ( renderer );

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow ( renderWindow );
  renderWindowInteractor->Initialize();

  // create a callback that gets called for a pick event. 
  PickCallback* pickCallback = PickCallback::New();
  vtkSmartPointer<vtkPropPicker> picker = 
      vtkSmartPointer<vtkPropPicker>::New();
  picker->AddObserver(vtkCommand::EndPickEvent,  pickCallback);
  
  // set the cell picker to use. 
  renderWindowInteractor->SetPicker(picker);

  // set the custom stype to use for interaction. 
  vtkSmartPointer<MouseInteractorStyle> style =
      vtkSmartPointer<MouseInteractorStyle>::New();
  style->SetDefaultRenderer(renderer);
  style->SetPicker(picker);
  
  renderWindowInteractor->SetInteractorStyle( style );
  
  // add the actors to the scene
  renderer->AddActor ( actor );
  //renderer->SetBackground ( 1,1,1 ); // Background color white
  renderer->SetBackground ( 0,0,1 );

  // render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
