#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkPointPicker.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
  
// Define interaction style
class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
     
    static MouseInteractorStyle* New();
    vtkTypeRevisionMacro(MouseInteractorStyle, vtkInteractorStyleTrackballCamera);
 
    MouseInteractorStyle() : NumberOfClicks(0), ResetPixelDistance(5) 
    { 
      this->PreviousPosition[0] = 0;
      this->PreviousPosition[1] = 0;
    }
    
    virtual void OnLeftButtonDown() 
    {
      //cout << "Pressed left mouse button." << endl;
      this->NumberOfClicks++;
      //cout << "NumberOfClicks = " << this->NumberOfClicks << endl;
      int pickPosition[2];
      this->GetInteractor()->GetEventPosition(pickPosition);

      int xdist = pickPosition[0] - this->PreviousPosition[0];
      int ydist = pickPosition[1] - this->PreviousPosition[1];

      this->PreviousPosition[0] = pickPosition[0];
      this->PreviousPosition[1] = pickPosition[1];

      int moveDistance = (int)sqrt((double)(xdist*xdist + ydist*ydist));

      // Reset numClicks - If mouse moved further than resetPixelDistance
      if(moveDistance > this->ResetPixelDistance)
        { 
        this->NumberOfClicks = 1;
        }
        
      
      if(this->NumberOfClicks == 2)
        {
        cout << "Double clicked." << endl;
        this->NumberOfClicks = 0;
        }
      // forward events
      vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }
 
  private:
    unsigned int NumberOfClicks;
    int PreviousPosition[2];
    int ResetPixelDistance;
};
 vtkCxxRevisionMacro(MouseInteractorStyle, "$Revision: 1.1 $");
vtkStandardNewMacro(MouseInteractorStyle);
 
int main ( int argc, char* argv[] )
{
 
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(0.0, 0.0, 0.0);
  sphereSource->SetRadius(5.0);
  sphereSource->Update();
 
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());
 
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderer->SetBackground(1,1,1); // Background color white
  renderer->AddActor(actor);
 
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
 
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow ( renderWindow );
 
  vtkSmartPointer<MouseInteractorStyle> style = vtkSmartPointer<MouseInteractorStyle>::New();
  renderWindowInteractor->SetInteractorStyle( style );
 
  renderWindowInteractor->Initialize();
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}