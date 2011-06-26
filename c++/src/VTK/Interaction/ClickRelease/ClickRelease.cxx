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
    vtkTypeRevisionMacro(MouseInteractorStyle,vtkInteractorStyleTrackballCamera);
    
    virtual void OnLeftButtonUp() 
    {
      std::cout << "Left button up." << std::endl;
      // forward events
      vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
    }
 
    virtual void OnMiddleButtonUp() 
    {
      std::cout << "Middle button up." << std::endl;
      // forward events
      vtkInteractorStyleTrackballCamera::OnMiddleButtonUp();
    }
 
    virtual void OnRightButtonUp() 
    {
      std::cout << "Right button up." << std::endl;
      // forward events
      vtkInteractorStyleTrackballCamera::OnRightButtonUp();
    }
 
};
vtkCxxRevisionMacro(MouseInteractorStyle, "$Revision: 1.1 $");
vtkStandardNewMacro(MouseInteractorStyle);
 
int main(int, char *[])
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
 
  vtkSmartPointer<MouseInteractorStyle> style = 
      vtkSmartPointer<MouseInteractorStyle>::New();
  renderWindowInteractor->SetInteractorStyle( style );
 
  renderWindowInteractor->Initialize();
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}