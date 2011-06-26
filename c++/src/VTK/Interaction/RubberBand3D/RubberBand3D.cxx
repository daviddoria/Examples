#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkInteractorStyleRubberBand3D.h>
#include <vtkObjectFactory.h>

class MyRubberBand : public vtkInteractorStyleRubberBand3D
{
  public:
    static MyRubberBand* New();
    vtkTypeRevisionMacro(MyRubberBand, vtkInteractorStyleRubberBand3D);

    virtual void OnLeftButtonUp() 
    {
      //forward events
      vtkInteractorStyleRubberBand3D::OnLeftButtonUp();
 
      cout << "Start position: " << this->StartPosition[0] << " " << this->StartPosition[1] << endl;
      cout << "End position: " << this->EndPosition[0] << " " << this->EndPosition[1] << endl;
    }
 
};
vtkCxxRevisionMacro(MyRubberBand, "$Revision: 1.1 $");
vtkStandardNewMacro(MyRubberBand);

int main (int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  
  //create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
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

  vtkSmartPointer<MyRubberBand> style = 
      vtkSmartPointer<MyRubberBand>::New();
  renderWindowInteractor->SetInteractorStyle( style );
  
  // begin mouse interaction
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}