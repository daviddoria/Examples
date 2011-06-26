#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkObjectFactory.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkSphereWidget.h>
#include <vtkSphereRepresentation.h>
#include <vtkCommand.h>

class MySphereWidget : public vtkSphereWidget
{
 public:
    static MySphereWidget* New();
    vtkTypeRevisionMacro(MySphereWidget, vtkSphereWidget);
 
    void OnLeftButtonUp()
    {
      cout << "Left button up!" << endl;
      vtkSphereWidget::OnLeftButtonUp();
    }
};
vtkCxxRevisionMacro(MySphereWidget, "$Revision: 1.1 $");
vtkStandardNewMacro(MySphereWidget);

int main(int argc, char *argv[])
{
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
  
  vtkSmartPointer<MySphereWidget> sphereWidget = 
      vtkSmartPointer<MySphereWidget>::New();
  sphereWidget->SetInteractor(renderWindowInteractor);
  sphereWidget->SetRepresentationToSurface();

  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  sphereWidget->On();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
