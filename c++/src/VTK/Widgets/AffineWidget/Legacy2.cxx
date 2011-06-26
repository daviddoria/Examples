#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkCubeSource.h>
#include <vtkAffineWidget.h>
#include <vtkAffineRepresentation2D.h>
#include <vtkCommand.h>

class vtkAffineCallback : public vtkCommand
{
  public:
    static vtkAffineCallback *New()
    {
      return new vtkAffineCallback;
    }
    
    vtkAffineCallback(){}
    
    virtual void Execute(vtkObject *caller, unsigned long, void*)
    {
      
      vtkAffineWidget* affineWidget = 
          reinterpret_cast<vtkAffineWidget*>(caller);
      
      vtkAffineRepresentation2D* affineRepresentation = 
          static_cast<vtkAffineRepresentation2D*>(affineWidget->GetRepresentation());
      
      //box
      double width = affineRepresentation->GetBoxWidth();
      cout << "Width: " << width << endl;
      
      double origin[3];
      affineRepresentation->GetOrigin(origin);
      cout << "Origin: " << origin[0] << " " << origin[1] << " " << origin[2] << endl;
    }

};

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkCubeSource> cubeSource = 
      vtkSmartPointer<vtkCubeSource>::New();
  
  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(cubeSource->GetOutputPort());
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  
  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  renderer->AddActor(actor);
  
  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkSmartPointer<vtkAffineWidget> affineWidget = 
      vtkSmartPointer<vtkAffineWidget>::New();
  affineWidget->SetInteractor(renderWindowInteractor);
  
  vtkSmartPointer<vtkAffineCallback> affineCallback = 
      vtkSmartPointer<vtkAffineCallback>::New();
 
  affineWidget->AddObserver(vtkCommand::InteractionEvent,affineCallback);
    
  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  affineWidget->On();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
