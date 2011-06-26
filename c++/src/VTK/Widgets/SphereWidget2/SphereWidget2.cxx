#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkSphereWidget2.h>
#include <vtkSphereRepresentation.h>
#include <vtkCommand.h>


// This does the actual work.
// Callback for the interaction
class vtkSphereCallback : public vtkCommand
{
  public:
    static vtkSphereCallback *New()
    {
      return new vtkSphereCallback;
    }
    
    virtual void Execute(vtkObject *caller, unsigned long, void*)
    {
      
      vtkSphereWidget2 *sphereWidget = 
          reinterpret_cast<vtkSphereWidget2*>(caller);
      
      /*
      //get the actual box coordinates/planes
      vtkSmartPointer<vtkPolyData> polydata = 
          vtkSmartPointer<vtkPolyData>::New();
      static_cast<vtkSphereRepresentation*>(sphereWidget->GetRepresentation())->GetPolyData (polydata);
      
      //display one of the points, just so we know it's working
      double p[3];
      polydata->GetPoint(0,p);
      cout << "P: " << p[0] << " " << p[1] << " " << p[2] << endl;
      */
      
      double handlePos[3];
      vtkSphereRepresentation* sphereRepresentation = 
          static_cast<vtkSphereRepresentation*>(sphereWidget->GetRepresentation());
      sphereRepresentation->GetHandlePosition (handlePos);
      cout << "Handle pos: " << handlePos[0] << " " << handlePos[1] << " " << handlePos[2] << endl;
      sphereRepresentation->RadialLineOff ();
      
      sphereRepresentation->HandleVisibilityOn( );
    }
    vtkSphereCallback(){}
    
};

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  
  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  
  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  //renderer->AddActor(actor);
  
  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkSmartPointer<vtkSphereWidget2> sphereWidget = 
      vtkSmartPointer<vtkSphereWidget2>::New();
  sphereWidget->SetInteractor(renderWindowInteractor);
  sphereWidget->CreateDefaultRepresentation();
  
  vtkSmartPointer<vtkSphereCallback> sphereCallback = 
      vtkSmartPointer<vtkSphereCallback>::New();
 
  sphereWidget->AddObserver(vtkCommand::InteractionEvent,sphereCallback);
  
  // render an image (lights and cameras are created automatically)
  renderWindow->Render();
  
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  sphereWidget->On();
  
  // begin mouse interaction
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
