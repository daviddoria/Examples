#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyDataMapper.h>
#include <vtkSeedWidget.h>
#include <vtkSeedRepresentation.h>
#include <vtkPointHandleRepresentation2D.h>
#include <vtkProperty2D.h>

class vtkSeedCallback : public vtkCommand
{
  public:
    static vtkSeedCallback *New()
    { 
      return new vtkSeedCallback; 
    }
    
    vtkSeedCallback() {}
    
    virtual void Execute(vtkObject*, unsigned long event, void *calldata)
    {
      if (event == vtkCommand::PlacePointEvent)
        {
        cout << "Point placed, total of: " 
            << this->SeedRepresentation->GetNumberOfSeeds() << endl;
        
        if(this->SeedRepresentation->GetNumberOfSeeds() > 2)
          {
          this->SeedWidget->DeleteSeed(0);
          }
        }
      if (event == vtkCommand::InteractionEvent)
        {
        if (calldata)
          {
          cout << "Interacting with seed : " 
              << *(static_cast< int * >(calldata)) << endl;
          }
        }
      
      cout << "List of seeds (Display coordinates):" << endl;
      for(unsigned int i = 0; i < this->SeedRepresentation->GetNumberOfSeeds(); i++)
        {
        double pos[3];
        this->SeedRepresentation->GetSeedDisplayPosition(i, pos);
        cout << "(" << pos[0] << " " << pos[1] << " " << pos[2] << ")" << endl;
        }
        
    }
    
    void SetRepresentation(vtkSmartPointer<vtkSeedRepresentation> rep) {this->SeedRepresentation = rep;}
    void SetWidget(vtkSmartPointer<vtkSeedWidget> widget) {this->SeedWidget = widget;}
  private:
    vtkSmartPointer<vtkSeedRepresentation> SeedRepresentation;
    vtkSmartPointer<vtkSeedWidget> SeedWidget;
};

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  
  //Create a mapper and actor
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
  renderer->AddActor(actor);
  
  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Create the representation
  vtkSmartPointer<vtkPointHandleRepresentation2D> handle = 
      vtkSmartPointer<vtkPointHandleRepresentation2D>::New();
  handle->GetProperty()->SetColor(1,0,0);
  vtkSmartPointer<vtkSeedRepresentation> rep = 
      vtkSmartPointer<vtkSeedRepresentation>::New();
  rep->SetHandleRepresentation(handle);
  
  //seed widget
  vtkSmartPointer<vtkSeedWidget> seedWidget = 
      vtkSmartPointer<vtkSeedWidget>::New();
  seedWidget->SetInteractor(renderWindowInteractor);
  seedWidget->SetRepresentation(rep);
  
  vtkSmartPointer<vtkSeedCallback> seedCallback = 
      vtkSmartPointer<vtkSeedCallback>::New();
  seedCallback->SetRepresentation(rep);
  seedCallback->SetWidget(seedWidget);
  seedWidget->AddObserver(vtkCommand::PlacePointEvent,seedCallback);
  seedWidget->AddObserver(vtkCommand::InteractionEvent,seedCallback);
  
  renderWindow->Render();
  
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  seedWidget->On();
  
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
