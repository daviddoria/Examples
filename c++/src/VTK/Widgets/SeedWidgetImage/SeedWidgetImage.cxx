#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkImageActor.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkInteractorStyleImage.h>
#include <vtkPointHandleRepresentation2D.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty2D.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSeedWidget.h>
#include <vtkSeedRepresentation.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>

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
	if(this->SeedRepresentation->GetNumberOfSeeds() > 1)
          {
          this->SeedWidget->DeleteSeed(0);
          }
        }
      if (event == vtkCommand::InteractionEvent)
        {
        if (calldata)
          {
	  double pos[3];
	  this->SeedRepresentation->GetSeedDisplayPosition(0, pos);
	  cout << "(" << pos[0] << " " << pos[1] << " " << pos[2] << ")" << endl;
          }
        }
    }
    
    void SetRepresentation(vtkSmartPointer<vtkSeedRepresentation> rep)
    {
      this->SeedRepresentation = rep;
    }
    void SetWidget(vtkSmartPointer<vtkSeedWidget> widget) 
    {
      this->SeedWidget = widget;
    }
    
  private:
    vtkSmartPointer<vtkSeedRepresentation> SeedRepresentation;
    vtkSmartPointer<vtkSeedWidget> SeedWidget;
};

int main(int argc, char *argv[])
{
  // Create an image
  vtkSmartPointer<vtkImageCanvasSource2D> drawing = 
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  drawing->SetScalarTypeToUnsignedChar();
  drawing->SetNumberOfScalarComponents(3);
  drawing->SetExtent(0, 20, 0, 50, 0, 0);
  drawing->FillBox(0,20,0,50);
  drawing->SetDrawColor(255, 0, 0, 0);
  drawing->DrawCircle(9, 10, 5);
  drawing->Update();
  
  vtkSmartPointer<vtkImageActor> imageActor = 
    vtkSmartPointer<vtkImageActor>::New();
  imageActor->SetInput(drawing->GetOutput());
  
  // Create a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
    vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(imageActor);
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  
  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
    
  // Setup interactor style
  vtkSmartPointer<vtkInteractorStyleImage> interactorStyleImage =
    vtkSmartPointer<vtkInteractorStyleImage>::New();
  renderWindowInteractor->SetInteractorStyle(interactorStyleImage);
  
  // Create the representation
  vtkSmartPointer<vtkPointHandleRepresentation2D> handle = 
    vtkSmartPointer<vtkPointHandleRepresentation2D>::New();
  handle->GetProperty()->SetColor(1,0,0);
  vtkSmartPointer<vtkSeedRepresentation> rep = 
    vtkSmartPointer<vtkSeedRepresentation>::New();
  rep->SetHandleRepresentation(handle);
  
  // Seed widget
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
