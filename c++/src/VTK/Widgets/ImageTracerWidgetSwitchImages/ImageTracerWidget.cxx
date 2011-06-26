#include <vtkSmartPointer.h>
#include <vtkObjectFactory.h>
#include <vtkImageData.h>
#include <vtkCallbackCommand.h>
#include <vtkImageActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkImageTracerWidget.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkInteractorStyleImage.h>
#include <vtkProperty.h>

namespace
{
void CallbackFunction (vtkObject* caller, long unsigned int eventId,
                        void* clientData, void* callData );

void CreateImage1(vtkImageData*);
void CreateImage2(vtkImageData*);

}

class CustomInteractorStyle : public vtkInteractorStyleImage
{
  public:
    static CustomInteractorStyle* New();
    vtkTypeMacro(CustomInteractorStyle, CustomInteractorStyle);

    virtual void OnKeyPress()
    {
      // Get the keypress
      std::string key = this->Interactor->GetKeySym();

      if(key.compare("a") == 0)
        {
        vtkSmartPointer<vtkImageData> image =
          vtkSmartPointer<vtkImageData>::New();
        CreateImage2(image);
        actor->SetInput(image);
        this->Interactor->Render();
        }

      // Forward events
      vtkInteractorStyleImage::OnKeyPress();
    }

  vtkImageTracerWidget* widget;
  vtkImageActor* actor;
};
vtkStandardNewMacro(CustomInteractorStyle);



int main(int, char *[])
{
  vtkSmartPointer<vtkImageData> image =
    vtkSmartPointer<vtkImageData>::New();
  CreateImage1(image);

  vtkSmartPointer<vtkImageActor> actor =
    vtkSmartPointer<vtkImageActor>::New();
  actor->SetInput(image);

  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(actor);
  renderer->ResetCamera();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  /*
  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();
  */
  vtkSmartPointer<CustomInteractorStyle> style =
    vtkSmartPointer<CustomInteractorStyle>::New();
  interactor->SetInteractorStyle(style);

  vtkSmartPointer<vtkImageTracerWidget> tracer =
    vtkSmartPointer<vtkImageTracerWidget>::New();
  //style->widget = tracer;
  style->actor = actor;
  tracer->GetLineProperty()->SetLineWidth(5);
  tracer->GetLineProperty()->SetColor(0,0,1);
  tracer->SetInteractor(interactor);
  tracer->SetViewProp(actor);
  renderWindow->Render();

  // The observer must be added BEFORE the On() call.
  vtkSmartPointer<vtkCallbackCommand> callback =
    vtkSmartPointer<vtkCallbackCommand>::New();
  callback->SetCallback(CallbackFunction);
  tracer->AddObserver(vtkCommand::EndInteractionEvent, callback);

  tracer->On();
  interactor->Start();

  return EXIT_SUCCESS;
}

namespace
{
void CallbackFunction (vtkObject* caller, long unsigned int eventId,
                        void* clientData, void* callData )
{
  vtkImageTracerWidget* tracerWidget =
    static_cast<vtkImageTracerWidget*>(caller);

  vtkSmartPointer<vtkPolyData> path =
    vtkSmartPointer<vtkPolyData>::New();

  tracerWidget->GetPath(path);
  std::cout << "There are " << path->GetNumberOfPoints() << " points in the path." << std::endl;
}


void CreateImage1(vtkImageData* image)
{
  vtkSmartPointer<vtkImageCanvasSource2D> imageSource =
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  imageSource->SetScalarTypeToUnsignedChar();
  imageSource->SetNumberOfScalarComponents(3);
  imageSource->SetExtent(0, 20, 0, 50, 0, 0);
  imageSource->SetDrawColor(0,0,0);
  imageSource->FillBox(0,20,0,50);
  imageSource->SetDrawColor(255,0,0);
  imageSource->FillBox(0,10,0,30);
  imageSource->Update();

  image->ShallowCopy(imageSource->GetOutput());
}

void CreateImage2(vtkImageData* image)
{
  vtkSmartPointer<vtkImageCanvasSource2D> imageSource =
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  imageSource->SetScalarTypeToUnsignedChar();
  imageSource->SetNumberOfScalarComponents(3);
  imageSource->SetExtent(0, 20, 0, 50, 0, 0);
  imageSource->SetDrawColor(0,0,0);
  imageSource->FillBox(0,20,0,50);
  imageSource->SetDrawColor(0,255,0);
  imageSource->FillBox(0,10,0,30);
  imageSource->Update();
  image->ShallowCopy(imageSource->GetOutput());
}

} // end anonymous namespace
