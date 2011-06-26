#include <vtkSmartPointer.h>
#include <vtkInteractorStyleUser.h>
#include <vtkProperty.h>
#include <vtkOutlineFilter.h>
#include <vtkCommand.h>
#include <vtkSliderWidget.h>
#include <vtkSliderRepresentation.h>
#include <vtkSliderRepresentation3D.h>
#include <vtkImageData.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkContourFilter.h>
#include <vtkXMLImageDataWriter.h>

void CreateData(vtkImageData* data);

class vtkSliderCallback : public vtkCommand
{
public:
  static vtkSliderCallback *New()
    {
    return new vtkSliderCallback;
    }
  virtual void Execute(vtkObject *caller, unsigned long, void*)
    {
    vtkSliderWidget *sliderWidget =
      reinterpret_cast<vtkSliderWidget*>(caller);
    double value = static_cast<vtkSliderRepresentation *>(sliderWidget->GetRepresentation())->GetValue();
    this->ContourFilter->GenerateValues(1, value, value);

    }
  vtkSliderCallback():ContourFilter(NULL) {}
  vtkContourFilter* ContourFilter;
};

int main(int, char *[])
{
  vtkSmartPointer<vtkImageData> data =
    vtkSmartPointer<vtkImageData>::New();
  CreateData(data);

  // Create an isosurface
  vtkSmartPointer<vtkContourFilter> contourFilter =
    vtkSmartPointer<vtkContourFilter>::New();
  contourFilter->SetInput(data);
  contourFilter->GenerateValues(1, 1, 2); // (numContours, rangeStart, rangeEnd)

  // Map the contours to graphical primitives
  vtkSmartPointer<vtkPolyDataMapper> contourMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  contourMapper->SetInput(contourFilter->GetOutput());

  // Create an actor for the contours
  vtkSmartPointer<vtkActor> contourActor =
    vtkSmartPointer<vtkActor>::New();
  contourActor->SetMapper(contourMapper);

  // Create the outline
  vtkSmartPointer<vtkOutlineFilter> outlineFilter =
    vtkSmartPointer<vtkOutlineFilter>::New();
  outlineFilter->SetInput(data);
  vtkSmartPointer<vtkPolyDataMapper> outlineMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  outlineMapper->SetInputConnection(outlineFilter->GetOutputPort());
  vtkSmartPointer<vtkActor> outlineActor =
    vtkSmartPointer<vtkActor>::New();
  outlineActor->SetMapper(outlineMapper);

  // Visualize
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  renderer->AddActor(contourActor);
  renderer->AddActor(outlineActor);

  vtkSmartPointer<vtkSliderRepresentation3D> sliderRep =
    vtkSmartPointer<vtkSliderRepresentation3D>::New();
  sliderRep->SetMinimumValue(0.0);
  sliderRep->SetMaximumValue(30.0);
  sliderRep->SetValue(1);
  sliderRep->SetTitleText("Contour value");
  sliderRep->SetPoint1InWorldCoordinates(-20, -30, 0);
  sliderRep->SetPoint2InWorldCoordinates(0, -30, 0);
  sliderRep->SetSliderWidth(.2);

  vtkSmartPointer<vtkSliderWidget> sliderWidget =
    vtkSmartPointer<vtkSliderWidget>::New();
  sliderWidget->SetInteractor(interactor);
  sliderWidget->SetRepresentation(sliderRep);
  sliderWidget->SetAnimationModeToAnimate();
  sliderWidget->EnabledOn();

  vtkSmartPointer<vtkSliderCallback> callback =
    vtkSmartPointer<vtkSliderCallback>::New();
  callback->ContourFilter = contourFilter;

  sliderWidget->AddObserver(vtkCommand::InteractionEvent,callback);

  vtkSmartPointer<vtkInteractorStyleUser> style =
    vtkSmartPointer<vtkInteractorStyleUser>::New();
  interactor->SetInteractorStyle(style);

  renderWindow->SetSize(300,300);
  renderWindow->Render();
  renderWindow->Render();
  interactor->Start();

  return EXIT_SUCCESS;
}

void CreateData(vtkImageData* data)
{
  data->SetExtent(-25,25,-25,25,0,0);
  data->SetNumberOfScalarComponents(1);
  data->SetScalarTypeToDouble();

  int* extent = data->GetExtent();

  for (int y = extent[2]; y <= extent[3]; y++)
    {
    for (int x = extent[0]; x <= extent[1]; x++)
      {
      double* pixel = static_cast<double*>(data->GetScalarPointer(x,y,0));
      pixel[0] = sqrt(pow(x,2) + pow(y,2));
      }
    }

  vtkSmartPointer<vtkXMLImageDataWriter> writer =
    vtkSmartPointer<vtkXMLImageDataWriter>::New();
  writer->SetFileName("data.vti");
  writer->SetInputConnection(data->GetProducerPort());
  writer->Write();
}