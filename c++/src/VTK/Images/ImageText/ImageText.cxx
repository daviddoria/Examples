#include <vtkFeatureEdges.h>
#include <vtkImageData.h>
#include <vtkImageStencil.h>
#include <vtkJPEGReader.h>
#include <vtkLinearExtrusionFilter.h>
#include <vtkPointData.h>
#include <vtkPolyDataToImageStencil.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkVectorText.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderer.h>
#include <vtkImageActor.h>

int main(int argc, char* argv[])
{
  vtkSmartPointer<vtkJPEGReader> reader =
    vtkSmartPointer<vtkJPEGReader>::New();
  reader->SetFileName(argv[1]);
  reader->Update();

  vtkImageData* image = reader->GetOutput();

  // Create a 2D triangulated set of letters
  vtkSmartPointer<vtkVectorText> vectorText =
    vtkSmartPointer<vtkVectorText>::New();
  vectorText->SetText("hello");

  // Exract the outline edges
  vtkSmartPointer<vtkFeatureEdges> featureEdgesFilter =
    vtkSmartPointer<vtkFeatureEdges>::New();
  featureEdgesFilter->BoundaryEdgesOn();
  featureEdgesFilter->FeatureEdgesOff();
  featureEdgesFilter->NonManifoldEdgesOff();
  featureEdgesFilter->ManifoldEdgesOff();
  featureEdgesFilter->ColoringOff();
  featureEdgesFilter->SetInputConnection(vectorText->GetOutputPort());

  // Extrude the edges to form surfaces that the vtkPolyDataToImageStencil can operate on
  vtkSmartPointer<vtkLinearExtrusionFilter> extrusionFilter =
    vtkSmartPointer<vtkLinearExtrusionFilter>::New();
  extrusionFilter->SetExtrusionTypeToNormalExtrusion();
  extrusionFilter->CappingOff();
  extrusionFilter->SetScaleFactor(1);
  extrusionFilter->SetVector(0, 0, 1);
  extrusionFilter->SetInputConnection(featureEdgesFilter->GetOutputPort());
  extrusionFilter->Update();

  double* bounds = extrusionFilter->GetOutput()->GetBounds();

  // Translate to the origin and straddle the x-y plane
  vtkSmartPointer<vtkTransform> transform =
    vtkSmartPointer<vtkTransform>::New();
  transform->Translate( -bounds[0], -bounds[2], -0.5 );

  vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyDataFilter =
    vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformPolyDataFilter->SetTransform(transform);
  transformPolyDataFilter->SetInputConnection(extrusionFilter->GetOutputPort());
  transformPolyDataFilter->Update();

  // Create an image to stencil the word onto
  int numPixelsX = 200;
  double pixelSizeX = (bounds[1]-bounds[0])/numPixelsX;
  int numPixelsY = static_cast<int>((bounds[3]-bounds[2])/pixelSizeX + 0.5);
  double pixelSizeY = (bounds[3]-bounds[2])/numPixelsY;
  int padding = 10;

  // Stencil the word onto the image
  vtkSmartPointer<vtkPolyDataToImageStencil> polyDataToImageStencil =
    vtkSmartPointer<vtkPolyDataToImageStencil>::New();
  polyDataToImageStencil->SetInput(transformPolyDataFilter->GetOutput());
  polyDataToImageStencil->SetOutputOrigin(image->GetOrigin());
  polyDataToImageStencil->SetOutputWholeExtent(image->GetExtent());
  polyDataToImageStencil->SetOutputSpacing(image->GetSpacing());

  vtkSmartPointer<vtkImageStencil> stencil =
    vtkSmartPointer<vtkImageStencil>::New();
  stencil->SetStencil(polyDataToImageStencil->GetOutput());
  stencil->SetBackgroundValue(0);
  stencil->SetInputConnection(image->GetProducerPort());

  // Create an actor
  vtkSmartPointer<vtkImageActor> actor =
    vtkSmartPointer<vtkImageActor>::New();
  actor->SetInput(stencil->GetOutput());

  // Setup renderer
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(actor);
  renderer->ResetCamera();

  // Setup render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // Setup render window interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();

  renderWindowInteractor->SetInteractorStyle(style);

  // Render and start interaction
  renderWindowInteractor->SetRenderWindow(renderWindow);
  renderWindowInteractor->Initialize();

  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}