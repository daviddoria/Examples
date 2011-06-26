#include <vtkImageData.h>
#include <vtkInteractorStyleImage.h>
#include <vtkDoubleArray.h>
#include <vtkArrowSource.h>
#include <vtkGlyph3DMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageActor.h>
#include <vtkPointData.h>
#include <vtkImageMathematics.h>
#include <vtkImageShiftScale.h>
#include <vtkJPEGReader.h>
#include <vtkJPEGWriter.h>
#include <vtkSmartPointer.h>
#include <vtkImageExtractComponents.h>
#include <vtkImageRGBToHSV.h>
#include <vtkImageGradient.h>
#include <vtkImageCast.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkXMLImageDataWriter.h>

int main(int, char *[])
{
  // Create an image
  vtkSmartPointer<vtkImageCanvasSource2D> imageSource =
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  imageSource->SetScalarTypeToUnsignedChar();
  imageSource->SetNumberOfScalarComponents(1);
  imageSource->SetExtent(0, 200, 0, 200, 0, 0);
  imageSource->SetDrawColor(0, 0, 0);
  imageSource->FillBox(0, 200, 0, 200);
  imageSource->SetDrawColor(255, 0, 0);
  imageSource->FillBox(10, 30,  10, 30);
  imageSource->Update();

  vtkSmartPointer<vtkImageGradient> gradientFilter =
    vtkSmartPointer<vtkImageGradient>::New();
  gradientFilter->SetInputConnection(imageSource->GetOutputPort());
  gradientFilter->SetDimensionality(3);
  gradientFilter->Update();

  vtkSmartPointer<vtkXMLImageDataWriter> writer =
    vtkSmartPointer<vtkXMLImageDataWriter>::New();
  writer->SetFileName("test.vti");
  writer->SetInputConnection(gradientFilter->GetOutputPort());
  writer->Write();

  // Extract the x component of the gradient
  vtkSmartPointer<vtkImageExtractComponents> extractXFilter =
    vtkSmartPointer<vtkImageExtractComponents>::New();
  extractXFilter->SetComponents(0);
  extractXFilter->SetInputConnection(gradientFilter->GetOutputPort());
  extractXFilter->Update();

  double xRange[2];
  extractXFilter->GetOutput()->GetPointData()->GetScalars()->GetRange(xRange);

  // Gradient could be negative, so take the absolute value
  vtkSmartPointer<vtkImageMathematics> imageAbsX =
    vtkSmartPointer<vtkImageMathematics>::New();
  imageAbsX->SetOperationToAbsoluteValue();
  imageAbsX->SetInputConnection(extractXFilter->GetOutputPort());
  imageAbsX->Update();

  // Scale the output (0,255)
  vtkSmartPointer<vtkImageShiftScale> shiftScaleX =
    vtkSmartPointer<vtkImageShiftScale>::New();
  shiftScaleX->SetOutputScalarTypeToUnsignedChar();
  shiftScaleX->SetScale(255 / xRange[1]);
  shiftScaleX->SetInputConnection(imageAbsX->GetOutputPort());
  shiftScaleX->Update();

  // Extract the y component of the gradient
  vtkSmartPointer<vtkImageExtractComponents> extractYFilter =
    vtkSmartPointer<vtkImageExtractComponents>::New();
  extractYFilter->SetComponents(1);
  extractYFilter->SetInputConnection(gradientFilter->GetOutputPort());
  extractYFilter->Update();

  double yRange[2];
  extractYFilter->GetOutput()->GetPointData()->GetScalars()->GetRange( yRange );

  // Gradient could be negative, so take the absolute value
  vtkSmartPointer<vtkImageMathematics> imageAbsY =
    vtkSmartPointer<vtkImageMathematics>::New();
  imageAbsY->SetOperationToAbsoluteValue();
  imageAbsY->SetInputConnection(extractYFilter->GetOutputPort());
  imageAbsY->Update();

  // Scale the output (0,255)
  vtkSmartPointer<vtkImageShiftScale> shiftScaleY =
    vtkSmartPointer<vtkImageShiftScale>::New();
  shiftScaleY->SetOutputScalarTypeToUnsignedChar();
  shiftScaleY->SetScale(255 / yRange[1]);
  shiftScaleY->SetInputConnection(imageAbsY->GetOutputPort());
  shiftScaleY->Update();

  // Visualize

  // (xmin, ymin, xmax, ymax)
  double originalViewport[4] = {0.0, 0.0, 0.25, 1.0};
  double xGradientViewport[4] = {0.25, 0.0, 0.5, 1.0};
  double yGradientViewport[4] = {0.5, 0.0, 0.75, 1.0};
  double vectorGradientViewport[4] = {0.75, 0.0, 1.0, 1.0};

  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> originalRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  originalRenderer->SetViewport(originalViewport);

  vtkSmartPointer<vtkRenderer> xGradientRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  xGradientRenderer->SetViewport(xGradientViewport);

  vtkSmartPointer<vtkRenderer> yGradientRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  yGradientRenderer->SetViewport(yGradientViewport);

  vtkSmartPointer<vtkRenderer> vectorGradientRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  vectorGradientRenderer->SetViewport(vectorGradientViewport);

  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(1200,300);
  renderWindow->AddRenderer(originalRenderer);
  renderWindow->AddRenderer(xGradientRenderer);
  renderWindow->AddRenderer(yGradientRenderer);
  renderWindow->AddRenderer(vectorGradientRenderer);

  vtkSmartPointer<vtkImageActor> originalActor =
    vtkSmartPointer<vtkImageActor>::New();
  originalActor->SetInput(imageSource->GetOutput());

  vtkSmartPointer<vtkImageActor> xGradientActor =
    vtkSmartPointer<vtkImageActor>::New();
  xGradientActor->SetInput(shiftScaleX->GetOutput());

  vtkSmartPointer<vtkImageActor> yGradientActor =
    vtkSmartPointer<vtkImageActor>::New();
  yGradientActor->SetInput(shiftScaleY->GetOutput());

  vtkSmartPointer<vtkArrowSource> arrowSource =
    vtkSmartPointer<vtkArrowSource>::New();
  gradientFilter->GetOutput()->GetPointData()->SetActiveVectors("ImageScalarsGradient");

  vtkSmartPointer<vtkGlyph3DMapper> vectorGradientMapper =
    vtkSmartPointer<vtkGlyph3DMapper>::New();
  vectorGradientMapper->ScalingOn();
  vectorGradientMapper->SetScaleFactor(.01);
  vectorGradientMapper->SetSourceConnection(arrowSource->GetOutputPort());
  vectorGradientMapper->SetInputConnection(gradientFilter->GetOutputPort());
  vectorGradientMapper->Update();

  vtkSmartPointer<vtkActor> vectorGradientActor =
    vtkSmartPointer<vtkActor>::New();
  vectorGradientActor->SetMapper(vectorGradientMapper);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();
  renderWindowInteractor->SetInteractorStyle(style);

  // Add the actor to the scene
  originalRenderer->AddActor(originalActor);
  xGradientRenderer->AddActor(xGradientActor);
  yGradientRenderer->AddActor(yGradientActor);

  vectorGradientRenderer->AddActor(originalActor);
  vectorGradientRenderer->AddActor(vectorGradientActor);

  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
