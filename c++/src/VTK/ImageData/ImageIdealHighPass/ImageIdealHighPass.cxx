#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageFFT.h>
#include <vtkImageMandelbrotSource.h>
#include <vtkImageIdealHighPass.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkImageActor.h>
#include <vtkImageEllipsoidSource.h>
#include <vtkImageCast.h>
#include <vtkImageRFFT.h>

int main(int, char *[])
{
  // Create an image
  vtkSmartPointer<vtkImageMandelbrotSource> source =
    vtkSmartPointer<vtkImageMandelbrotSource>::New();
  source->Update();

  vtkSmartPointer<vtkImageCast> originalCastFilter =
    vtkSmartPointer<vtkImageCast>::New();
  originalCastFilter->SetInputConnection(source->GetOutputPort());
  originalCastFilter->SetOutputScalarTypeToUnsignedChar();
  originalCastFilter->Update();
  
  vtkSmartPointer<vtkImageActor> originalActor =
    vtkSmartPointer<vtkImageActor>::New();
  originalActor->SetInput(originalCastFilter->GetOutput());

  vtkSmartPointer<vtkImageFFT> fftFilter =
    vtkSmartPointer<vtkImageFFT>::New();
  fftFilter->SetInputConnection(originalCastFilter->GetOutputPort());
  fftFilter->Update();

  vtkSmartPointer<vtkImageIdealHighPass> highPassFilter = 
    vtkSmartPointer<vtkImageIdealHighPass>::New();
  highPassFilter->SetInputConnection(fftFilter->GetOutputPort());
  highPassFilter->Update();

  vtkSmartPointer<vtkImageRFFT> rfftFilter =
    vtkSmartPointer<vtkImageRFFT>::New();
  rfftFilter->SetInputConnection(highPassFilter->GetOutputPort());
  rfftFilter->Update();

  vtkSmartPointer<vtkImageCast> outputCastFilter =
    vtkSmartPointer<vtkImageCast>::New();
  outputCastFilter->SetInputConnection(rfftFilter->GetOutputPort());
  outputCastFilter->SetOutputScalarTypeToUnsignedChar();
  outputCastFilter->Update();

  vtkSmartPointer<vtkImageActor> highPassActor =
    vtkSmartPointer<vtkImageActor>::New();
  highPassActor->SetInput(outputCastFilter->GetOutput());

  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double originalViewport[4] = {0.0, 0.0, 0.5, 1.0};
  double highPassViewport[4] = {0.5, 0.0, 1.0, 1.0};

  // Setup renderers
  vtkSmartPointer<vtkRenderer> originalRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  originalRenderer->SetViewport(originalViewport);
  originalRenderer->AddActor(originalActor);
  originalRenderer->ResetCamera();
  originalRenderer->SetBackground(.4, .5, .6);

  vtkSmartPointer<vtkRenderer> highPassRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  highPassRenderer->SetViewport(highPassViewport);
  highPassRenderer->AddActor(highPassActor);
  highPassRenderer->ResetCamera();
  highPassRenderer->SetBackground(.4, .5, .7);

  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(600, 300);
  renderWindow->AddRenderer(originalRenderer);
  renderWindow->AddRenderer(highPassRenderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();

  renderWindowInteractor->SetInteractorStyle(style);

  renderWindowInteractor->SetRenderWindow(renderWindow);
  renderWindowInteractor->Initialize();

  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
