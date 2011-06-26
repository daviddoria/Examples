#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageIdealLowPass.h>
#include <vtkImageActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkImageFFT.h>
#include <vtkImageRFFT.h>
#include <vtkImageCast.h>

int main(int argc, char *argv[])
{
  if(argc < 2)
    {
    std::cerr << "Required: filename.jpg" << std::endl;
    return EXIT_FAILURE;
    }
    
  // Create an image
  vtkSmartPointer<vtkJPEGReader> reader = 
    vtkSmartPointer<vtkJPEGReader>::New();
  reader->SetFileName(argv[1]);
  reader->Update();

  vtkSmartPointer<vtkImageFFT> fftFilter =
    vtkSmartPointer<vtkImageFFT>::New();
  fftFilter->SetInputConnection(reader->GetOutputPort());
  fftFilter->Update();
  
  vtkSmartPointer<vtkImageIdealLowPass> lowPassFilter = 
    vtkSmartPointer<vtkImageIdealLowPass>::New();
  lowPassFilter->SetInputConnection(fftFilter->GetOutputPort());
  lowPassFilter->Update();

  vtkSmartPointer<vtkImageIdealLowPass> rfftFilter =
    vtkSmartPointer<vtkImageIdealLowPass>::New();
  rfftFilter->SetInputConnection(lowPassFilter->GetOutputPort());
  rfftFilter->Update();

  vtkSmartPointer<vtkImageCast> castFilter =
    vtkSmartPointer<vtkImageCast>::New();
  castFilter->SetInputConnection(rfftFilter->GetOutputPort());
  castFilter->SetOutputScalarTypeToUnsignedChar();
  castFilter->Update();

  vtkSmartPointer<vtkImageActor> originalActor =
    vtkSmartPointer<vtkImageActor>::New();
  originalActor->SetInput(reader->GetOutput());

  vtkSmartPointer<vtkImageActor> erodedActor =
    vtkSmartPointer<vtkImageActor>::New();
  erodedActor->SetInput(castFilter->GetOutput());

  // Visualize
  double leftViewport[4] = {0.0, 0.0, 0.5, 1.0};
  double rightViewport[4] = {0.5, 0.0, 1.0, 1.0};

  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(600, 300);

  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  vtkSmartPointer<vtkRenderer> leftRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(leftRenderer);
  leftRenderer->SetViewport(leftViewport);
  leftRenderer->SetBackground(.6, .5, .4);

  vtkSmartPointer<vtkRenderer> rightRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(rightRenderer);
  rightRenderer->SetViewport(rightViewport);
  rightRenderer->SetBackground(.4, .5, .6);

  leftRenderer->AddActor(originalActor);
  rightRenderer->AddActor(erodedActor);

  leftRenderer->ResetCamera();
  rightRenderer->ResetCamera();

  renderWindow->Render();
  interactor->Start();
  return EXIT_SUCCESS;
}
