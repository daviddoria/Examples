#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <vtkImageCast.h>
#include <vtkImageActor.h>
#include <vtkImageMandelbrotSource.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkInteractorStyleImage.h>
#include <vtkImageAnisotropicDiffusion2D.h>

int main(int, char *[])
{
  // Create a float image
  vtkSmartPointer<vtkImageMandelbrotSource> source =
    vtkSmartPointer<vtkImageMandelbrotSource>::New();
  source->Update();

  vtkSmartPointer<vtkImageCast> sourceCast =
    vtkSmartPointer<vtkImageCast>::New();
  sourceCast->SetInputConnection(source->GetOutputPort());
  sourceCast->SetOutputScalarTypeToUnsignedChar();
  sourceCast->Update();

  vtkSmartPointer<vtkImageAnisotropicDiffusion2D> diffusion =
    vtkSmartPointer<vtkImageAnisotropicDiffusion2D>::New();
  diffusion->SetInputConnection(source->GetOutputPort());
  diffusion->Update();
  
  vtkSmartPointer<vtkImageCast> diffusionCast =
    vtkSmartPointer<vtkImageCast>::New();
  diffusionCast->SetInputConnection(diffusion->GetOutputPort());
  diffusionCast->SetOutputScalarTypeToUnsignedChar();
  diffusionCast->Update();

  // Create an actor
  vtkSmartPointer<vtkImageActor> originalActor =
    vtkSmartPointer<vtkImageActor>::New();
  originalActor->SetInput(sourceCast->GetOutput());

  // Create an actor
  vtkSmartPointer<vtkImageActor> diffusionActor =
    vtkSmartPointer<vtkImageActor>::New();
  diffusionActor->SetInput(diffusionCast->GetOutput());

  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double leftViewport[4] = {0.0, 0.0, 0.5, 1.0};
  double rightViewport[4] = {0.5, 0.0, 1.0, 1.0};

  // Setup renderers
  vtkSmartPointer<vtkRenderer> leftRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  leftRenderer->SetViewport(leftViewport);
  leftRenderer->AddActor(originalActor);
  leftRenderer->ResetCamera();

  vtkSmartPointer<vtkRenderer> rightRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  rightRenderer->SetViewport(rightViewport);
  rightRenderer->AddActor(diffusionActor);
  rightRenderer->ResetCamera();

  // Setup render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(600,300);
  renderWindow->AddRenderer(leftRenderer);
  renderWindow->AddRenderer(rightRenderer);

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
