#include <vtkImageActor.h>
#include <vtkImageCast.h>
#include <vtkInteractorStyleImage.h>
#include <vtkImageAccumulate.h>
#include <vtkImageData.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkPNGReader.h>
#include <vtkImageOpenClose3D.h>
#include <vtkImageThreshold.h>
#include <vtkImageMagnify.h>

int main(int argc, char *argv[])
{
  // Handle the arguments
  if(argc < 2)
    {
    std::cout << "Required arguments: filename.png" << std::endl;
    return EXIT_FAILURE;
    }

  // Read the image
  vtkSmartPointer<vtkPNGReader> reader =
    vtkSmartPointer<vtkPNGReader>::New();
  reader->SetFileName(argv[1]);
  reader->Update();

  int dims[3];
  reader->GetOutput()->GetDimensions(dims);
  std::cout << "Dims: " << dims[0] << " " << dims[1] << std::endl;

  vtkSmartPointer<vtkImageMagnify> magnifyFilter =
    vtkSmartPointer<vtkImageMagnify>::New();
  magnifyFilter->SetInputConnection(reader->GetOutputPort());
  magnifyFilter->SetMagnificationFactors(2,1,1);
  magnifyFilter->Update();

  int magnifiedDims[3];
  magnifyFilter->GetOutput()->GetDimensions(magnifiedDims);
  std::cout << "Magnified dims: " << magnifiedDims[0] << " " << magnifiedDims[1] << std::endl;

  vtkSmartPointer<vtkImageActor> originalActor =
    vtkSmartPointer<vtkImageActor>::New();
  originalActor->SetInput(reader->GetOutput());

  vtkSmartPointer<vtkImageActor> magnifiedActor =
    vtkSmartPointer<vtkImageActor>::New();
  magnifiedActor->SetInput(magnifyFilter->GetOutput());

  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double originalViewport[4] = {0.0, 0.0, 0.5, 1.0};
  double magnifiedViewport[4] = {0.5, 0.0, 1.0, 1.0};

  // Setup renderers
  vtkSmartPointer<vtkRenderer> originalRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  originalRenderer->SetViewport(originalViewport);
  originalRenderer->AddActor(originalActor);
  originalRenderer->ResetCamera();
  originalRenderer->SetBackground(.4, .5, .6);

  vtkSmartPointer<vtkRenderer> magnifiedRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  magnifiedRenderer->SetViewport(magnifiedViewport);
  magnifiedRenderer->AddActor(magnifiedActor);
  magnifiedRenderer->ResetCamera();
  magnifiedRenderer->SetBackground(.4, .5, .7);

  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(600, 300);
  renderWindow->AddRenderer(originalRenderer);
  renderWindow->AddRenderer(magnifiedRenderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();

  renderWindowInteractor->SetInteractorStyle(style);

  renderWindowInteractor->SetRenderWindow(renderWindow);
  renderWindowInteractor->Initialize();

  renderWindowInteractor->Start();

  return  EXIT_SUCCESS;
}