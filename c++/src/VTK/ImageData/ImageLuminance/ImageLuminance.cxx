#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkJPEGWriter.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageLuminance.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkImageActor.h>
#include <vtkImageEllipsoidSource.h>
#include <vtkImageCast.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkImageData> input =
    vtkSmartPointer<vtkImageData>::New();
  if(argc == 1) // No args specified
    {
    // Create an image of a rectangle
    vtkSmartPointer<vtkImageCanvasSource2D> source =
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
    source->SetScalarTypeToUnsignedChar();
    source->SetNumberOfScalarComponents(3);
    source->SetExtent(0, 200, 0, 200, 0, 0);

    // Clear the image
    source->SetDrawColor(0,0,0);
    source->FillBox(0,200,0,200);

    // Draw a red box
    source->SetDrawColor(255,0,0);
    source->FillBox(100,120,100,120);
    source->Update();

    input->ShallowCopy(source->GetOutput());
    }
  else
    {
    vtkSmartPointer<vtkJPEGReader> reader =
      vtkSmartPointer<vtkJPEGReader>::New();
    reader->SetFileName(argv[1]);
    reader->Update();
    input->ShallowCopy(reader->GetOutput());
    }

  std::cout << "Input has " << input->GetNumberOfScalarComponents() << " components." << std::endl;

  vtkSmartPointer<vtkImageLuminance> luminanceFilter = 
    vtkSmartPointer<vtkImageLuminance>::New();
  luminanceFilter->SetInputConnection(input->GetProducerPort());
  luminanceFilter->Update();

  std::cout << "Output has " << luminanceFilter->GetOutput()->GetNumberOfScalarComponents() << " components." << std::endl;

  // Create actors
  vtkSmartPointer<vtkImageActor> originalActor =
    vtkSmartPointer<vtkImageActor>::New();
  originalActor->SetInput(input);

  vtkSmartPointer<vtkImageActor> luminanceActor =
    vtkSmartPointer<vtkImageActor>::New();
  luminanceActor->SetInput(luminanceFilter->GetOutput());

  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double originalViewport[4] = {0.0, 0.0, 0.5, 1.0};
  double luminanceViewport[4] = {0.5, 0.0, 1.0, 1.0};

  // Setup renderers
  vtkSmartPointer<vtkRenderer> originalRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  originalRenderer->SetViewport(originalViewport);
  originalRenderer->AddActor(originalActor);
  originalRenderer->ResetCamera();
  originalRenderer->SetBackground(.4, .5, .6);

  vtkSmartPointer<vtkRenderer> luminanceRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  luminanceRenderer->SetViewport(luminanceViewport);
  luminanceRenderer->AddActor(luminanceActor);
  luminanceRenderer->ResetCamera();
  luminanceRenderer->SetBackground(.4, .5, .7);

  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(600, 300);
  renderWindow->AddRenderer(originalRenderer);
  renderWindow->AddRenderer(luminanceRenderer);

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
