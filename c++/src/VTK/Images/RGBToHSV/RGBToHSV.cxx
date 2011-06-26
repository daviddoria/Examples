#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <vtkImageRGBToHSV.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkImageActor.h>
#include <vtkImageExtractComponents.h>
#include <vtkTesting.h>

int main(int argc, char *argv[])
{

  std::string inputFilename;
  // Verify command line arguments
  if(argc != 2)
    {
    // Locate VTK_DATA_ROOT
    vtkSmartPointer<vtkTesting> testHelper =
      vtkSmartPointer<vtkTesting>::New();
    std::string dataRoot = testHelper->GetDataRoot();
    inputFilename = dataRoot + "/Data/beach.jpg";
    }
  else
    {
    // Parse command line arguments
    inputFilename = argv[1];
    }

  // Read JPG file
  vtkSmartPointer<vtkJPEGReader> reader =
    vtkSmartPointer<vtkJPEGReader>::New();
  reader->SetFileName(inputFilename.c_str());
  reader->Update();

  vtkSmartPointer<vtkImageRGBToHSV> hsvFilter =
    vtkSmartPointer<vtkImageRGBToHSV>::New();
  hsvFilter->SetInputConnection(reader->GetOutputPort());
  hsvFilter->Update();

   vtkSmartPointer<vtkImageExtractComponents> extractHueFilter =
    vtkSmartPointer<vtkImageExtractComponents>::New();
  extractHueFilter->SetInputConnection(reader->GetOutputPort());
  extractHueFilter->SetComponents(0);
  extractHueFilter->Update();

  vtkSmartPointer<vtkImageExtractComponents> extractSaturationFilter =
    vtkSmartPointer<vtkImageExtractComponents>::New();
  extractSaturationFilter->SetInputConnection(reader->GetOutputPort());
  extractSaturationFilter->SetComponents(1);
  extractSaturationFilter->Update();

  vtkSmartPointer<vtkImageExtractComponents> extractValueFilter =
    vtkSmartPointer<vtkImageExtractComponents>::New();
  extractValueFilter->SetInputConnection(reader->GetOutputPort());
  extractValueFilter->SetComponents(2);
  extractValueFilter->Update();

  // Create actors
  vtkSmartPointer<vtkImageActor> inputActor =
    vtkSmartPointer<vtkImageActor>::New();
  inputActor->SetInput(reader->GetOutput());

  vtkSmartPointer<vtkImageActor> hActor =
    vtkSmartPointer<vtkImageActor>::New();
  hActor->SetInput(extractHueFilter->GetOutput());

  vtkSmartPointer<vtkImageActor> sActor =
    vtkSmartPointer<vtkImageActor>::New();
  sActor->SetInput(extractSaturationFilter->GetOutput());

  vtkSmartPointer<vtkImageActor> vActor =
    vtkSmartPointer<vtkImageActor>::New();
  vActor->SetInput(extractValueFilter->GetOutput());

  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double inputViewport[4] = {0.0, 0.0, 0.25, 1.0};
  double hViewport[4] = {0.25, 0.0, 0.5, 1.0};
  double sViewport[4] = {0.5, 0.0, 0.75, 1.0};
  double vViewport[4] = {0.75, 0.0, 1.0, 1.0};

  // Setup renderers
  vtkSmartPointer<vtkRenderer> inputRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  inputRenderer->SetViewport(inputViewport);
  inputRenderer->AddActor(inputActor);
  inputRenderer->ResetCamera();
  inputRenderer->SetBackground(.4, .5, .9);

  vtkSmartPointer<vtkRenderer> hRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  hRenderer->SetViewport(hViewport);
  hRenderer->AddActor(hActor);
  hRenderer->ResetCamera();
  hRenderer->SetBackground(.4, .5, .6);

  vtkSmartPointer<vtkRenderer> sRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  sRenderer->SetViewport(sViewport);
  sRenderer->AddActor(sActor);
  sRenderer->ResetCamera();
  sRenderer->SetBackground(.4, .5, .7);

  vtkSmartPointer<vtkRenderer> vRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  vRenderer->SetViewport(vViewport);
  vRenderer->AddActor(vActor);
  vRenderer->ResetCamera();
  vRenderer->SetBackground(.4, .5, .8);

  // Setup render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(1200, 300);
  renderWindow->AddRenderer(inputRenderer);
  renderWindow->AddRenderer(hRenderer);
  renderWindow->AddRenderer(sRenderer);
  renderWindow->AddRenderer(vRenderer);

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
