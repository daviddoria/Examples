#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkMinimumMaximumImageCalculator.h"
#include "itkImageFileReader.h"

#include <itkImageToVTKImageFilter.h>

#include "vtkImageViewer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkSmartPointer.h"
#include "vtkImageActor.h"
#include "vtkInteractorStyleImage.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkProperty.h"

typedef itk::Image<unsigned char, 2>  ImageType;

int main(int argc, char*argv[])
{
  if(argc < 2)
    {
    std::cerr << "Required: filename" << std::endl;

    return EXIT_FAILURE;
    }
  std::string inputFilename = argv[1];

  typedef itk::Image< unsigned char, 2 >  ImageType;

  typedef itk::ImageFileReader<ImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();

  reader->SetFileName(inputFilename.c_str());
  reader->Update();

  typedef itk::MinimumMaximumImageCalculator <ImageType>
          ImageCalculatorFilterType;

  ImageCalculatorFilterType::Pointer imageCalculatorFilter
          = ImageCalculatorFilterType::New ();
  imageCalculatorFilter->SetImage(reader->GetOutput());
  imageCalculatorFilter->Compute();

  typedef itk::ImageToVTKImageFilter<ImageType> ConnectorType;
  ConnectorType::Pointer originalConnector = ConnectorType::New();

  originalConnector->SetInput(reader->GetOutput());

  vtkSmartPointer<vtkImageActor> originalActor =
    vtkSmartPointer<vtkImageActor>::New();
  originalActor->SetInput(originalConnector->GetOutput());

  vtkSmartPointer<vtkSphereSource> minimumSphereSource =
    vtkSmartPointer<vtkSphereSource>::New();
  ImageType::IndexType minimumLocation = imageCalculatorFilter->GetIndexOfMinimum();
  minimumSphereSource->SetCenter(minimumLocation[0], minimumLocation[1], 0);
  minimumSphereSource->SetRadius(10);

  vtkSmartPointer<vtkPolyDataMapper> minimumMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  minimumMapper->SetInputConnection(minimumSphereSource->GetOutputPort());
  vtkSmartPointer<vtkActor> minimumActor =
    vtkSmartPointer<vtkActor>::New();
  minimumActor->SetMapper(minimumMapper);
  minimumActor->GetProperty()->SetColor(0,1,0);
  
  vtkSmartPointer<vtkSphereSource> maximumSphereSource =
    vtkSmartPointer<vtkSphereSource>::New();
  maximumSphereSource->SetRadius(10);
  ImageType::IndexType maximumLocation = imageCalculatorFilter->GetIndexOfMaximum();
  maximumSphereSource->SetCenter(maximumLocation[0], maximumLocation[1], 0);

  vtkSmartPointer<vtkPolyDataMapper> maximumMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  maximumMapper->SetInputConnection(maximumSphereSource->GetOutputPort());
  vtkSmartPointer<vtkActor> maximumActor =
    vtkSmartPointer<vtkActor>::New();
  maximumActor->SetMapper(maximumMapper);
  maximumActor->GetProperty()->SetColor(1,0,0);


  // Visualize
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();

  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(renderer);

  // Add the sphere to the left and the cube to the right
  renderer->AddActor(originalActor);
  renderer->AddActor(minimumActor);
  renderer->AddActor(maximumActor);

  renderer->ResetCamera();

  renderWindow->Render();

  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();
  interactor->SetInteractorStyle(style);

  interactor->Start();

  return EXIT_SUCCESS;
}
