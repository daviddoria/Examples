#include "itkImage.h"
#include <itkImageFileReader.h>
#include <itkRegionOfInterestImageFilter.h>

#include <itkImageToVTKImageFilter.h>

#include "vtkImageViewer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkSmartPointer.h"
#include "vtkImageActor.h"
#include "vtkInteractorStyleImage.h"
#include "vtkRenderer.h"

#include <iostream>
#include <string>

int main(int argc, char *argv[])
{
  if(argc < 2)
    {
    std::cerr << "Required: filename" << std::endl;
    return EXIT_FAILURE;
    }

  std::string filename = argv[1];

  typedef itk::Image<unsigned char, 2> ImageType;
  typedef itk::ImageFileReader<ImageType> ReaderType;
  typedef itk::ImageToVTKImageFilter<ImageType> ConnectorType;

  ReaderType::Pointer reader = ReaderType::New();
  ConnectorType::Pointer originalConnector = ConnectorType::New();
  ConnectorType::Pointer extractedConnector = ConnectorType::New();

  reader->SetFileName(filename.c_str());
  originalConnector->SetInput(reader->GetOutput());

  vtkSmartPointer<vtkImageActor> originalActor =
    vtkSmartPointer<vtkImageActor>::New();
  originalActor->SetInput(originalConnector->GetOutput());

  typedef itk::RegionOfInterestImageFilter< ImageType,
                                            ImageType > FilterType;

  FilterType::Pointer filter = FilterType::New();

  ImageType::IndexType start;
  start[0] = 50;
  start[1] = 50;

  ImageType::SizeType size;
  size[0] = 100;
  size[1] = 100;

  ImageType::RegionType desiredRegion;
  desiredRegion.SetSize(size);
  desiredRegion.SetIndex(start);

  filter->SetRegionOfInterest(desiredRegion);
  filter->SetInput(reader->GetOutput());

  extractedConnector->SetInput(filter->GetOutput());

  vtkSmartPointer<vtkImageActor> extractedActor =
    vtkSmartPointer<vtkImageActor>::New();
  extractedActor->SetInput(extractedConnector->GetOutput());

  // There will be one render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(600, 300);

  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double leftViewport[4] = {0.0, 0.0, 0.5, 1.0};
  double rightViewport[4] = {0.5, 0.0, 1.0, 1.0};

  // Setup both renderers
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

  // Add the sphere to the left and the cube to the right
  leftRenderer->AddActor(originalActor);
  rightRenderer->AddActor(extractedActor);

  leftRenderer->ResetCamera();
  rightRenderer->ResetCamera();

  renderWindow->Render();

  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();

  interactor->SetInteractorStyle(style);

  interactor->Start();

  return EXIT_SUCCESS;
}