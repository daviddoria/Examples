#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkMeanSquareRegistrationFunction.h>
#include <itkRegionOfInterestImageFilter.h>
#include <itkCastImageFilter.h>
#include <itkRescaleIntensityImageFilter.h>
#include <itkResampleImageFilter.h>
#include "itkImageRegionIterator.h"

#include <itkImageToVTKImageFilter.h>

#include "vtkImageViewer.h"
#include "itkImageFileWriter.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkSmartPointer.h"
#include "vtkImageActor.h"
#include "vtkInteractorStyleImage.h"
#include "vtkRenderer.h"

#include <iostream>
#include <string>

typedef itk::Image<float, 2> FloatImageType;

void FullMask(FloatImageType::Pointer mask);

int main(int argc, char *argv[])
{
  if(argc < 2)
    {
    std::cerr << "Required: filename" << std::endl;
    return EXIT_FAILURE;
    }
  
  std::string filename = argv[1];

  typedef itk::Image<unsigned char, 2> UnsignedCharImageType;
  
  typedef itk::ImageFileReader<FloatImageType> ReaderType;
  typedef itk::ImageToVTKImageFilter<UnsignedCharImageType> ConnectorType;

  // Read the image
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(filename.c_str());
  reader->Update();

  // Create a mask
  FloatImageType::Pointer mask = FloatImageType::New();
  mask->SetRegions(reader->GetOutput()->GetLargestPossibleRegion());
  mask->Allocate();
  FullMask(mask);
  
  // Setup the original image actor
  typedef itk::CastImageFilter< FloatImageType,
                                UnsignedCharImageType > CastFilterType;
  CastFilterType::Pointer originalCastFilter = CastFilterType::New();
  originalCastFilter->SetInput(reader->GetOutput());
  
  ConnectorType::Pointer originalConnector = ConnectorType::New();
  originalConnector->SetInput(originalCastFilter->GetOutput());

  vtkSmartPointer<vtkImageActor> originalActor =
    vtkSmartPointer<vtkImageActor>::New();
  originalActor->SetInput(originalConnector->GetOutput());

  // Extract a small region
  typedef itk::RegionOfInterestImageFilter< FloatImageType,
                                            FloatImageType > ExtractFilterType;

  ExtractFilterType::Pointer extractFilter = ExtractFilterType::New();

  FloatImageType::IndexType start;
  start[0] = 50;
  start[1] = 50;

  FloatImageType::SizeType size;
  size[0] = 100;
  size[1] = 100;

  FloatImageType::RegionType desiredRegion;
  desiredRegion.SetSize(size);
  desiredRegion.SetIndex(start);

  extractFilter->SetRegionOfInterest(desiredRegion);
  extractFilter->SetInput(reader->GetOutput());
  extractFilter->Update();

  // Display extracted region (kernel)
  CastFilterType::Pointer kernelCastFilter = CastFilterType::New();
  kernelCastFilter->SetInput(extractFilter->GetOutput());
  
  ConnectorType::Pointer extractedConnector = ConnectorType::New();
  extractedConnector->SetInput(kernelCastFilter->GetOutput());

  vtkSmartPointer<vtkImageActor> extractedActor =
    vtkSmartPointer<vtkImageActor>::New();
  extractedActor->SetInput(extractedConnector->GetOutput());

  // Perform mean square registration
  // <TFixedImage, TMovingImage, TDeformationField>
  typedef itk::MeanSquareRegistrationFunction<FloatImageType, FloatImageType, FloatImageType> RegistrationFilterType;
  RegistrationFilterType::Pointer registrationFilter = RegistrationFilterType::New();
  registrationFilter->SetFixedImage(reader->GetOutput());
  registrationFilter->SetMovingImage(extractFilter->GetOutput());
  registrationFilter->SetMaskImage(mask);
  registrationFilter->Update();

  // Display the correlation image
  typedef itk::RescaleIntensityImageFilter< FloatImageType, UnsignedCharImageType > RescaleFilterType;
  RescaleFilterType::Pointer rescaleFilter = RescaleFilterType::New();
  rescaleFilter->SetInput(correlationFilter->GetOutput());
  rescaleFilter->SetOutputMinimum(0);
  rescaleFilter->SetOutputMaximum(255);
  rescaleFilter->Update();
  
  ConnectorType::Pointer correlationConnector = ConnectorType::New();
  correlationConnector->SetInput(rescaleFilter->GetOutput());

  vtkSmartPointer<vtkImageActor> correlationActor =
    vtkSmartPointer<vtkImageActor>::New();
  correlationActor->SetInput(correlationConnector->GetOutput());

  // Visualize
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(900, 300);

  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  double leftViewport[4] = {0.0, 0.0, 0.33, 1.0};
  double centerViewport[4] = {0.33, 0.0, 0.66, 1.0};
  double rightViewport[4] = {0.66, 0.0, 1.0, 1.0};

  vtkSmartPointer<vtkRenderer> leftRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(leftRenderer);
  leftRenderer->SetViewport(leftViewport);
  leftRenderer->SetBackground(.6, .5, .4);

  vtkSmartPointer<vtkRenderer> centerRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(centerRenderer);
  centerRenderer->SetViewport(centerViewport);
  centerRenderer->SetBackground(.5, .5, .6);

  vtkSmartPointer<vtkRenderer> rightRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(rightRenderer);
  rightRenderer->SetViewport(rightViewport);
  rightRenderer->SetBackground(.4, .5, .6);

  leftRenderer->AddActor(originalActor);
  centerRenderer->AddActor(extractedActor);
  rightRenderer->AddActor(correlationActor);

  leftRenderer->ResetCamera();
  rightRenderer->ResetCamera();

  renderWindow->Render();

  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();
  interactor->SetInteractorStyle(style);

  interactor->Start();

  return EXIT_SUCCESS;
}

/*
void FullMask(FloatImageType::Pointer image, FloatImageType::Pointer mask)
{
  typedef itk::ResampleImageFilter<FloatImageType, FloatImageType> ResampleType;
  ResampleType::Pointer resample = ResampleType::New();
  resample->SetInput(mask);
  resample->SetOutputParametersFromImage(image);
  mask = resample->GetOutput();
  
  // Create an image with 2 connected components
  ImageType::RegionType region;
  ImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;
 
  ImageType::SizeType size;
  unsigned int NumRows = 200;
  unsigned int NumCols = 300;
  size[0] = NumRows;
  size[1] = NumCols;
 
  region.SetSize(size);
  region.SetIndex(start);
 
  image->SetRegions(region);
  image->Allocate();
 
  // Make another square
  for(unsigned int r = 40; r < 100; r++)
  {
      for(unsigned int c = 40; c < 100; c++)
      {
          ImageType::IndexType pixelIndex;
          pixelIndex[0] = r;
          pixelIndex[1] = c;
 
          image->SetPixel(pixelIndex, 15);
      }
  }
  
}
*/


void FullMask(FloatImageType::Pointer mask)
{
  itk::ImageRegionIterator<FloatImageType> imageIterator(mask,mask->GetLargestPossibleRegion());
 
  while(!imageIterator.IsAtEnd())
  {
    // Set the current pixel to white
    imageIterator.Set(255);
 
    ++imageIterator;
  }
}