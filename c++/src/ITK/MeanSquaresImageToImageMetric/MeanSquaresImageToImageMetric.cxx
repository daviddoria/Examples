#include <itkImage.h>
#include <itkTranslationTransform.h>
#include <itkImageFileReader.h>
#include <itkImageRegistrationMethod.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkRegularStepGradientDescentOptimizer.h>
#include <itkMeanSquaresImageToImageMetric.h>
#include <itkRegionOfInterestImageFilter.h>
#include <itkCastImageFilter.h>
#include <itkRescaleIntensityImageFilter.h>
#include <itkImageRegionIterator.h>

#include <itkImageToVTKImageFilter.h>

#include "vtkImageViewer.h"
#include "itkImageFileWriter.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkSmartPointer.h"
#include "vtkImageActor.h"
#include "vtkInteractorStyleImage.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkProperty.h"

#include <iostream>
#include <string>

typedef itk::Image<float, 2> FloatImageType;

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

  // Perform registration
  typedef itk::MeanSquaresImageToImageMetric<FloatImageType, FloatImageType> MetricType; // <TFixedImage, TMovingImage>
  typedef itk::TranslationTransform<double, 2> TransformType; // Compiler error if you change double to float??!
  typedef itk::RegularStepGradientDescentOptimizer OptimizerType;
  typedef itk::LinearInterpolateImageFunction<FloatImageType, double> InterpolatorType; // Compiler error if you change double to float??!
  typedef itk::ImageRegistrationMethod<FloatImageType, FloatImageType> RegistrationType;

  MetricType::Pointer metric = MetricType::New();
  TransformType::Pointer transform = TransformType::New();
  OptimizerType::Pointer optimizer = OptimizerType::New();
  InterpolatorType::Pointer interpolator = InterpolatorType::New();
  
  RegistrationType::Pointer registration = RegistrationType::New();
  registration->SetMetric(metric);
  registration->SetOptimizer(optimizer);
  registration->SetInterpolator(interpolator);
  registration->SetTransform(transform);
  registration->SetFixedImage(reader->GetOutput());
  registration->SetMovingImage(extractFilter->GetOutput());
  registration->SetFixedImageRegion(reader->GetOutput()->GetLargestPossibleRegion());

  RegistrationType::ParametersType initialParameters(transform->GetNumberOfParameters());
  initialParameters[0] = 0;
  initialParameters[1] = 0;

  registration->SetInitialTransformParameters(initialParameters);

  registration->Update();

  RegistrationType::ParametersType finalParameters = registration->GetLastTransformParameters();
  std::cout << "Final parameters: " << finalParameters << std::endl;

  vtkSmartPointer<vtkSphereSource> sphereSource =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(finalParameters[0], finalParameters[1], 0);
  sphereSource->SetRadius(10);
  sphereSource->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkSmartPointer<vtkActor> sphereActor =
    vtkSmartPointer<vtkActor>::New();
  sphereActor->SetMapper(mapper);
  sphereActor->GetProperty()->SetColor(1,0,0);

  // Visualize
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(900, 300);

  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  double leftViewport[4] = {0.0, 0.0, 0.5, 1.0};
  double rightViewport[4] = {0.5, 0.0, 1.0, 1.0};

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
  leftRenderer->AddActor(sphereActor); // Display the location of the best position of the moving image in the fixed image
  
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
