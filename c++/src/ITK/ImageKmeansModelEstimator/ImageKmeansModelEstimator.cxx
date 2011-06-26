#include "itkImage.h"
#include "itkListSample.h"
#include "itkVector.h"
#include "itkImageKmeansModelEstimator.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkImageToListSampleAdaptor.h"
#include "itkDistanceToCentroidMembershipFunction.h"
#include "itkSampleClassifierFilter.h"
#include "itkMinimumDecisionRule2.h"

#include "vtkSmartPointer.h"
#include "vtkImageActor.h"
#include "vtkImageData.h"
#include "vtkInteractorStyleImage.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"

typedef itk::Vector<unsigned char,3> MeasurementVectorType;
typedef itk::Image<MeasurementVectorType,2> ColorImageType;
typedef itk::Image<unsigned char,2> GrayscaleImageType;

void CreateImage(ColorImageType::Pointer image);
void ITKImagetoVTKImageColor(ColorImageType::Pointer image, vtkImageData* outputImage);
void ITKImagetoVTKImageGrayscale(GrayscaleImageType::Pointer image, vtkImageData* outputImage);
void CreateBlankImage(GrayscaleImageType::Pointer image, ColorImageType::Pointer inputImage);

int main(int, char* [] )
{
  // Create a demo image
  ColorImageType::Pointer image = ColorImageType::New();
  CreateImage(image);

  // Compute pixel clusters using KMeans
  typedef itk::Statistics::DistanceToCentroidMembershipFunction< itk::Vector<unsigned char,3> >  MembershipFunctionType ;
  typedef MembershipFunctionType::Pointer MembershipFunctionPointer ;
  typedef std::vector< MembershipFunctionPointer >  MembershipFunctionPointerVector;

  typedef itk::ImageKmeansModelEstimator<ColorImageType, MembershipFunctionType>  ImageKmeansModelEstimatorType;

  ImageKmeansModelEstimatorType::Pointer
    kmeansEstimator = ImageKmeansModelEstimatorType::New();
  kmeansEstimator->SetInputImage(image);
  kmeansEstimator->SetNumberOfModels(3);
  kmeansEstimator->SetThreshold(0.01 );
  kmeansEstimator->SetOffsetAdd( 0.01 );
  kmeansEstimator->SetOffsetMultiply( 0.01 );
  kmeansEstimator->SetMaxSplitAttempts( 10 );
  kmeansEstimator->Update();

  // Classify each pixel
  typedef itk::Statistics::ListSample< MeasurementVectorType > SampleType ;
  typedef itk::Statistics::SampleClassifierFilter< SampleType > ClassifierType;
  ClassifierType::Pointer classifier = ClassifierType::New();

  typedef itk::Statistics::MinimumDecisionRule2 DecisionRuleType;
  DecisionRuleType::Pointer decisionRule = DecisionRuleType::New();
  
  classifier->SetDecisionRule(decisionRule);
  classifier->SetNumberOfClasses(3);

  typedef ClassifierType::ClassLabelVectorObjectType               ClassLabelVectorObjectType;
  typedef ClassifierType::ClassLabelVectorType                     ClassLabelVectorType;
  typedef ClassifierType::MembershipFunctionVectorObjectType       MembershipFunctionVectorObjectType;
  typedef ClassifierType::MembershipFunctionVectorType             MembershipFunctionVectorType;

  // Setup membership functions
  MembershipFunctionPointerVector kmeansMembershipFunctions =
    kmeansEstimator->GetMembershipFunctions();

  MembershipFunctionVectorObjectType::Pointer  membershipFunctionsVectorObject = MembershipFunctionVectorObjectType::New();
  classifier->SetMembershipFunctions(membershipFunctionsVectorObject);

  MembershipFunctionVectorType &  membershipFunctionsVector = membershipFunctionsVectorObject->Get();

  for(unsigned int i = 0; i < kmeansMembershipFunctions.size(); i++)
    {
    membershipFunctionsVector.push_back(kmeansMembershipFunctions[i].GetPointer());
    }

  // Setup class labels
  ClassLabelVectorObjectType::Pointer  classLabelsObject = ClassLabelVectorObjectType::New();
  classifier->SetClassLabels( classLabelsObject );

  ClassLabelVectorType &  classLabelsVector = classLabelsObject->Get();
  classLabelsVector.push_back( 50 );
  classLabelsVector.push_back( 150 );
  classLabelsVector.push_back( 250 );

  // Perform the classification
  typedef itk::Statistics::ImageToListSampleAdaptor< ColorImageType > SampleAdaptorType;
  SampleAdaptorType::Pointer sample = SampleAdaptorType::New();
  sample->SetImage(image);

  classifier->SetInput(sample);
  classifier->Update();
  
  // Prepare the output image
  GrayscaleImageType::Pointer outputImage = GrayscaleImageType::New();
  CreateBlankImage(outputImage, image);

  // Setup the membership iterator
  const ClassifierType::MembershipSampleType* membershipSample = classifier->GetOutput();
  ClassifierType::MembershipSampleType::ConstIterator membershipIterator = membershipSample->Begin();

  // Setup the output image iterator - this is automatically synchronized with the membership iterator since the sample is an adaptor
  itk::ImageRegionIteratorWithIndex<GrayscaleImageType> outputIterator(outputImage,outputImage->GetLargestPossibleRegion());
  outputIterator.GoToBegin();
  
  while(membershipIterator != membershipSample->End())
    {
    int classLabel = membershipIterator.GetClassLabel();
    std::cout << "Class label: " << classLabel << std::endl;
    outputIterator.Set(static_cast<unsigned char>(classLabel));
    ++membershipIterator;
    ++outputIterator;
    }

  // Visualize
  // Original image
  vtkSmartPointer<vtkImageData> originalVTKImage =
    vtkSmartPointer<vtkImageData>::New();
  ITKImagetoVTKImageColor(image, originalVTKImage);

  vtkSmartPointer<vtkImageActor> originalActor =
    vtkSmartPointer<vtkImageActor>::New();
  originalActor->SetInput(originalVTKImage);

  // Kmeans image
  vtkSmartPointer<vtkImageData> kmeansVTKImage =
    vtkSmartPointer<vtkImageData>::New();
  ITKImagetoVTKImageGrayscale(outputImage, kmeansVTKImage);

  vtkSmartPointer<vtkImageActor> kmeansActor =
    vtkSmartPointer<vtkImageActor>::New();
  kmeansActor->SetInput(kmeansVTKImage);
  
  // There will be one render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(600, 300);

  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double leftViewport[4] = {0.0, 0.0, 0.5, 1.0};
  double rightViewport[4] = {0.5, 0.0, 1.0, 1.0};
  
  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  vtkSmartPointer<vtkRenderer> leftRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(leftRenderer);
  leftRenderer->SetViewport(leftViewport);
  leftRenderer->SetBackground(.6, .5, .4);

  leftRenderer->AddActor(originalActor);
  leftRenderer->ResetCamera();

  vtkSmartPointer<vtkRenderer> rightRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(rightRenderer);
  rightRenderer->SetViewport(rightViewport);
  rightRenderer->SetBackground(.7, .4, .4);

  rightRenderer->AddActor(kmeansActor);
  rightRenderer->ResetCamera();

  renderWindow->Render();

  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();
  interactor->SetInteractorStyle(style);

  interactor->Start();

  return EXIT_SUCCESS;
}

void CreateImage(ColorImageType::Pointer image)
{
  // Create a black image with a red square and a green square
  ColorImageType::RegionType region;
  ColorImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;

  ColorImageType::SizeType size;
  size[0] = 200;
  size[1] = 300;

  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ColorImageType> imageIterator(image,region);

  itk::Vector<unsigned char, 3> redPixel;
  redPixel[0] = 255;
  redPixel[1] = 0;
  redPixel[2] = 0;

  itk::Vector<unsigned char, 3> greenPixel;
  greenPixel[0] = 0;
  greenPixel[1] = 255;
  greenPixel[2] = 0;
  
  itk::Vector<unsigned char, 3> blackPixel;
  blackPixel[0] = 0;
  blackPixel[1] = 0;
  blackPixel[2] = 0;
  
  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] > 100 &&
      imageIterator.GetIndex()[0] < 150 &&
      imageIterator.GetIndex()[1] > 100 &&
      imageIterator.GetIndex()[1] < 150)
      {
      imageIterator.Set(redPixel);
      }
    else if(imageIterator.GetIndex()[0] > 50 &&
      imageIterator.GetIndex()[0] < 70 &&
      imageIterator.GetIndex()[1] > 50 &&
      imageIterator.GetIndex()[1] < 70)
      {
      imageIterator.Set(greenPixel);
      }
    else
      {
      imageIterator.Set(blackPixel);
      }

    ++imageIterator;
  }
}


void ITKImagetoVTKImageColor(ColorImageType::Pointer image, vtkImageData* outputImage)
{
  outputImage->SetNumberOfScalarComponents(3);
  outputImage->SetScalarTypeToUnsignedChar();

  outputImage->SetDimensions(image->GetLargestPossibleRegion().GetSize()[0],
                             image->GetLargestPossibleRegion().GetSize()[1],
                             1);

  outputImage->AllocateScalars();

  int* dims = outputImage->GetDimensions();

  for (int y = 0; y < dims[1]; y++)
    {
    for (int x = 0; x < dims[0]; x++)
      {
      unsigned char* pixel = static_cast<unsigned char*>(outputImage->GetScalarPointer(x,y,0));
      ColorImageType::IndexType index;

      index[0] = x;
      index[1] = y;
      pixel[0] = image->GetPixel(index)[0];
      pixel[1] = image->GetPixel(index)[1];
      pixel[2] = image->GetPixel(index)[2];
      }
    }
}


void ITKImagetoVTKImageGrayscale(GrayscaleImageType::Pointer image, vtkImageData* outputImage)
{
  outputImage->SetNumberOfScalarComponents(1);
  outputImage->SetScalarTypeToUnsignedChar();

  outputImage->SetDimensions(image->GetLargestPossibleRegion().GetSize()[0],
                             image->GetLargestPossibleRegion().GetSize()[1],
                             1);

  outputImage->AllocateScalars();

  int* dims = outputImage->GetDimensions();

  for (int y = 0; y < dims[1]; y++)
    {
    for (int x = 0; x < dims[0]; x++)
      {
      unsigned char* pixel = static_cast<unsigned char*>(outputImage->GetScalarPointer(x,y,0));
      GrayscaleImageType::IndexType index;

      index[0] = x;
      index[1] = y;
      pixel[0] = image->GetPixel(index);
      }
    }
}

void CreateBlankImage(GrayscaleImageType::Pointer image, ColorImageType::Pointer inputImage)
{
  image->SetRegions(inputImage->GetLargestPossibleRegion());
  image->Allocate();

  itk::ImageRegionIterator<GrayscaleImageType> imageIterator(image,image->GetLargestPossibleRegion());

  while(!imageIterator.IsAtEnd())
    {

    imageIterator.Set(0);

    ++imageIterator;
    }
}