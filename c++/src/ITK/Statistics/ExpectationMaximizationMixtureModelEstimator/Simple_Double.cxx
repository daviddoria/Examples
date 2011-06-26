#include "itkVector.h"
#include "itkListSample.h"
#include "itkGaussianMixtureModelComponent.h"
#include "itkExpectationMaximizationMixtureModelEstimator.h"
#include "itkNormalVariateGenerator.h"
#include "itkSampleClassifierFilter.h"
#include "itkMaximumDecisionRule2.h"
#include "itkImageToListSampleFilter.h"
#include "itkCovariantVector.h"
#include "itkImageRegionIterator.h"
#include "itkImageFileReader.h"

typedef itk::CovariantVector<double, 3> PixelType;
typedef itk::Image<PixelType, 2>  ImageType;

void CreateImage(ImageType::Pointer image);

int main(int argc, char*argv[])
{
  ImageType::Pointer image = ImageType::New();

  CreateImage(image);
  typedef itk::Statistics::ImageToListSampleFilter<ImageType> ImageToListSampleFilterType;
  ImageToListSampleFilterType::Pointer imageToListSampleFilter = ImageToListSampleFilterType::New();
  imageToListSampleFilter->SetInput(image);
  imageToListSampleFilter->Update();

  unsigned int numberOfClasses = 3;

  typedef itk::Statistics::NormalVariateGenerator NormalGeneratorType;
  NormalGeneratorType::Pointer normalGenerator = NormalGeneratorType::New();

  typedef itk::Array< double > ParametersType;
  ParametersType params( numberOfClasses + numberOfClasses*numberOfClasses ); // 3 for means and 9 for 3x3 covariance

  // Create the first set (for the first cluster/model) of initial parameters
  std::vector< ParametersType > initialParameters( numberOfClasses );
  for(unsigned int i = 0; i < 3; i++)
    {
    params[i] = 5.0; // mean of dimension i
    }
  for(unsigned int i = 3; i < 12; i++)
    {
    params[i] = 5.0; // covariance
    }

  initialParameters[0] = params;

  // Create the second set (for the second cluster/model) of initial parameters
  params[0] = 210.0;
  params[1] = 5.0;
  params[2] = 5.0;
  for(unsigned int i = 3; i < 12; i++)
    {
    params[i] = 5.0; // covariance
    }
  initialParameters[1] = params;

  // Create the third set (for the third cluster/model) of initial parameters
  params[0] = 5.0;
  params[1] = 210.0;
  params[2] = 5.0;
  for(unsigned int i = 3; i < 12; i++)
    {
    params[i] = 5.0; // covariance
    }
  initialParameters[2] = params;

  std::cout << "Initial parameters: " << std::endl;
  for ( unsigned int i = 0 ; i < numberOfClasses ; i++ )
    {
    std::cout << initialParameters[i] << std::endl;
    }

  typedef itk::Statistics::GaussianMixtureModelComponent< ImageToListSampleFilterType::ListSampleType >
    ComponentType;

  // Create the components
  std::vector< ComponentType::Pointer > components;
  for ( unsigned int i = 0 ; i < numberOfClasses ; i++ )
    {
    components.push_back( ComponentType::New() );
    (components[i])->SetSample( imageToListSampleFilter->GetOutput() );
    (components[i])->SetParameters( initialParameters[i] );
    }

  typedef itk::Statistics::ExpectationMaximizationMixtureModelEstimator<
                           ImageToListSampleFilterType::ListSampleType > EstimatorType;
  EstimatorType::Pointer estimator = EstimatorType::New();

  estimator->SetSample( imageToListSampleFilter->GetOutput() );
  estimator->SetMaximumIteration( 200 );

  itk::Array< double > initialProportions(numberOfClasses);
  initialProportions[0] = 0.33;
  initialProportions[1] = 0.33;
  initialProportions[2] = 0.33;

  estimator->SetInitialProportions( initialProportions );

  for ( unsigned int i = 0 ; i < numberOfClasses ; i++)
    {
    estimator->AddComponent( (ComponentType::Superclass*)
                             (components[i]).GetPointer() );
    }

  estimator->Update();

  // Output the results
  for ( unsigned int i = 0 ; i < numberOfClasses ; i++ )
    {
    std::cout << "Cluster[" << i << "]" << std::endl;
    std::cout << "    Parameters:" << std::endl;
    std::cout << "         " << (components[i])->GetFullParameters()
              << std::endl;
    std::cout << "    Proportion: ";
    // Outputs: // mean of dimension 1, mean of dimension 2, covariance(0,0), covariance(0,1), covariance(1,0), covariance(1,1)
    std::cout << "         " << estimator->GetProportions()[i] << std::endl;
    }

  // Display the membership of each sample
  typedef itk::Statistics::SampleClassifierFilter< ImageToListSampleFilterType::ListSampleType > FilterType;

  typedef itk::Statistics::MaximumDecisionRule2  DecisionRuleType;
  DecisionRuleType::Pointer    decisionRule = DecisionRuleType::New();

  typedef FilterType::ClassLabelVectorObjectType               ClassLabelVectorObjectType;
  typedef FilterType::ClassLabelVectorType                     ClassLabelVectorType;

  ClassLabelVectorObjectType::Pointer  classLabelsObject = ClassLabelVectorObjectType::New();
  ClassLabelVectorType & classLabelVector  = classLabelsObject->Get();

  typedef FilterType::ClassLabelType        ClassLabelType;

  ClassLabelType  class0 = 0;
  classLabelVector.push_back( class0 );

  ClassLabelType  class1 = 1;
  classLabelVector.push_back( class1 );

  ClassLabelType  class2 = 2;
  classLabelVector.push_back( class2 );

  FilterType::Pointer sampleClassifierFilter = FilterType::New();
  sampleClassifierFilter->SetInput( imageToListSampleFilter->GetOutput() );
  sampleClassifierFilter->SetNumberOfClasses( numberOfClasses );
  sampleClassifierFilter->SetClassLabels( classLabelsObject );
  sampleClassifierFilter->SetDecisionRule( decisionRule );
  sampleClassifierFilter->SetMembershipFunctions( estimator->GetOutput() );
  sampleClassifierFilter->Update();

  const FilterType::MembershipSampleType* membershipSample = sampleClassifierFilter->GetOutput();
  FilterType::MembershipSampleType::ConstIterator iter = membershipSample->Begin();

  /*
  while ( iter != membershipSample->End() )
    {
    std::cout << (int)iter.GetMeasurementVector()[0] << " " << (int)iter.GetMeasurementVector()[1] << " " << (int)iter.GetMeasurementVector()[2]
              << " : " << iter.GetClassLabel() << std::endl;
    ++iter;
    }
  */
  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create an image
  ImageType::RegionType region;
  ImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;

  ImageType::SizeType size;
  size[0] = 10;
  size[1] = 10;

  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  // Make a red and a green square
  itk::CovariantVector<double, 3> green;
  green[0] = 0;
  green[1] = 255;
  green[2] = 0;

  itk::CovariantVector<double, 3> red;
  red[0] = 255;
  red[1] = 0;
  red[2] = 0;

  itk::CovariantVector<double, 3> black;
  black[0] = 0;
  black[1] = 0;
  black[2] = 0;

  itk::ImageRegionIterator<ImageType> imageIterator(image,region);
  imageIterator.GoToBegin();

  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] > 2 && imageIterator.GetIndex()[0] < 5 &&
      imageIterator.GetIndex()[1] > 2 && imageIterator.GetIndex()[1] < 5)
      {
      imageIterator.Set(green);
      }
    else if(imageIterator.GetIndex()[0] > 6 && imageIterator.GetIndex()[0] < 9 &&
      imageIterator.GetIndex()[1] > 6 && imageIterator.GetIndex()[1] < 9)
      {
      imageIterator.Set(red);
      }
    else
      {
      imageIterator.Set(black);
      }
    ++imageIterator;
    }

}