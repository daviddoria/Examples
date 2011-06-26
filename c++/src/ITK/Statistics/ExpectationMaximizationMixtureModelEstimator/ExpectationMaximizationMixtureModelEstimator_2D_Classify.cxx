#include "itkVector.h"
#include "itkListSample.h"
#include "itkGaussianMixtureModelComponent.h"
#include "itkExpectationMaximizationMixtureModelEstimator.h"
#include "itkNormalVariateGenerator.h"
#include "itkSampleClassifierFilter.h"
#include "itkMaximumDecisionRule2.h"

int main(int, char*[])
{
  unsigned int numberOfClasses = 2;
  typedef itk::Vector< double, 2 > MeasurementVectorType;
  typedef itk::Statistics::ListSample< MeasurementVectorType > SampleType;
  SampleType::Pointer sample = SampleType::New();

  typedef itk::Statistics::NormalVariateGenerator NormalGeneratorType;
  NormalGeneratorType::Pointer normalGenerator = NormalGeneratorType::New();

  // Create the first set of 2D Gaussian samples
  normalGenerator->Initialize( 101 );

  MeasurementVectorType mv;
  double mean = 100;
  double standardDeviation = 30;
  for ( unsigned int i = 0 ; i < 10 ; ++i )
    {
    mv[0] = ( normalGenerator->GetVariate() * standardDeviation ) + mean;
    mv[1] = ( normalGenerator->GetVariate() * standardDeviation ) + mean;
    sample->PushBack( mv );
    }

  // Create the second set of 2D Gaussian samples
  normalGenerator->Initialize( 3024 );
  mean = 200;
  standardDeviation = 30;
  for ( unsigned int i = 0 ; i < 10 ; ++i )
    {
    mv[0] = ( normalGenerator->GetVariate() * standardDeviation ) + mean;
    mv[1] = ( normalGenerator->GetVariate() * standardDeviation ) + mean;
    sample->PushBack( mv );
    }

  typedef itk::Array< double > ParametersType;
  ParametersType params( 6 );

  // Create the first set (for the first cluster/model) of initial parameters
  std::vector< ParametersType > initialParameters( numberOfClasses );
  params[0] = 110.0; // mean of dimension 1
  params[1] = 115.0; // mean of dimension 2
  params[2] = 800.0; // covariance(0,0)
  params[3] = 0; // covariance(0,1)
  params[4] = 0; // covariance(1,0)
  params[5] = 805.0; // covariance(1,1)
  initialParameters[0] = params;

  // Create the second set (for the second cluster/model) of initial parameters
  params[0] = 210.0; // mean of dimension 1
  params[1] = 215.0; // mean of dimension 2
  params[2] = 850.0; // covariance(0,0)
  params[3] = 0; // covariance(0,1)
  params[4] = 0; // covariance(1,0)
  params[5] = 855.0; // covariance(1,1)
  initialParameters[1] = params;

  typedef itk::Statistics::GaussianMixtureModelComponent< SampleType >
    ComponentType;

  // Create the components
  std::vector< ComponentType::Pointer > components;
  for ( unsigned int i = 0 ; i < numberOfClasses ; i++ )
    {
    components.push_back( ComponentType::New() );
    (components[i])->SetSample( sample );
    (components[i])->SetParameters( initialParameters[i] );
    }

  typedef itk::Statistics::ExpectationMaximizationMixtureModelEstimator<
                           SampleType > EstimatorType;
  EstimatorType::Pointer estimator = EstimatorType::New();

  estimator->SetSample( sample );
  estimator->SetMaximumIteration( 200 );

  itk::Array< double > initialProportions(numberOfClasses);
  initialProportions[0] = 0.5;
  initialProportions[1] = 0.5;

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
  typedef itk::Statistics::SampleClassifierFilter< SampleType > FilterType;
  
  typedef itk::Statistics::MaximumDecisionRule2  DecisionRuleType;
  DecisionRuleType::Pointer    decisionRule = DecisionRuleType::New();

  typedef FilterType::ClassLabelVectorObjectType               ClassLabelVectorObjectType;
  typedef FilterType::ClassLabelVectorType                     ClassLabelVectorType;

  ClassLabelVectorObjectType::Pointer  classLabelsObject = ClassLabelVectorObjectType::New();
  ClassLabelVectorType & classLabelVector  = classLabelsObject->Get();

  typedef FilterType::ClassLabelType        ClassLabelType;

  ClassLabelType  class1 = 0;
  classLabelVector.push_back( class1 );

  ClassLabelType  class2 = 1;
  classLabelVector.push_back( class2 );
  
  FilterType::Pointer sampleClassifierFilter = FilterType::New();
  sampleClassifierFilter->SetInput( sample );
  sampleClassifierFilter->SetNumberOfClasses( numberOfClasses );
  sampleClassifierFilter->SetClassLabels( classLabelsObject );
  sampleClassifierFilter->SetDecisionRule( decisionRule );
  sampleClassifierFilter->SetMembershipFunctions( estimator->GetOutput() );
  sampleClassifierFilter->Update();

  const FilterType::MembershipSampleType* membershipSample = sampleClassifierFilter->GetOutput();
  FilterType::MembershipSampleType::ConstIterator iter = membershipSample->Begin();

  while ( iter != membershipSample->End() )
    {
    std::cout << iter.GetMeasurementVector() << " : " << iter.GetClassLabel() << std::endl;
    ++iter;
    }

  return EXIT_SUCCESS;
}