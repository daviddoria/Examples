#include "itkVector.h"
#include "itkListSample.h"
#include "itkGaussianMixtureModelComponent.h"
#include "itkExpectationMaximizationMixtureModelEstimator.h"
#include "itkNormalVariateGenerator.h"

int main(int, char*[])
{
  unsigned int numberOfClasses = 2;
  typedef itk::Vector< double, 1 > MeasurementVectorType;
  typedef itk::Statistics::ListSample< MeasurementVectorType > SampleType;
  SampleType::Pointer sample = SampleType::New();

  typedef itk::Statistics::NormalVariateGenerator NormalGeneratorType;
  NormalGeneratorType::Pointer normalGenerator = NormalGeneratorType::New();

  normalGenerator->Initialize(101);

  MeasurementVectorType mv;
  double mean = 100;
  double standardDeviation = 30;
  for(unsigned int i = 0; i < 10; ++i )
    {
    mv[0] = ( normalGenerator->GetVariate() * standardDeviation ) + mean;
    std::cout << "m[" << i << "] = " << mv[0] << std::endl;
    sample->PushBack( mv );
    }

  normalGenerator->Initialize(3024);
  mean = 200;
  standardDeviation = 30;
  for(unsigned int i = 0; i < 10; ++i )
    {
    mv[0] = ( normalGenerator->GetVariate() * standardDeviation ) + mean;
    std::cout << "m[" << i << "] = " << mv[0] << std::endl;
    sample->PushBack( mv );
    }

  typedef itk::Array< double > ParametersType;
  ParametersType params1( 2 );

  std::vector< ParametersType > initialParameters( numberOfClasses );
  params1[0] = 110.0;
  params1[1] = 900.0; // Estimated variance (standard deviation^2)
  initialParameters[0] = params1;

  ParametersType params2( 2 );
  params2[0] = 210.0;
  params2[1] = 900.0;  // Estimated variance (standard deviation^2)
  initialParameters[1] = params2;

  typedef itk::Statistics::GaussianMixtureModelComponent< SampleType > ComponentType;

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

  estimator->SetSample(sample);
  estimator->SetMaximumIteration(500);

  itk::Array< double > initialProportions(numberOfClasses);
  initialProportions[0] = 0.5;
  initialProportions[1] = 0.5;

  estimator->SetInitialProportions( initialProportions );

  for(unsigned int i = 0; i < numberOfClasses; i++)
    {
    estimator->AddComponent( (ComponentType::Superclass*)
                             (components[i]).GetPointer() );
    }

  estimator->Update();

  for(unsigned int i = 0; i < numberOfClasses; i++ )
    {
    std::cout << "Cluster[" << i << "]" << std::endl;
    std::cout << "    Parameters:" << std::endl;
    std::cout << "         " << (components[i])->GetFullParameters()
              << std::endl;
    std::cout << "    Proportion: ";
    std::cout << "         " << estimator->GetProportions()[i] << std::endl; // This outputs the mean and the variance (make sure you square the standard deviation used to create the samples when checking that this estimate is sane).
    }

  return EXIT_SUCCESS;
}