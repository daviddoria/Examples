#include "itkVector.h"
#include "itkListSample.h"
#include "itkGaussianMixtureModelComponent.h"
#include "itkExpectationMaximizationMixtureModelEstimator.h"
#include "itkNormalVariateGenerator.h"

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
  for ( unsigned int i = 0 ; i < 100 ; ++i )
    {
    mv[0] = ( normalGenerator->GetVariate() * standardDeviation ) + mean;
    mv[1] = ( normalGenerator->GetVariate() * standardDeviation ) + mean;
    sample->PushBack( mv );
    }

  // Create the second set of 2D Gaussian samples
  normalGenerator->Initialize( 3024 );
  mean = 200;
  standardDeviation = 30;
  for ( unsigned int i = 0 ; i < 100 ; ++i )
    {
    mv[0] = ( normalGenerator->GetVariate() * standardDeviation ) + mean;
    mv[1] = ( normalGenerator->GetVariate() * standardDeviation ) + mean;
    sample->PushBack( mv );
    }

  typedef itk::Array< double > ParametersType;
  ParametersType params( 6 );

  // Create the first set of initial parameters
  std::vector< ParametersType > initialParameters( numberOfClasses );
  params[0] = 110.0; // mean of dimension 1
  params[1] = 115.0; // mean of dimension 2
  params[2] = 800.0; // covariance(0,0)
  params[3] = 0; // covariance(0,1)
  params[4] = 0; // covariance(1,0)
  params[5] = 805.0; // covariance(1,1)
  initialParameters[0] = params;

  // Create the second set of initial parameters
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
    std::cout << "         " << estimator->GetProportions()[i] << std::endl;
    }

  return EXIT_SUCCESS;
}