#include "itkGaussianDistribution.h"

int main(int, char* [] )
{
  itk::Statistics::GaussianDistribution::Pointer gaussian = itk::Statistics::GaussianDistribution::New();
  gaussian->SetMean(2.0);
  gaussian->SetVariance(1.0);
  std::cout << gaussian->EvaluatePDF(2.1) << std::endl;
  return EXIT_SUCCESS;
}