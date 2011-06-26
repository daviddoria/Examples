#include <itkNeighborhoodOperator.h>

#include <itkDerivativeOperator.h>
#include <itkForwardDifferenceOperator.h>
#include <itkGaussianDerivativeOperator.h>
#include <itkGaussianOperator.h>
#include <itkImageKernelOperator.h>
#include <itkLaplacianOperator.h>
#include <itkSobelOperator.h>
#include <itkAnnulusOperator.h>
#include <itkBackwardDifferenceOperator.h>

#include <vector>

int main(int, char*[])
{
  typedef itk::DerivativeOperator<float, 2> DerivativeOperatorType;
  typedef itk::ForwardDifferenceOperator<float, 2> ForwardDifferenceOperatorType;
  typedef itk::GaussianDerivativeOperator<float, 2> GaussianDerivativeOperatorType;
  typedef itk::GaussianOperator<float, 2> GaussianOperatorType;
  typedef itk::ImageKernelOperator<float, 2> ImageKernelOperatorType;
  typedef itk::LaplacianOperator<float, 2> LaplacianOperatorType;
  typedef itk::SobelOperator<float, 2> SobelOperatorType;
  typedef itk::AnnulusOperator<float, 2> AnnulusOperatorType;
  typedef itk::BackwardDifferenceOperator<float, 2> BackwardDifferenceOperatorType;

  std::vector<itk::NeighborhoodOperator<float, 2>*> operators;
  operators.push_back(new DerivativeOperatorType);
  operators.push_back(new ForwardDifferenceOperatorType);
  operators.push_back(new GaussianDerivativeOperatorType);
  operators.push_back(new GaussianOperatorType);
  operators.push_back(new ImageKernelOperatorType);
  operators.push_back(new LaplacianOperatorType);
  operators.push_back(new SobelOperatorType);
  operators.push_back(new AnnulusOperatorType);
  operators.push_back(new BackwardDifferenceOperatorType);

  itk::Size<2> radius;
  radius.Fill(1);

  for(unsigned int operatorId = 0; operatorId < operators.size(); operatorId++)
    {
    operators[operatorId]->SetDirection(0); // Create the operator for the X axis derivative
    operators[operatorId]->CreateToRadius(radius);
    //std::cout << *(operators[operatorId]) << std::endl;
    //operators[operatorId]->Print(std::cout);
    //std::cout << operators[operatorId]->GetNameOfClass() << std::endl;

    for(int i = -operators[operatorId]->GetSize()[0]/2; i <= operators[operatorId]->GetSize()[0]/2; i++)
      {
      for(int j = -operators[operatorId]->GetSize()[1]/2; j <= operators[operatorId]->GetSize()[1]/2; j++)
        {
        itk::Offset<2> offset;
        offset[0] = i;
        offset[1] = j;

        unsigned int neighborhoodIndex = operators[operatorId]->GetNeighborhoodIndex(offset);
        std::cout << operators[operatorId]->GetElement(neighborhoodIndex) << " ";
        }
      std::cout << std::endl;
      }
    }
  return EXIT_SUCCESS;
}
