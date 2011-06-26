#include "itkNumericTraits.h"
#include "itkImage.h"
#include "itkCovariantVector.h"

typedef itk::CovariantVector<double, 1> VectorType;

typedef itk::Image<VectorType, 2> ImageType;

void Infinity();

int main(int, char* [] )
{
  std::cout << "Int Max: " << itk::NumericTraits< int>::max() << std::endl;

  //Infinity();
  /*
  std::cout << "Min: " << itk::NumericTraits< float >::min() << std::endl;
  std::cout << "Max: " << itk::NumericTraits< float >::max() << std::endl;
  std::cout << "Zero: " << itk::NumericTraits< float >::Zero << std::endl;
  std::cout << "Zero: " << itk::NumericTraits< float >::ZeroValue() << std::endl;
  std::cout << "Is -1 negative? " << itk::NumericTraits< float >::IsNegative(-1) << std::endl;
  std::cout << "Is 1 negative? " << itk::NumericTraits< float >::IsNegative(1) << std::endl;
  std::cout << "One: " << itk::NumericTraits< float >::One << std::endl;
  std::cout << "Epsilon: " << itk::NumericTraits< float >::epsilon() << std::endl;
  */

  //std::cout << itk::NumericTraits< ImageType::PixelType >::Zero << std::endl;

  return EXIT_SUCCESS;
}

void Infinity()
{
  std::cout << "Float Infinity: " << itk::NumericTraits< float >::infinity() << std::endl;

  if(0 == itk::NumericTraits< float >::infinity())
    {
    std::cout << " 0 == inf for float!" << std::endl;
    }
  else
    {
    std::cout << "Good float" << std::endl;
    }

  std::cout << "Int Infinity: " << itk::NumericTraits< int >::infinity() << std::endl;

  if(0 == itk::NumericTraits< int >::infinity())
    {
    std::cout << " 0 == inf for int!" << std::endl;
    }
  else
    {
    std::cout << "Good int" << std::endl;
    }
}