#include "itkNumericTraits.h"
#include "itkImage.h"
#include "itkCovariantVector.h"

struct MyType
{
  double a;
};


namespace itk
{
template<>
class NumericTraits< MyType >
{
private:
public:

  typedef MyType Self;
  
  static const Self ZeroValue()
  {
    MyType temp;
    temp.a = 0;
    return temp;
  }
};
} // end namespace itk


int main(int, char* [] )
{
  MyType test = itk::NumericTraits<MyType>::ZeroValue();
  return EXIT_SUCCESS;
}
