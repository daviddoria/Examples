#include "itkImageAdaptor.h"
#include "itkImageRegionIterator.h"

typedef itk::Image<float, 2> ScalarImageType;
typedef itk::Image<itk::CovariantVector< float, 3>, 2> VectorImageType;


void CreateImage(VectorImageType::Pointer image);

class VectorPixelAccessor
{
public:
  typedef itk::CovariantVector<float,3>   InternalType;
  typedef                      float      ExternalType;

  void operator=( const VectorPixelAccessor & vpa )
    {
      m_Index = vpa.m_Index;
    }
  ExternalType Get( const InternalType & input ) const
    {
    return static_cast<ExternalType>( input[ m_Index ] );
    }
  void SetIndex( unsigned int index )
    {
    m_Index = index;
    }
private:
  unsigned int m_Index;
};

int main(int, char *[])
{
  VectorImageType::Pointer image = VectorImageType::New();
  CreateImage(image);

  itk::Index<2> index;
  index.Fill(0);

  std::cout << image->GetPixel(index) << std::endl;

  typedef itk::ImageAdaptor<  VectorImageType,
                              VectorPixelAccessor > ImageAdaptorType;

  ImageAdaptorType::Pointer adaptor = ImageAdaptorType::New();

  VectorPixelAccessor  accessor;
  accessor.SetIndex(0);
  adaptor->SetPixelAccessor( accessor );
  adaptor->SetImage(image);

  std::cout << adaptor->GetPixel(index) << std::endl;

  return EXIT_SUCCESS;
}

void CreateImage(VectorImageType::Pointer image)
{
  VectorImageType::IndexType start;
  start.Fill(0);

  VectorImageType::SizeType size;
  size.Fill(2);

  VectorImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<VectorImageType> imageIterator(image,image->GetLargestPossibleRegion());
  itk::CovariantVector<float, 3> vec;
  vec[0] = 1;
  vec[1] = 2;
  vec[2] = 3;

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(vec);

    ++imageIterator;
    }
}