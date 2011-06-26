#include <itkAbsImageFilter.h>
#include <itkImage.h>

typedef itk::Image<double, 2> ImageType;
void CreateImage(ImageType::Pointer image);

int main(int, char*[])
{

  typedef itk::AbsImageFilter<ImageType, ImageType> FilterType;
  FilterType::Pointer filter = FilterType::New();
  FilterType::Pointer filter2 = dynamic_cast<FilterType*>(filter->CreateAnother().GetPointer());

  ImageType::Pointer image = ImageType::New();
  CreateImage(image);
  
  filter2->SetInput(image);
  filter2->Update();

  itk::Index<2> index;
  index.Fill(0);
  
  std::cout << filter2->GetOutput()->GetPixel(index) << std::endl;
  
  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(2);

  ImageType::RegionType region(start,size);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(-2);
}