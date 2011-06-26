#include "itkVectorGradientAnisotropicDiffusionImageFilter.h"

typedef itk::Image<unsigned char, 2> ImageType;

void CreateCircleImage(ImageType::Pointer image);

int main()
{
  typedef itk::FastMarchingImageFilter<ImageType, ImageType> FastMarchingFilterType;
  typedef FastMarchingFilterType::NodeType       NodeType;
  typedef FastMarchingFilterType::NodeContainer  NodeContainer;

  ImageType::Pointer image = ImageType::New();
  CreateCircleImage(image);

  itk::Index<2> startPoint;
  startPoint[0] = 30;
  startPoint[1] = 49;

  if(!image->GetPixel(startPoint))
    {
    std::cerr << "Start point is not on the circle!" << std::endl;
    return EXIT_FAILURE;
    }

  itk::Index<2> endPoint;
  endPoint[0] = 69;
  endPoint[1] = 50;

  if(!image->GetPixel(endPoint))
    {
    std::cerr << "End point is not on the circle!" << std::endl;
    return EXIT_FAILURE;
    }
    
  NodeType node;
  node.SetValue( 0.0 );
  node.SetIndex( startPoint );
  NodeContainer::Pointer trialPoints = NodeContainer::New();
  trialPoints->InsertElement(0, node);

  FastMarchingFilterType::Pointer fastMarchingFilter = FastMarchingFilterType::New();
  fastMarchingFilter->SetTrialPoints( trialPoints );
  fastMarchingFilter->SetBinaryMask(image.GetPointer());
  fastMarchingFilter->Update();

  double distance = fastMarchingFilter->GetOutput()->GetPixel(endPoint);
  std::cout << "Distance: " << distance << std::endl;
  
  return EXIT_SUCCESS;
}


void CreateCircleImage(ImageType::Pointer image)
{
  itk::Size<2> size;
  size.Fill(100);

  itk::Index<2> start;
  start.Fill(0);

  itk::ImageRegion<2> region(start, size);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

  for(unsigned int i = 0; i < 1000; i++)
    {
    int x = 20 * cos(i) + 50;
    int y = 20 * sin(i) + 50;
    itk::Index<2> pixel;
    pixel[0] = x;
    pixel[1] = y;
    image->SetPixel(pixel, 255);
    }
}