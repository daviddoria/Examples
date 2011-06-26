#include "itkMesh.h"
#include "itkVector.h"
#include "itkVersor.h"
#include "itkVectorImage.h"
#include "itkMatrix.h"
#include "itkEuler2DTransform.h"
#include "itkRigid2DTransform.h"

int main(int, char *[])
{
  // Create a vector image
  typedef itk::VectorImage<float, 2>  ImageType;

  ImageType::RegionType region;
  ImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;

  ImageType::SizeType size;
  size[0] = 2;
  size[1] = 3;

  region.SetSize( size);
  region.SetIndex(start);

  ImageType::Pointer image = ImageType::New();
  image->SetRegions(region);
  image->SetVectorLength(2);
  image->Allocate();

  ImageType::IndexType pixelIndex;
  pixelIndex[0] = 1;
  pixelIndex[1] = 1;

  // Fill the image with values
  ImageType::PixelType pixelValue;
  pixelValue.SetSize(2);

  pixelValue[0] = 1;
  pixelValue[0] = 0;
  std::cout << "pixelValue: " << pixelValue << std::endl;
  //image->SetPixel(pixelIndex, pixelValue);

  // Create a transform

  //typedef itk::Euler2DTransform< float > TransformType;
  typedef itk::Rigid2DTransform< float > TransformType;
  TransformType::InputPointType p;
  p[0] = 1;
  p[1] = 0;
  std::cout << "inputVector: " << p << std::endl;
  TransformType::Pointer transform = TransformType::New();
  transform->SetAngle(M_PI/2.0);
  //TransformType::InputPointType transformedVector = transform->TransformVector(p);

  // works
  //TransformType::InputPointType transformedVector = transform->TransformPoint(p);

  //ImageType::PixelType transformedVector = transform->TransformPoint(p);

  typedef itk::Vector<double, 2> VectorType;
  VectorType v;
  v[0] = pixelValue[0];
  v[1] = pixelValue[1];

  VectorType outputV = transform->TransformVector(v);
  ImageType::PixelType transformedVector;
  transformedVector[0] = outputV[0];
  transformedVector[1] = outputV[1];


  //ImageType::PixelType transformedVector = transform->TransformVector(image->GetPixel(pixelIndex));

  std::cout << "transformedVector: " << transformedVector << std::endl;

  return EXIT_SUCCESS;
}
