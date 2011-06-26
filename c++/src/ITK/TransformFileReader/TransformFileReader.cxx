#include "itkRigid2DTransform.h"
#include "itkTransformFileReader.h"

int main(int, char *[])
{
  typedef itk::Rigid2DTransform< float > TransformType;
  TransformType::Pointer transform = TransformType::New();

  itk::TransformFileReader::Pointer reader = itk::TransformFileReader::New();
  reader->SetFileName("test.tfm");
  reader->Update();

  return EXIT_SUCCESS;
}
