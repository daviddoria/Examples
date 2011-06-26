#include "itkRigid2DTransform.h"
#include "itkTransformFileWriter.h"

int main(int, char *[])
{
  typedef itk::Rigid2DTransform< float > TransformType;
  TransformType::Pointer transform = TransformType::New();

  itk::TransformFileWriter::Pointer writer = itk::TransformFileWriter::New();
  writer->SetInput(transform);
  writer->SetFileName("test.tfm");
  writer->Update();

  return EXIT_SUCCESS;
}
