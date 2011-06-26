#include "itkImage.h"
#include <itkBinaryBallStructuringElement.h>

int main(int, char *[])
{
  typedef itk::BinaryBallStructuringElement<float, 2> StructuringElementType;
  StructuringElementType structuringElement;
  structuringElement.SetRadius(5);
  structuringElement.CreateStructuringElement();

  return EXIT_SUCCESS;
}
