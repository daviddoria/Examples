#include "itkSparseImage.h"

#include <iostream>
#include <string>

struct Node
{
  itk::Index<2> m_Index;
  unsigned char data;
  Node *Next;
  Node *Previous;
};

int main(int argc, char *argv[])
{
  typedef itk::SparseImage<Node, 2>  ImageType;

  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(100);

  itk::ImageRegion<2> region(start, size);

  ImageType::Pointer image = ImageType::New();
  image->SetRegions(region);
  image->Allocate();

  return EXIT_SUCCESS;
}
