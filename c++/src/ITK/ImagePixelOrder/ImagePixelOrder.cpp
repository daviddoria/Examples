#include <iostream>

#include <itkImage.h>
#include <itkImageFileWriter.h>

int main(int, char*[])
{
  typedef itk::Image< unsigned char, 2>  ImageType;

  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size[0] = 10; // number of columns
  size[1] = 20; // number of rows

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  ImageType::Pointer image = ImageType::New();
  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,region);

  // Pixel values in create left to right, top to bottom
  
  int counter = 0;
  for(unsigned int row = 0; row < size[1]; row++)
  {
    for(unsigned int col = 0; col < size[0]; col++)
    {
    itk::Index<2> index;
    index[0] = col;
    index[1] = row;
    //unsigned char val = (unsigned char)(255.0f * data[col + cols * row]); // close
    unsigned char val = (unsigned char)(counter);
    image->SetPixel(index, val);
    counter++;
    }
  }

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("image.png");
  writer->SetInput(image);
  writer->Update();
  
  return EXIT_SUCCESS;
}