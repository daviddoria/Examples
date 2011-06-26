#include <itkImage.h>
#include <itkMetaDataDictionary.h>
#include <itkMetaDataObject.h>
#include <itkImageRegionIterator.h>
#include <itkImageFileWriter.h>
#include <itkImageFileReader.h>

typedef itk::Image<unsigned char, 2> ImageType;

static void CreateImage(ImageType::Pointer image);

int main(int, char*[])
{
  // Create an image
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  // Store some data in it
  itk::MetaDataDictionary dictionary;
  itk::EncapsulateMetaData<float>(dictionary,"ASimpleFloat",1.2);
  image->SetMetaDataDictionary(dictionary);

  // View all of the data
  dictionary.Print(std::cout);

  // View the data individually
  itk::MetaDataDictionary::Iterator itr = dictionary.Begin();

  while( itr != dictionary.End() )
    {
    std::cout << "Key   = " << itr->first << std::endl;
    std::cout << "Value = ";
    itr->second->Print( std::cout );
    std::cout << std::endl;
    ++itr;
    }

  // Write the image (and the data) to a file
  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("test.mhd");
  writer->SetInput(image);
  writer->Update();

  // Read the image (and data) from the file
  typedef itk::ImageFileReader<ImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName("test.mhd");

  // Display the data
  std::cout << "Data read from file:" << std::endl;
  reader->GetMetaDataDictionary().Print(std::cout);

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(10);

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,image->GetLargestPossibleRegion());

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(20);
    ++imageIterator;
    }
}