#include "itkImage.h"
#include "itkImportImageFilter.h"

#include "itkImageFileWriter.h"

int main(int, char * [])
{
  typedef unsigned char   PixelType;
  const unsigned int Dimension = 3;
  typedef itk::Image< PixelType, Dimension > ImageType;
  typedef itk::ImportImageFilter< PixelType, Dimension >   ImportFilterType;

  ImportFilterType::Pointer importFilter = ImportFilterType::New();

  ImportFilterType::SizeType  size;

  size[0]  = 200;  // size along X
  size[1]  = 200;  // size along Y
  size[2]  = 200;  // size along Z

  ImportFilterType::IndexType start;
  start.Fill( 0 );

  ImportFilterType::RegionType region;
  region.SetIndex( start );
  region.SetSize(  size  );

  importFilter->SetRegion( region );

  double origin[ Dimension ];
  origin[0] = 0.0;    // X coordinate
  origin[1] = 0.0;    // Y coordinate
  origin[2] = 0.0;    // Z coordinate

  importFilter->SetOrigin( origin );

  double spacing[ Dimension ];
  spacing[0] = 1.0;    // along X direction
  spacing[1] = 1.0;    // along Y direction
  spacing[2] = 1.0;    // along Z direction

  importFilter->SetSpacing( spacing );

  const unsigned int numberOfPixels =  size[0] * size[1] * size[2];
  PixelType * localBuffer = new PixelType[ numberOfPixels ];

  const double radius = 80.0;

  const double radius2 = radius * radius;
  PixelType * it = localBuffer;

  for(unsigned int z=0; z < size[2]; z++)
    {
    const double dz = static_cast<double>( z ) - static_cast<double>(size[2])/2.0;
    for(unsigned int y=0; y < size[1]; y++)
      {
      const double dy = static_cast<double>( y ) - static_cast<double>(size[1])/2.0;
      for(unsigned int x=0; x < size[0]; x++)
        {
        const double dx = static_cast<double>( x ) - static_cast<double>(size[0])/2.0;
        const double d2 = dx*dx + dy*dy + dz*dz;
        *it++ = ( d2 < radius2 ) ? 255 : 0;
        }
      }
    }

  const bool importImageFilterWillOwnTheBuffer = true;
  importFilter->SetImportPointer( localBuffer, numberOfPixels,
                                  importImageFilterWillOwnTheBuffer );

  typedef itk::ImageFileWriter< ImageType > WriterType;
  WriterType::Pointer writer = WriterType::New();

  writer->SetFileName("test.png");

  writer->SetInput(  importFilter->GetOutput()  );
  writer->Update();

  return EXIT_SUCCESS;
}
