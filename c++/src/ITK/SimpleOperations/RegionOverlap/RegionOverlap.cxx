#include <cstdlib>
#include <cstdio>

#include "QuickView.h"

#include "itkRGBPixel.h"
#include "itkImageRegionIterator.h"
#include "itkImageFileWriter.h"

int main(int argc, char *argv[])
{
  const unsigned int Dimension = 2;
  typedef itk::RGBPixel <unsigned char >           RGBPixelType;
  typedef itk::Image< RGBPixelType, Dimension >    RGBImageType;
  typedef RGBImageType::RegionType                 RegionType;
  typedef RGBImageType::IndexType                  IndexType;
  typedef RGBImageType::SizeType                   SizeType;
  typedef itk::ImageRegionIterator< RGBImageType > IteratorType;

  IndexType index;
  index[0] = 0;
  index[1] = 0;

  SizeType size;
  size[0] = 100;
  size[1] = 100;

  RegionType region;
  region.SetIndex( index );
  region.SetSize( size );

  IndexType indexA;
  indexA[0] = 9;
  indexA[1] = 9;

  SizeType sizeA;
  sizeA[0] = 50;
  sizeA[1] = 50;

  RegionType regionA;
  regionA.SetIndex( indexA );
  regionA.SetSize( sizeA );

  IndexType indexB;
  indexB[0] = 39;
  indexB[1] = 39;

  SizeType sizeB;
  sizeB[0] = 50;
  sizeB[1] = 50;

  RegionType regionB;
  regionB.SetIndex( indexB );
  regionB.SetSize( sizeB );

  // Region C is the intersection of A and B
  // padded by 10 pixels.
  RegionType regionC = regionA;
  regionC.Crop( regionB );
  regionC.PadByRadius( 10 );

  RGBPixelType pix_black;
  pix_black.Fill( 0 );

  RGBPixelType pix_red;
  pix_red.Fill( 0 );
  pix_red[0] = 255;

  RGBPixelType pix_green;
  pix_green.Fill( 0 );
  pix_green[1] = 255;

  RGBPixelType pix_blue;
  pix_blue.Fill( 0 );
  pix_blue[2] = 255;

  // A black canvas
  RGBImageType::Pointer image = RGBImageType::New();
  image->SetRegions( region );
  image->Allocate();
  image->FillBuffer( pix_black );

  // Paint region A red.
  IteratorType itA( image, regionA );
  itA.GoToBegin();
  while ( !itA.IsAtEnd() )
    {
    itA.Set( itA.Get() + pix_red );
    ++itA;
    }

  // Paint region B green.
  IteratorType itB( image, regionB );
  itB.GoToBegin();
  while ( !itB.IsAtEnd() )
    {
    itB.Set( itB.Get() + pix_green );
    ++itB;
    }

  // Paint region C blue.
  IteratorType itC( image, regionC );
  itC.GoToBegin();
  while ( !itC.IsAtEnd() )
    {
    itC.Set( itC.Get() + pix_blue );
    ++itC;
    }

  QuickView viewer;
  viewer.AddImage<RGBImageType>(image);
  viewer.Visualize();

  return EXIT_SUCCESS;
}