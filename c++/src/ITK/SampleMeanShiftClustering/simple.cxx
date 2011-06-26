#include <itkImageFileReader.h>
#include <itkImageRegionIterator.h>
#include <itkScalarImageToListAdaptor.h>
#include <itkKdTree.h>
#include <itkKdTreeGenerator.h>
#include <itkMeanShiftModeCacheMethod.h>
#include <itkHypersphereKernelMeanShiftModeSeeker.h>
#include <itkSampleMeanShiftBlurringFilter.h>
#include <itkSampleMeanShiftClusteringFilter.h>

#include <itkImageFileWriter.h>

int main(int argc, char* argv[] ) 
{
  if (argc < 2)
    {
      std::cout << "ERROR: data file name argument missing." 
                << std::endl ;
      return EXIT_FAILURE;
    }

  std::string inputFilename = argv[1];
  
  typedef unsigned char PixelType ;
  typedef itk::Image< PixelType, 2 > ImageType ;
  typedef itk::ImageFileReader< ImageType > ImageReaderType ;
  ImageReaderType::Pointer imageReader = ImageReaderType::New() ;

  imageReader->SetFileName(inputFilename.c_str());
  imageReader->Update() ;
  ImageType::Pointer image = imageReader->GetOutput() ;
  
  typedef itk::Statistics::SampleMeanShiftBlurringFilter< TreeType >
    FilterType ;
  FilterType::Pointer filter = FilterType::New() ;
  filter->SetInputSample( tree ) ;
  filter->Update() ;

  typedef ImageType OutputImageType ;
  OutputImageType::Pointer outputImage = OutputImageType::New() ;
  outputImage->SetRegions( image->GetLargestPossibleRegion() ) ;
  outputImage->Allocate() ;

  typedef itk::Statistics::SampleMeanShiftClusteringFilter< TreeType >
    ClusteringMethodType ;

  ClusteringMethodType::Pointer clusteringMethod =
    ClusteringMethodType::New() ;
  clusteringMethod->SetInputSample( treeGenerator2->GetOutput() ) ;
  clusteringMethod->SetThreshold( 0.5 ) ;
  clusteringMethod->SetMinimumClusterSize( 16 ) ;
  clusteringMethod->Update() ;

  
  typedef itk::ImageFileWriter< OutputImageType > ImageWriterType ;
  ImageWriterType::Pointer map_writer = ImageWriterType::New() ;
  map_writer->SetFileName("segmented.png") ;
  map_writer->SetInput( clusterMap ) ;
  map_writer->Update() ;


  return EXIT_SUCCESS;
}
