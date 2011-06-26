/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkSampleMeanShiftClusteringFilterTest.cxx,v $
  Language:  C++
  Date:      $Date: 2005-02-08 03:18:41 $
  Version:   $Revision: 1.5 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#if defined(_MSC_VER)
#pragma warning ( disable : 4786 )
#endif

#include "itkImageFileReader.h"
#include "itkImageRegionIterator.h"
#include "itkScalarImageToListAdaptor.h"
#include "itkKdTree.h"
#include "itkKdTreeGenerator.h"
#include "itkMeanShiftModeCacheMethod.h"
#include "itkHypersphereKernelMeanShiftModeSeeker.h"
#include "itkSampleMeanShiftBlurringFilter.h"
#include "itkSampleMeanShiftClusteringFilter.h"

#include "itkImageFileWriter.h"


int main(int argc, char* argv[] ) 
{

  //verify command line arguments
  if (argc != 3)
    {
      std::cout << "Required arguments: inputFileName OutputFileName" 
                << std::endl ;
      return EXIT_FAILURE;
    }
    
  //parse command line arguments
  char* inputFileName = argv[1];
  char* outputFileName = argv[2];
  
  bool saveClusteredImage = true;
  typedef unsigned char PixelType ;
  typedef itk::Image< PixelType, 2 > ImageType ;
  typedef itk::ImageFileReader< ImageType > ImageReaderType ;
  ImageReaderType::Pointer imageReader = ImageReaderType::New() ;

  imageReader->SetFileName(inputFileName) ;
  imageReader->Update() ;
  ImageType::Pointer image = imageReader->GetOutput() ;
  
  typedef itk::Statistics::ScalarImageToListAdaptor< ImageType >
    ListSampleType ;
  
  ListSampleType::Pointer listSample = 
    ListSampleType::New() ;
  listSample->SetImage( image ) ;

  typedef itk::Statistics::KdTreeGenerator< ListSampleType > 
    TreeGeneratorType ;
  TreeGeneratorType::Pointer treeGenerator = TreeGeneratorType::New() ;
  treeGenerator->SetSample( listSample ) ;
  treeGenerator->SetBucketSize( 200 ) ;
  treeGenerator->Update() ;

  typedef TreeGeneratorType::KdTreeType TreeType ;
  TreeType::Pointer tree = treeGenerator->GetOutput() ;

  typedef itk::Statistics::HypersphereKernelMeanShiftModeSeeker< 
    TreeType > ModeSeekerType ;
  ModeSeekerType::Pointer modeSeeker = ModeSeekerType::New() ;
  modeSeeker->SetInputSample( tree ) ;
//  modeSeeker->SetInputSample( listSample ) ;
  modeSeeker->SetSearchRadius( 4.0 ) ;

  typedef itk::Statistics::MeanShiftModeCacheMethod< TreeType::MeasurementVectorType > CacheMethodType ;
  CacheMethodType::Pointer cacheMethod = CacheMethodType::New() ;
  cacheMethod->SetMaximumEntries(255) ;
  cacheMethod->SetMaximumConsecutiveFailures(100) ;
  cacheMethod->SetHitRatioThreshold( 0.5 ) ;
  modeSeeker->SetCacheMethod( cacheMethod.GetPointer() ) ;

  typedef itk::Statistics::SampleMeanShiftBlurringFilter< TreeType >
    FilterType ;
  FilterType::Pointer filter = FilterType::New() ;
  filter->SetInputSample( tree ) ;
  filter->SetMeanShiftModeSeeker( modeSeeker ) ;
  try 
    {
    filter->Update() ;
    }
  catch ( ... )
    {
    std::cout << "Test failed. - blurring proces" << std::endl;
    return EXIT_FAILURE;
    }

  std::cout << "Cache statistics: " << std::endl ;
  cacheMethod->Print(std::cout) ;

  typedef ImageType OutputImageType ;
  OutputImageType::Pointer outputImage = OutputImageType::New() ;
  outputImage->SetRegions( image->GetLargestPossibleRegion() ) ;
  outputImage->Allocate() ;

  typedef itk::ImageRegionIterator< OutputImageType > ImageIteratorType ;
  ImageIteratorType io_iter( outputImage,
                             outputImage->GetLargestPossibleRegion() ) ;
  io_iter.GoToBegin() ;

  FilterType::OutputType::Pointer output = filter->GetOutput() ;
  FilterType::OutputType::Iterator fo_iter = output->Begin() ;
  FilterType::OutputType::Iterator fo_end = output->End() ;

  while ( fo_iter != fo_end )
    {
    io_iter.Set( (PixelType) fo_iter.GetMeasurementVector()[0]) ;
    ++fo_iter ;
    ++io_iter ;
    }

  ListSampleType::Pointer listSample2 = ListSampleType::New() ;
  listSample2->SetImage( outputImage ) ;

  TreeGeneratorType::Pointer treeGenerator2 = TreeGeneratorType::New() ;
  treeGenerator2->SetSample( listSample2 ) ;
  treeGenerator2->SetBucketSize( 200 ) ;
  treeGenerator2->Update() ;

  typedef itk::Statistics::SampleMeanShiftClusteringFilter< TreeType >
    ClusteringMethodType ;

  ClusteringMethodType::Pointer clusteringMethod =
    ClusteringMethodType::New() ;
  clusteringMethod->SetInputSample( treeGenerator2->GetOutput() ) ;
  clusteringMethod->SetThreshold( 0.5 ) ;
  clusteringMethod->SetMinimumClusterSize( 16 ) ;
  clusteringMethod->DebugOn() ;
  try 
    {
    clusteringMethod->Update() ;
    }
  catch ( ... )
    {
    std::cout << "Test failed. - clustering proces" << std::endl;
    return EXIT_FAILURE;
    }

  if ( saveClusteredImage )
    {
    OutputImageType::Pointer clusterMap = OutputImageType::New() ;
    clusterMap->SetRegions( image->GetLargestPossibleRegion() ) ;
    clusterMap->Allocate() ;
    
    ImageIteratorType m_iter( clusterMap, 
                              clusterMap->GetLargestPossibleRegion() ) ;
    m_iter.GoToBegin() ;
    
    ClusteringMethodType::ClusterLabelsType clusterLabels = 
      clusteringMethod->GetOutput() ;
    
    ClusteringMethodType::ClusterLabelsType::iterator co_iter = 
      clusterLabels.begin() ;
    
    while ( co_iter != clusterLabels.end() )
      {
      m_iter.Set( (PixelType) *co_iter ) ;
      ++co_iter ;
      ++m_iter ;
      }
    
    typedef itk::ImageFileWriter< OutputImageType > ImageWriterType ;
    ImageWriterType::Pointer map_writer = ImageWriterType::New() ;
    map_writer->SetFileName(outputFileName) ;
    map_writer->SetInput( clusterMap ) ;
    map_writer->Update() ;
    }

  return EXIT_SUCCESS;
}


