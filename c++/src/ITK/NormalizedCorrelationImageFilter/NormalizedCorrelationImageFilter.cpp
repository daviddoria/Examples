#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkNormalizedCorrelationImageFilter.h"
#include "itkRegionOfInterestImageFilter.h"
#include "itkImageKernelOperator.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkImageFileWriter.h"
#include "itkMinimumMaximumImageCalculator.h"
#include "QuickView.h"

#include <iostream>
#include <string>

typedef itk::Image<float, 2> FloatImageType;
typedef itk::Image<unsigned char, 2> UnsignedCharImageType;

int main(int argc, char *argv[])
{
  if(argc < 2)
    {
    std::cerr << "Required: filename" << std::endl;
    return EXIT_FAILURE;
    }

  std::string filename = argv[1];

  typedef itk::ImageFileReader<FloatImageType> ReaderType;

  // Read the image
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(filename.c_str());
  reader->Update();

  // Extract a small region
  typedef itk::RegionOfInterestImageFilter< FloatImageType,
                                            FloatImageType > ExtractFilterType;

  ExtractFilterType::Pointer extractFilter = ExtractFilterType::New();

  FloatImageType::IndexType start;
  start.Fill(50);

  FloatImageType::SizeType patchSize;
  patchSize.Fill(51);

  FloatImageType::RegionType desiredRegion(start,patchSize);

  extractFilter->SetRegionOfInterest(desiredRegion);
  extractFilter->SetInput(reader->GetOutput());
  extractFilter->Update();

  // Perform normalized correlation
  // <input type, mask type (not used), output type>
  typedef itk::NormalizedCorrelationImageFilter<FloatImageType, FloatImageType, FloatImageType> CorrelationFilterType;

  itk::ImageKernelOperator<float> kernelOperator;
  kernelOperator.SetImageKernel(extractFilter->GetOutput());

  // The radius of the kernel must be the radius of the patch, NOT the size of the patch
  itk::Size<2> radius = extractFilter->GetOutput()->GetLargestPossibleRegion().GetSize();
  radius[0] = (radius[0]-1) / 2;
  radius[1] = (radius[1]-1) / 2;

  kernelOperator.CreateToRadius(radius);

  CorrelationFilterType::Pointer correlationFilter = CorrelationFilterType::New();
  correlationFilter->SetInput(reader->GetOutput());
  correlationFilter->SetTemplate(kernelOperator);
  correlationFilter->Update();

  typedef itk::MinimumMaximumImageCalculator <FloatImageType>
          MinimumMaximumImageCalculatorType;

  MinimumMaximumImageCalculatorType::Pointer minimumMaximumImageCalculatorFilter
          = MinimumMaximumImageCalculatorType::New ();
  minimumMaximumImageCalculatorFilter->SetImage(correlationFilter->GetOutput());
  minimumMaximumImageCalculatorFilter->Compute();

  itk::Index<2> maximumCorrelationPatchCenter = minimumMaximumImageCalculatorFilter->GetIndexOfMaximum();
  std::cout << "Maximum: " << maximumCorrelationPatchCenter << std::endl;

  // Note that the best correlation is at the center of the patch we extracted (ie. (75, 75) rather than the corner (50,50)

  typedef itk::RescaleIntensityImageFilter< FloatImageType, UnsignedCharImageType > RescaleFilterType;
  typedef itk::ImageFileWriter<UnsignedCharImageType> WriterType;
  {
  RescaleFilterType::Pointer rescaleFilter = RescaleFilterType::New();
  rescaleFilter->SetInput(correlationFilter->GetOutput());
  rescaleFilter->SetOutputMinimum(0);
  rescaleFilter->SetOutputMaximum(255);
  rescaleFilter->Update();

  WriterType::Pointer writer = WriterType::New();
  writer->SetInput(rescaleFilter->GetOutput());
  writer->SetFileName("correlation.png");
  writer->Update();
  }

  {
  RescaleFilterType::Pointer rescaleFilter = RescaleFilterType::New();
  rescaleFilter->SetInput(extractFilter->GetOutput());
  rescaleFilter->SetOutputMinimum(0);
  rescaleFilter->SetOutputMaximum(255);
  rescaleFilter->Update();

  WriterType::Pointer writer = WriterType::New();
  writer->SetInput(rescaleFilter->GetOutput());
  writer->SetFileName("patch.png");
  writer->Update();
  }

  // Extract the best matching patch
  FloatImageType::IndexType bestPatchStart;
  bestPatchStart[0] = maximumCorrelationPatchCenter[0] - radius[0];
  bestPatchStart[1] = maximumCorrelationPatchCenter[1] - radius[1];

  FloatImageType::RegionType bestPatchRegion(bestPatchStart,patchSize);

  ExtractFilterType::Pointer bestPatchExtractFilter = ExtractFilterType::New();
  bestPatchExtractFilter->SetRegionOfInterest(bestPatchRegion);
  bestPatchExtractFilter->SetInput(reader->GetOutput());
  bestPatchExtractFilter->Update();

  QuickView viewer;
  viewer.AddImage(reader->GetOutput());
  viewer.AddImage(extractFilter->GetOutput());
  viewer.AddImage(correlationFilter->GetOutput());
  viewer.AddImage(bestPatchExtractFilter->GetOutput());
  viewer.Visualize();

  return EXIT_SUCCESS;
}
