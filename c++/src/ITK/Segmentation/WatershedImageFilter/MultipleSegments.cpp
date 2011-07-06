#include <iostream>
#include <string>
#include <sstream>

#include "itkVectorGradientAnisotropicDiffusionImageFilter.h"
#include "itkVectorGradientMagnitudeImageFilter.h"
#include "itkWatershedImageFilter.h"

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkVectorCastImageFilter.h"
#include "itkUnaryFunctorImageFilter.h"
#include "itkScalarToRGBPixelFunctor.h"

#include <itkLabelMap.h>
#include <itkLabelImageToLabelMapFilter.h>
#include <itkLabelObject.h>
//#include <Review/itkLabelMapFilter.h>
#include <itkLabelMapMaskImageFilter.h>

int main( int argc, char *argv[] )
{
  //input output 2 10 0 0.05 1
  if (argc < 3 )
  {
    std::cerr << "Missing Parameters " << std::endl;
    //std::cerr << "Usage: " << argv[0];
    //std::cerr << " inputImage outputImage conductanceTerm diffusionIterations lowerThreshold outputScaleLevel gradientMode " << std::endl;
    std::cerr << " inputImage outputImage " << std::endl;
    return 1;
  }
  std::string InputFilename = argv[1];
  std::string OutputFilename = argv[2];

  // Pixel types
  typedef itk::RGBPixel<unsigned char>   RGBPixelType;
  typedef itk::Image<RGBPixelType, 2>    RGBImageType;
  typedef itk::Vector<float, 3>          VectorPixelType;
  
  // Image types
  typedef itk::Image<VectorPixelType, 2> VectorImageType;
  typedef itk::Image<unsigned long, 2>   LabeledImageType;
  typedef itk::Image<float, 2>           ScalarImageType;
  
  // Reader type
  typedef itk::ImageFileReader<RGBImageType> FileReaderType;
  // Read the file
  FileReaderType::Pointer reader = FileReaderType::New();
  reader->SetFileName(InputFilename.c_str());
  
  // Cast filter types
  typedef itk::VectorCastImageFilter<RGBImageType, VectorImageType> CastFilterType;
  CastFilterType::Pointer caster = CastFilterType::New();
  
  // Setup blurring filter
  typedef itk::VectorGradientAnisotropicDiffusionImageFilter<VectorImageType, VectorImageType>  DiffusionFilterType;
  DiffusionFilterType::Pointer diffusion = DiffusionFilterType::New();
  diffusion->SetNumberOfIterations( 10 );
  diffusion->SetConductanceParameter( 2.0 );
  diffusion->SetTimeStep(0.125);
  
  // Gradient
  typedef itk::VectorGradientMagnitudeImageFilter<VectorImageType>  GradientMagnitudeFilterType; 
  GradientMagnitudeFilterType::Pointer gradient = GradientMagnitudeFilterType::New();
  gradient->SetUsePrincipleComponents(atoi(argv[7]));
  
  // Watershed filter
  typedef itk::WatershedImageFilter<ScalarImageType> WatershedFilterType;
  WatershedFilterType::Pointer watershed = WatershedFilterType::New();
  watershed->SetLevel( .05 );
  watershed->SetThreshold( 0.0 );
  
  // Setup pseudo-coloring
  typedef itk::Functor::ScalarToRGBPixelFunctor<unsigned long> ColorMapFunctorType;
  typedef itk::UnaryFunctorImageFilter<LabeledImageType, RGBImageType, ColorMapFunctorType> ColorMapFilterType;
  ColorMapFilterType::Pointer colormapper = ColorMapFilterType::New();
  
  // Setup writer
  typedef itk::ImageFileWriter<RGBImageType> FileWriterType;
  
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName(argv[2]);
  
  // Convert to a label map
  typedef itk::LabelObject<unsigned long, 2> LabelObjectType;
  typedef itk::LabelMap < LabelObjectType> LabelMapType;
  typedef itk::LabelImageToLabelMapFilter< LabeledImageType, LabelMapType> I2LType;
  I2LType::Pointer i2l = I2LType::New();
  
  // Setup pipeline
  caster->SetInput(reader->GetOutput());
  diffusion->SetInput(caster->GetOutput());
  gradient->SetInput(diffusion->GetOutput());
  watershed->SetInput(gradient->GetOutput());
  i2l->SetInput(watershed->GetOutput());
  i2l->Update();
  
  LabelMapType::LabelObjectContainerType::const_iterator it;
  LabelMapType::Pointer labelMap = i2l->GetOutput();
  const LabelMapType::LabelObjectContainerType & labelObjectContainer = labelMap->GetLabelObjectContainer();
  
  // Output size and color of each region
  unsigned int counter = 0;
  for(it = labelObjectContainer.begin(); it != labelObjectContainer.end(); it++)
    {
    //const RGBPixelType &label = it->first;
    const long unsigned int &label = it->first;
    LabelObjectType* labelObject = it->second;
    
    std::cout << "Segment #: " << counter << std::endl 
	    << "Label: " << label << std::endl
	    << "Size: " << labelObject->Size() << std::endl << std::endl;
    
    //std::cout << "Segment #: " << counter << std::endl;
    //labelObject->Print(std::cout);
    
    typedef itk::LabelMapMaskImageFilter< LabelMapType, RGBImageType> MaskType;
    MaskType::Pointer mask = MaskType::New();

    mask->SetInput(i2l->GetOutput());
    mask->SetFeatureImage(reader->GetOutput());
    mask->SetLabel(label);

    //setup writer
    FileWriterType::Pointer SegmentWriter = FileWriterType::New();
    std::stringstream ssLabel;
    ssLabel << "label_" << counter << ".png";
    SegmentWriter->SetFileName(ssLabel.str().c_str());
    SegmentWriter->SetInput(mask->GetOutput());
    SegmentWriter->Update();
    
    counter++;
    }
  
  /*
  colormapper->SetInput(watershed->GetOutput());
  writer->SetInput(colormapper->GetOutput());
  
  try 
  {
	  writer->Update();
  }
  catch (itk::ExceptionObject &e)
  {
	  std::cerr << e << std::endl;
  }
  */
  return EXIT_SUCCESS;
}
