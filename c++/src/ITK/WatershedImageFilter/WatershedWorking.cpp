#include <iostream>

#include "itkVectorGradientAnisotropicDiffusionImageFilter.h"
#include "itkVectorGradientMagnitudeImageFilter.h"
#include "itkWatershedImageFilter.h"

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkVectorCastImageFilter.h"
#include "itkUnaryFunctorImageFilter.h"
#include "itkScalarToRGBPixelFunctor.h"

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
  
	typedef itk::RGBPixel<unsigned char>   RGBPixelType;
	typedef itk::Image<RGBPixelType, 2>    RGBImageType;
	typedef itk::Vector<float, 3>          VectorPixelType;
	typedef itk::Image<VectorPixelType, 2> VectorImageType;
	typedef itk::Image<unsigned long, 2>   LabeledImageType;
	typedef itk::Image<float, 2>           ScalarImageType;
  	
	typedef itk::ImageFileReader<RGBImageType> FileReaderType;
	typedef itk::VectorCastImageFilter<RGBImageType, VectorImageType> 
			CastFilterType;
			typedef itk::VectorGradientAnisotropicDiffusionImageFilter<VectorImageType,
   VectorImageType>  DiffusionFilterType;
   typedef itk::VectorGradientMagnitudeImageFilter<VectorImageType>
		   GradientMagnitudeFilterType; 
   typedef itk::WatershedImageFilter<ScalarImageType> WatershedFilterType;
  
   typedef itk::ImageFileWriter<RGBImageType> FileWriterType;

   FileReaderType::Pointer reader = FileReaderType::New();
   reader->SetFileName(argv[1]);
  
   CastFilterType::Pointer caster = CastFilterType::New();
  
   DiffusionFilterType::Pointer diffusion = DiffusionFilterType::New();
   //diffusion->SetNumberOfIterations( atoi(argv[4]) );
   diffusion->SetNumberOfIterations( 10 );
   //diffusion->SetConductanceParameter( atof(argv[3]) );
   diffusion->SetConductanceParameter( 2.0 );
		   
   diffusion->SetTimeStep(0.125);
   
   GradientMagnitudeFilterType::Pointer 
		   gradient = GradientMagnitudeFilterType::New();
   gradient->SetUsePrincipleComponents(atoi(argv[7]));
   
   WatershedFilterType::Pointer watershed = WatershedFilterType::New();
   //watershed->SetLevel( atof(argv[6]) );
   watershed->SetLevel( .05 );
   //watershed->SetThreshold( atof(argv[5]) );
   watershed->SetThreshold( 0.0 );
   
   typedef itk::Functor::ScalarToRGBPixelFunctor<unsigned long>
		   ColorMapFunctorType;
   typedef itk::UnaryFunctorImageFilter<LabeledImageType,
   RGBImageType, ColorMapFunctorType> ColorMapFilterType;
   ColorMapFilterType::Pointer colormapper = ColorMapFilterType::New();
  
   FileWriterType::Pointer writer = FileWriterType::New();
   writer->SetFileName(argv[2]);
 
   caster->SetInput(reader->GetOutput());
   diffusion->SetInput(caster->GetOutput());
   gradient->SetInput(diffusion->GetOutput());
   watershed->SetInput(gradient->GetOutput());
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
    
   return 0;
}
