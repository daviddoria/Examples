#include <iostream>

#include "itkVectorGradientAnisotropicDiffusionImageFilter.h"
#include "itkVectorGradientMagnitudeImageFilter.h"
#include "itkWatershedImageFilter.h"
//#include "itkWatershedSegmentTable.h"
#include <Review/itkLabelMap.h>

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkVectorCastImageFilter.h"
#include "itkUnaryFunctorImageFilter.h"
#include "itkScalarToRGBPixelFunctor.h"

int main( int argc, char *argv[] )
{
	if (argc < 3 )
	{
		std::cerr << "Missing Parameters " << std::endl;
		std::cerr << "inputImage outputImage " << std::endl;
		return 1;
	}
	
	unsigned int lowerThreshold = 0;
	double outputScaleLevel = 0.05;
	unsigned int gradientMode = 1;
	
	std::string InputFilename = argv[1];
	std::string OutputFilename = argv[2];

	typedef itk::RGBPixel<unsigned char>   RGBPixelType;
	typedef itk::Image<RGBPixelType, 2>    RGBImageType;
	typedef itk::Vector<float, 3>          VectorPixelType;
	typedef itk::Image<VectorPixelType, 2> VectorImageType;
	typedef itk::Image<unsigned long, 2>   LabeledImageType;
	typedef itk::Image<float, 2>           ScalarImageType;
	
	typedef itk::ImageFileReader<RGBImageType> FileReaderType;
	typedef itk::VectorCastImageFilter<RGBImageType, VectorImageType> CastFilterType;
	
	typedef itk::VectorGradientMagnitudeImageFilter<VectorImageType> GradientMagnitudeFilterType; 
	typedef itk::WatershedImageFilter<ScalarImageType> WatershedFilterType;

	typedef itk::ImageFileWriter<RGBImageType> FileWriterType;
	
	FileReaderType::Pointer reader = FileReaderType::New();
	reader->SetFileName(InputFilename.c_str());
	
	CastFilterType::Pointer caster = CastFilterType::New();
		
	GradientMagnitudeFilterType::Pointer gradient = GradientMagnitudeFilterType::New();
	gradient->SetUsePrincipleComponents(gradientMode);
	
	WatershedFilterType::Pointer watershed = WatershedFilterType::New();
	watershed->SetLevel( outputScaleLevel );
	watershed->SetThreshold( lowerThreshold );
	
	typedef itk::Functor::ScalarToRGBPixelFunctor<unsigned long> ColorMapFunctorType;
	typedef itk::UnaryFunctorImageFilter<LabeledImageType, RGBImageType, ColorMapFunctorType> ColorMapFilterType;
	ColorMapFilterType::Pointer colormapper = ColorMapFilterType::New();
	
	FileWriterType::Pointer writer = FileWriterType::New();
	writer->SetFileName(OutputFilename.c_str());
	
	/*
	//original
	caster->SetInput(reader->GetOutput());
	diffusion->SetInput(caster->GetOutput());
	gradient->SetInput(diffusion->GetOutput());
	watershed->SetInput(gradient->GetOutput());
	colormapper->SetInput(watershed->GetOutput());
	writer->SetInput(colormapper->GetOutput());
	*/
	
	/*
	//without diffusion filter
	caster->SetInput(reader->GetOutput());
	gradient->SetInput(caster->GetOutput());
	watershed->SetInput(gradient->GetOutput());
	colormapper->SetInput(watershed->GetOutput());
	writer->SetInput(colormapper->GetOutput());
	*/
	
	/*
	typedef SegmentTable<InputPixelType> SegmentTableType;
	SegmentTableType* SegmentTable = watershed->GetSegmentTable();
	
	unsigned int NumSegments = SegmentTable->Size();
	
	writer->Update();
	*/
	
	return 0;
}
