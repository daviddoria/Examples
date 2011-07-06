/*
Author: Juan Cardelino <juan dot cardelino at gmail dot com>
*/

// ITK
#include "itkBinaryThresholdImageFilter.h"
#include "itkBoundedReciprocalImageFilter.h"
#include "itkChangeInformationImageFilter.h"
#include "itkCommand.h"
#include "itkCurvatureAnisotropicDiffusionImageFilter.h"
#include "itkEuler2DTransform.h"
#include "itkFastMarchingImageFilter.h"
#include "itkGeodesicActiveContourShapePriorLevelSetImageFilter.h"
#include "itkGradientMagnitudeRecursiveGaussianImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImagePCAShapeModelEstimator.h"
#include "itkMultiplyByConstantImageFilter.h"
#include "itkNumericSeriesFileNames.h"
#include "itkNormalVariateGenerator.h"
#include "itkOnePlusOneEvolutionaryOptimizer.h"
#include "itkPCAShapeSignedDistanceFunction.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkShapePriorMAPCostFunction.h"
#include "itkSpatialFunctionImageEvaluatorFilter.h"

// VNL
#include "vnl/vnl_sample.h"

int main( int argc, char *argv[] )
{
  if( argc < 5 )
    {
    std::cerr << "Missing Parameters " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr << " nbTrain  trainFilePattern";
    std::cerr << " nbModes  modeFilePattern";
    std::cerr << std::endl;
    return 1;
    }

  for(int i=0; i<argc;i++)
    {
    std::cout << "id: " << i << " arg: " << argv[i] << std::endl;
    }
  const   unsigned int        Dimension = 2;
  typedef float    my_PixelType;
  typedef itk::Image< my_PixelType, Dimension >    ImageType;

  typedef  itk::ImageFileReader< ImageType > ReaderType;
  typedef  itk::ImageFileWriter<  ImageType  > WriterType;
  typedef itk::MultiplyByConstantImageFilter < ImageType , double ,ImageType > ScaleType;

  int nb_train=atoi(argv[1]);

  itk::NumericSeriesFileNames::Pointer fileNamesCreator =
                                       itk::NumericSeriesFileNames::New();
  std::vector<ImageType::Pointer> trainingImages( nb_train );

  fileNamesCreator->SetStartIndex( 0 );
  fileNamesCreator->SetEndIndex( nb_train - 1 );
  fileNamesCreator->SetSeriesFormat( argv[2] );
  const std::vector<std::string> & shapeModeFileNames =
                                   fileNamesCreator->GetFileNames();

  for ( unsigned int k = 0; k < nb_train; k++ )
    {
    ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName( shapeModeFileNames[k].c_str() );
    reader->Update();
    trainingImages[k] = reader->GetOutput();
    }

  typedef itk::ImagePCAShapeModelEstimator<ImageType,   ImageType >  my_Estimatortype;
  my_Estimatortype::Pointer filter = my_Estimatortype::New();
  filter->SetNumberOfTrainingImages(nb_train);
  filter->SetNumberOfPrincipalComponentsRequired(2);

  for ( unsigned int k = 0; k < nb_train; k++ )
    {
    filter->SetInput(k, trainingImages[k] );
    }

  int nb_modes=atoi(argv[3]);


  itk::NumericSeriesFileNames::Pointer fileNamesOutCreator =
                                       itk::NumericSeriesFileNames::New();

  fileNamesOutCreator->SetStartIndex( 0 );
  fileNamesOutCreator->SetEndIndex( nb_modes-1  );
  fileNamesOutCreator->SetSeriesFormat( argv[4] );
  const std::vector<std::string> & outFileNames = fileNamesOutCreator->GetFileNames();

  ScaleType::Pointer scaler = ScaleType::New();

  filter->Update();
  my_Estimatortype::VectorOfDoubleType v=filter->GetEigenValues();
  double sv_mean=sqrt(v[0]);

  for ( unsigned int k = 0; k < nb_modes; k++ )
    {
    double sv=sqrt(v[k]);
    double sv_n=sv/sv_mean;
    //double sv_n=sv;
    std::cout << "writing: " << outFileNames[k] << std::endl;

    std::cout << "svd[" << k << "]: " << sv << " norm: " << sv_n << std::endl;
    WriterType::Pointer writer = WriterType::New();
    writer->SetFileName( outFileNames[k].c_str() );
    scaler->SetInput(filter->GetOutput(k));
    scaler->SetConstant(sv_n);
    writer->SetInput( scaler->GetOutput() );
    writer->Update();
    }

  return EXIT_SUCCESS;
}
