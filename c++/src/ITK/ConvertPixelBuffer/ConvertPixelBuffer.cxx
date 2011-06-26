#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkConvertPixelBuffer.h"
#include "itkDefaultConvertPixelTraits.h"

#include <itkImageToVTKImageFilter.h>

#include "vtkImageViewer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkSmartPointer.h"
#include "vtkImageActor.h"
#include "vtkInteractorStyleImage.h"
#include "vtkRenderer.h"

typedef itk::Image<unsigned char, 2>  ScalarImageType;
typedef itk::Image<itk::Vector<unsigned char, 3>, 2>  ColorImageType;

void CreateImage(ColorImageType::Pointer image);

int main(int, char *[])
{
  ColorImageType::Pointer image = ColorImageType::New();
  CreateImage(image);

  typedef itk::ConvertPixelBuffer<ColorImageType, ScalarImageType, itk::DefaultConvertPixelTraits< ScalarImageType::PixelType > >
          ConvertFilterType;

  ScalarImageType::Pointer scalarImage = ScalarImageType::New();
  ConvertFilterType::ConvertRGBToGray(image->GetBufferPointer(), scalarImage->GetBufferPointer(),
                                      scalarImage->GetLargestPossibleRegion().GetSize()[0]*scalarImage->GetLargestPossibleRegion().GetSize()[1]);

  // Visualize first image
  typedef itk::ImageToVTKImageFilter<ColorImageType> ColorConnectorType;
  ColorConnectorType::Pointer connector = ColorConnectorType::New();
  connector->SetInput(image);

  vtkSmartPointer<vtkImageActor> actor =
    vtkSmartPointer<vtkImageActor>::New();
  actor->SetInput(connector->GetOutput());

  // Visualize flipped image
  typedef itk::ImageToVTKImageFilter<ScalarImageType> ScalarConnectorType;
  ScalarConnectorType::Pointer grayscaleConnector = ScalarConnectorType::New();
  grayscaleConnector->SetInput(scalarImage);

  vtkSmartPointer<vtkImageActor> grayscaleActor =
    vtkSmartPointer<vtkImageActor>::New();
  grayscaleActor->SetInput(grayscaleConnector->GetOutput());

  // There will be one render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(600, 300);

  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double leftViewport[4] = {0.0, 0.0, 0.5, 1.0};
  double rightViewport[4] = {0.5, 0.0, 1.0, 1.0};

  // Setup both renderers
  vtkSmartPointer<vtkRenderer> leftRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(leftRenderer);
  leftRenderer->SetViewport(leftViewport);
  leftRenderer->SetBackground(.6, .5, .4);

  vtkSmartPointer<vtkRenderer> rightRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(rightRenderer);
  rightRenderer->SetViewport(rightViewport);
  rightRenderer->SetBackground(.4, .5, .6);

  // Add the sphere to the left and the cube to the right
  leftRenderer->AddActor(actor);
  rightRenderer->AddActor(grayscaleActor);

  leftRenderer->ResetCamera();
  rightRenderer->ResetCamera();

  renderWindow->Render();

  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();
  interactor->SetInteractorStyle(style);

  interactor->Start();

  return EXIT_SUCCESS;
}

void CreateImage(ColorImageType::Pointer image)
{
  // Create an image with 2 connected components
  ColorImageType::RegionType region;
  ColorImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;

  ColorImageType::SizeType size;
  unsigned int NumRows = 200;
  unsigned int NumCols = 300;
  size[0] = NumRows;
  size[1] = NumCols;

  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  // Make a square
  for(unsigned int r = 20; r < 80; r++)
  {
      for(unsigned int c = 20; c < 80; c++)
      {
          ColorImageType::IndexType pixelIndex;
          pixelIndex[0] = r;
          pixelIndex[1] = c;
          ColorImageType::PixelType pixel;
          pixel[0] = 150;
          pixel[1] = 0;
          pixel[2] = 0;
          image->SetPixel(pixelIndex, pixel);
      }
  }
}
