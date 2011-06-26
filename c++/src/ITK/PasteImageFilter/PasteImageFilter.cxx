#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkPasteImageFilter.h"

#include <itkImageToVTKImageFilter.h>

#include "vtkImageViewer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkSmartPointer.h"
#include "vtkImageActor.h"
#include "vtkInteractorStyleImage.h"
#include "vtkRenderer.h"

typedef itk::Image<unsigned char, 2>  ImageType;

void CreateImage1(ImageType::Pointer image);
void CreateImage2(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image1 = ImageType::New();
  CreateImage1(image1);

  ImageType::Pointer image2 = ImageType::New();
  CreateImage2(image2);

  typedef itk::PasteImageFilter <ImageType, ImageType >
          PasteImageFilterType;

  // The SetDestinationIndex() method prescribes where in the first input to start pasting data from the second input.
  // The SetSourceRegion method prescribes the section of the second image to paste into the first.

  ImageType::IndexType destinationIndex;
  destinationIndex[0] = 110;
  destinationIndex[1] = 110;
  
  PasteImageFilterType::Pointer pasteFilter
          = PasteImageFilterType::New ();
  pasteFilter->SetInput(0, image1);
  pasteFilter->SetInput(1, image2);
  pasteFilter->SetSourceRegion(image2->GetLargestPossibleRegion());
  pasteFilter->SetDestinationIndex(destinationIndex);
  pasteFilter->Update();

  // Visualize first image
  typedef itk::ImageToVTKImageFilter<ImageType> ConnectorType;
  ConnectorType::Pointer connector1 = ConnectorType::New();
  connector1->SetInput(image1);

  vtkSmartPointer<vtkImageActor> actor1 =
    vtkSmartPointer<vtkImageActor>::New();
  actor1->SetInput(connector1->GetOutput());

  // Visualize second image
  typedef itk::ImageToVTKImageFilter<ImageType> ConnectorType;
  ConnectorType::Pointer connector2 = ConnectorType::New();
  connector2->SetInput(image2);

  vtkSmartPointer<vtkImageActor> actor2 =
    vtkSmartPointer<vtkImageActor>::New();
  actor2->SetInput(connector2->GetOutput());

  // Visualize joined image
  ConnectorType::Pointer pasteConnector = ConnectorType::New();
  pasteConnector->SetInput(pasteFilter->GetOutput());

  vtkSmartPointer<vtkImageActor> pasteActor =
    vtkSmartPointer<vtkImageActor>::New();
  pasteActor->SetInput(pasteConnector->GetOutput());

  // There will be one render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(900, 300);

  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double leftViewport[4] = {0.0, 0.0, 0.33, 1.0};
  double centerViewport[4] = {0.33, 0.0, 0.66, 1.0};
  double rightViewport[4] = {0.66, 0.0, 1.0, 1.0};

  // Setup both renderers
  vtkSmartPointer<vtkRenderer> leftRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(leftRenderer);
  leftRenderer->SetViewport(leftViewport);
  leftRenderer->SetBackground(.6, .5, .4);

  vtkSmartPointer<vtkRenderer> centerRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(centerRenderer);
  centerRenderer->SetViewport(centerViewport);
  centerRenderer->SetBackground(.4, .5, .6);
  
  vtkSmartPointer<vtkRenderer> rightRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(rightRenderer);
  rightRenderer->SetViewport(rightViewport);
  rightRenderer->SetBackground(.4, .5, .6);

  // Add the sphere to the left and the cube to the right
  leftRenderer->AddActor(actor1);
  centerRenderer->AddActor(actor2);
  rightRenderer->AddActor(pasteActor);

  leftRenderer->ResetCamera();
  centerRenderer->ResetCamera();
  rightRenderer->ResetCamera();

  renderWindow->Render();

  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();
  interactor->SetInteractorStyle(style);

  interactor->Start();

  return EXIT_SUCCESS;
}

void CreateImage1(ImageType::Pointer image)
{
  // Create an image with 2 connected components
  ImageType::RegionType region;
  ImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;

  ImageType::SizeType size;
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
          ImageType::IndexType pixelIndex;
          pixelIndex[0] = r;
          pixelIndex[1] = c;

          image->SetPixel(pixelIndex, 255);
      }
  }
}


void CreateImage2(ImageType::Pointer image)
{
  // Create an image
  ImageType::RegionType region;
  ImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;

  ImageType::SizeType size;
  unsigned int NumRows = 20;
  unsigned int NumCols = 20;
  size[0] = NumRows;
  size[1] = NumCols;

  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  // Make another square
  for(unsigned int r = 0; r < NumRows; r++)
  {
      for(unsigned int c = 0; c < NumCols; c++)
      {
          ImageType::IndexType pixelIndex;
          pixelIndex[0] = r;
          pixelIndex[1] = c;

          image->SetPixel(pixelIndex, 200);
      }
  }
}