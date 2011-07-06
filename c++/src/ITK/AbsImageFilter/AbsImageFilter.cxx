#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkAbsImageFilter.h"

#include <itkImageToVTKImageFilter.h>

#include "vtkImageViewer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkSmartPointer.h"
#include "vtkImageActor.h"
#include "vtkInteractorStyleImage.h"
#include "vtkRenderer.h"

typedef itk::Image<unsigned char, 2>  UnsignedCharImageType;
typedef itk::Image<float, 2>  FloatImageType;

void CreateImage(FloatImageType::Pointer image);

int main(int, char *[])
{
  FloatImageType::Pointer image = FloatImageType::New();
  CreateImage(image);

  // Rescale the image so that it can be displayed
  typedef itk::RescaleIntensityImageFilter< FloatImageType, UnsignedCharImageType > FloatRescaleCastFilterType;
  FloatRescaleCastFilterType::Pointer rescaleFilter = FloatRescaleCastFilterType::New();
  rescaleFilter->SetInput(image);
  rescaleFilter->SetOutputMinimum(0);
  rescaleFilter->SetOutputMaximum(255);
  rescaleFilter->Update();

  // Take the absolute value of the image
  typedef itk::AbsImageFilter <FloatImageType, UnsignedCharImageType>
          AbsImageFilterType;

  AbsImageFilterType::Pointer absFilter
          = AbsImageFilterType::New ();
  absFilter->SetInput(image);
  absFilter->Update();

  typedef itk::RescaleIntensityImageFilter< UnsignedCharImageType, UnsignedCharImageType > RescaleFilterType;
  RescaleFilterType::Pointer absRescaleFilter = RescaleFilterType::New();
  absRescaleFilter->SetInput(absFilter->GetOutput());
  absRescaleFilter->SetOutputMinimum(0);
  absRescaleFilter->SetOutputMaximum(255);
  absRescaleFilter->Update();

  // Visualize input image
  typedef itk::ImageToVTKImageFilter<UnsignedCharImageType> ConnectorType;
  ConnectorType::Pointer originalConnector = ConnectorType::New();
  originalConnector->SetInput(rescaleFilter->GetOutput());

  vtkSmartPointer<vtkImageActor> actor =
    vtkSmartPointer<vtkImageActor>::New();
  actor->SetInput(originalConnector->GetOutput());

  // Visualize abs image
  ConnectorType::Pointer absConnector = ConnectorType::New();
  absConnector->SetInput(absRescaleFilter->GetOutput());

  vtkSmartPointer<vtkImageActor> absActor =
    vtkSmartPointer<vtkImageActor>::New();
  absActor->SetInput(absConnector->GetOutput());

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
  rightRenderer->AddActor(absActor);

  leftRenderer->ResetCamera();
  rightRenderer->ResetCamera();

  renderWindow->Render();

  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();
  interactor->SetInteractorStyle(style);

  interactor->Start();

  return EXIT_SUCCESS;
}

void CreateImage(FloatImageType::Pointer image)
{
  // Create an image with negative values
  FloatImageType::IndexType start;
  start.Fill(0);

  FloatImageType::SizeType size;
  size[0] = 200;
  size[1] = 300;

  FloatImageType::RegionType region(start,size);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<FloatImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(imageIterator.GetIndex()[0] - imageIterator.GetIndex()[1]);
    ++imageIterator;
    }

}
