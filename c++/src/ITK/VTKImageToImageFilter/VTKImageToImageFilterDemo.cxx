#include <itkImage.h>

#include <itkVTKImageToImageFilter.h>

#include "vtkSmartPointer.h"
#include "vtkPNGReader.h"
#include <vtkImageLuminance.h>

#include "QuickView.h"

int main(int argc, char*argv[])
{
  if(argc < 2)
    {
    std::cerr << "Required: filename" << std::endl;
    return EXIT_FAILURE;
    }

  vtkSmartPointer<vtkPNGReader> reader =
    vtkSmartPointer<vtkPNGReader>::New();
  reader->SetFileName(argv[1]);
  // reader->SetNumberOfScalarComponents(1); //doesn't seem to work - use ImageLuminance instead
  reader->Update();


  // Must convert image to grayscale because itkVTKImageToImageFilter only accepts single channel images
  vtkSmartPointer<vtkImageLuminance> luminanceFilter =
    vtkSmartPointer<vtkImageLuminance>::New();
  luminanceFilter->SetInputConnection(reader->GetOutputPort());
  luminanceFilter->Update();

  typedef itk::Image<unsigned char, 2> ImageType;

  typedef itk::VTKImageToImageFilter<ImageType> VTKImageToImageType;

  VTKImageToImageType::Pointer vtkImageToImageFilter = VTKImageToImageType::New();
  vtkImageToImageFilter->SetInput(luminanceFilter->GetOutput());
  //vtkImageToImageFilter->SetInput(reader->GetOutput());
  vtkImageToImageFilter->Update();

  ImageType::Pointer image = ImageType::New();
  image->Graft(vtkImageToImageFilter->GetOutput()); // Need to do this because QuickView can't accept const

  QuickView viewer;
  viewer.AddImage(image.GetPointer()); // Need to do this because QuickView can't accept smart pointers
  viewer.Visualize();

  return EXIT_SUCCESS;
}