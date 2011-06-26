#include <vtkJPEGReader.h>
#include <vtkJPEGWriter.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>

#include <vtkPolyDataToImageStencil.h>
#include <vtkTextSource.h>
#include <vtkImageStencilData.h>
#include <vtkImageStencil.h>

int main ( int argc, char* argv[] )
{
  vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();
  image->SetDimensions(100,200,1);

  vtkSmartPointer<vtkTextSource> textSource = vtkSmartPointer<vtkTextSource>::New();
  textSource->SetText("hello");
  textSource->Update();

  vtkSmartPointer<vtkPolyDataToImageStencil> polyDataToImageStencil = vtkSmartPointer<vtkPolyDataToImageStencil>::New();
  polyDataToImageStencil->SetInput(textSource->GetOutput());
  polyDataToImageStencil->Update();

  vtkSmartPointer<vtkImageStencil> stencil = vtkSmartPointer<vtkImageStencil>::New();
  stencil->SetStencil(polyDataToImageStencil->GetOutput());
  stencil->SetInput(image);
  stencil->Update();

  vtkImageData* outputImage = stencil->GetOutput();

  vtkSmartPointer<vtkJPEGWriter> writer = vtkSmartPointer<vtkJPEGWriter>::New();
  writer->SetInput(outputImage);
  writer->SetFileName("test.jpg");
  writer->Write();

  return 0 ;
}
