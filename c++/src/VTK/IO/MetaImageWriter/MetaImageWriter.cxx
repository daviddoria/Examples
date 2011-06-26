#include <vtkImageData.h>
#include <vtkMetaImageWriter.h>
#include <vtkSmartPointer.h>
#include <vtkImageMandelbrotSource.h>

int main(int, char *[])
{
  // Create an image
  vtkSmartPointer<vtkImageMandelbrotSource> source =
    vtkSmartPointer<vtkImageMandelbrotSource>::New();
  source->Update();

  vtkSmartPointer<vtkMetaImageWriter> writer =
    vtkSmartPointer<vtkMetaImageWriter>::New();
  writer->SetInputConnection(source->GetOutputPort());
  writer->SetFileName("test.mhd");
  writer->SetRAWFileName("test.raw");
  writer->Write();

  return EXIT_SUCCESS;
}