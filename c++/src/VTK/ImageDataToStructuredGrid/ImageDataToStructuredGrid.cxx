#include "vtkImageData.h"
#include "vtkPoints.h"
#include "vtkImageDataToStructuredGrid.h"
#include "vtkSmartPointer.h"
#include "vtkXMLStructuredGridWriter.h"
#include "vtkImageReader2.h"
#include "vtkImageReader2Factory.h"
#include "vtkStructuredGrid.h"

int main(int argc, char*argv[])
{
  vtkSmartPointer<vtkImageReader2Factory> readerFactory =
    vtkSmartPointer<vtkImageReader2Factory>::New();
  vtkImageReader2 * imageReader = readerFactory->CreateImageReader2(argv[1]);
  imageReader->SetFileName(argv[1]);
  imageReader->Update();

  vtkSmartPointer<vtkImageDataToStructuredGrid> filter =
    vtkSmartPointer<vtkImageDataToStructuredGrid>::New();
  filter->SetInputConnection(imageReader->GetOutputPort());
  filter->Update();

  vtkSmartPointer<vtkXMLStructuredGridWriter> writer =
    vtkSmartPointer<vtkXMLStructuredGridWriter>::New();
  writer->SetInputConnection(filter->GetOutputPort());
  writer->SetFileName("Test.vts");
  writer->Write();

  
  imageReader->Delete();

  return EXIT_SUCCESS;
}
