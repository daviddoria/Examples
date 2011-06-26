#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkBMPReader.h>
#include <vtkBMPWriter.h>

//filter
#include "vtkBinaryRemoveRegions.h"

int main( int argc, char **argv ) 
{
  //read dataset
  vtkSmartPointer<vtkBMPReader> reader = 
      vtkSmartPointer<vtkBMPReader>::New();
  reader->SetFileName(argv[1]);
  reader->SetNumberOfScalarComponents(1);
  reader->Update();

  // manipulate
  vtkSmartPointer<vtkBinaryRemoveRegions> filter = 
      vtkSmartPointer<vtkBinaryRemoveRegions>::New();
  filter->SetInput( reader->GetOutput() );
  filter->Update();
  
  //write
  vtkSmartPointer<vtkBMPWriter> writer = 
      vtkSmartPointer<vtkBMPWriter>::New();
  writer->SetInput( filter->GetOutput() );
  writer->SetFileName(argv[2]);
  writer->Write();

  return EXIT_SUCCESS;
}
