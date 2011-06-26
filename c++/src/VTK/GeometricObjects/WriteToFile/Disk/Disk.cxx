#include <vtkDiskSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>

int main(int argc, char *argv[])
{
  //Create a disk
  vtkSmartPointer<vtkDiskSource> diskSource = vtkSmartPointer<vtkDiskSource>::New();
  diskSource->SetInnerRadius(1.0);
  diskSource->SetOuterRadius(2.0);
  diskSource->Update();
  
  //Write the file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInputConnection(diskSource->GetOutputPort());
  writer->SetFileName("disk.vtp");
  writer->Write();
  
  return EXIT_SUCCESS;
}
