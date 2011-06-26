#include <vtkConeSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkConeSource> ConeSource = vtkSmartPointer<vtkConeSource>::New();
  vtkPolyData* Cone = ConeSource->GetOutput();
    
  //write the file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInput(Cone);

  writer->SetFileName("cone.vtp");
  writer->Write();
  
	return 0;
}
