#include <vtkSmartPointer.h>
#include <vtkVoxelModeller.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkImageCast.h>
#include <vtkXMLImageDataWriter.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
    
  double bounds[6];
  sphereSource->GetOutput()->GetBounds(bounds);
  
  
  vtkSmartPointer<vtkVoxelModeller> voxelModeller = 
      vtkSmartPointer<vtkVoxelModeller>::New();
  voxelModeller->SetSampleDimensions(50,50,50);
  voxelModeller->SetMaximumDistance(0.1);
  voxelModeller->SetModelBounds(bounds);
      
  voxelModeller->SetInputConnection(sphereSource->GetOutputPort());
  voxelModeller->Update();

  vtkSmartPointer<vtkXMLImageDataWriter> writer = 
      vtkSmartPointer<vtkXMLImageDataWriter>::New();

  writer->SetInputConnection(voxelModeller->GetOutputPort());
  writer->SetFileName("test.vti");
  writer->Write();
  
  return EXIT_SUCCESS;
}
