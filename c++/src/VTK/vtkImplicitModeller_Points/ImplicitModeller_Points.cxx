#include <vtkSmartPointer.h>
#include <vtkImplicitModeller.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkXMLImageDataWriter.h>
#include <vtkVertexGlyphFilter.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  
  vtkSmartPointer<vtkPolyData> points = 
      vtkSmartPointer<vtkPolyData>::New();
  points->SetPoints(sphereSource->GetOutput()->GetPoints());
  
  
  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = 
      vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInput(points);
  glyphFilter->Update();
  
  double bounds[6];
  points->GetBounds(bounds);
  
  vtkSmartPointer<vtkImplicitModeller> implicitModeller = 
      vtkSmartPointer<vtkImplicitModeller>::New();
  implicitModeller->SetSampleDimensions(20,20,20);
  implicitModeller->SetModelBounds(bounds);
      
  implicitModeller->SetInputConnection(glyphFilter->GetOutputPort());
  //implicitModeller->SetInput(points);
  implicitModeller->Update();

  
  vtkSmartPointer<vtkXMLImageDataWriter> writer = 
      vtkSmartPointer<vtkXMLImageDataWriter>::New();
  writer->SetInputConnection(implicitModeller->GetOutputPort());
  writer->SetFileName("voxel2.vti");
  writer->Write();
  
  return EXIT_SUCCESS;
}
