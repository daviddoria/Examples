#include <vtkSmartPointer.h>
#include <vtkVectorText.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkLinearExtrusionFilter.h>
#include <vtkSTLWriter.h>
#include <vtkTriangleFilter.h>

int main()
{
  // Create vector text
  vtkSmartPointer<vtkVectorText> vecText = 
    vtkSmartPointer<vtkVectorText>::New();
  vecText->SetText("Text!");
  vecText->Update();
  
  // Apply linear extrusion 
  vtkSmartPointer<vtkLinearExtrusionFilter> extrude = 
      vtkSmartPointer<vtkLinearExtrusionFilter>::New();
  extrude->SetInputConnection( vecText->GetOutputPort());
  extrude->SetExtrusionTypeToNormalExtrusion();
  extrude->SetVector(0, 0, 1 );
  extrude->SetScaleFactor (0.5);
  extrude->Update();
  
  vtkSmartPointer<vtkTriangleFilter> triangleFilter =
    vtkSmartPointer<vtkTriangleFilter>::New();
  triangleFilter->SetInputConnection(extrude->GetOutputPort());
  triangleFilter->Update();
  
  // write an STL file
  vtkstd::string outputFilename = "test.stl";
  vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
  stlWriter->SetFileName(outputFilename.c_str());
  stlWriter->SetInputConnection(triangleFilter->GetOutputPort());
  stlWriter->Write();

  return EXIT_SUCCESS;
}
