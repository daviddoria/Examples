#include <vtkSmartPointer.h>
#include <vtkPolygon.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkCellArray.h>
#include <vtkLinearExtrusionFilter.h>

int main()
{
  // Setup four points
  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0.0, 0.0, 0.0);
  points->InsertNextPoint(1.0, 0.0, 0.0);
  points->InsertNextPoint(1.0, 1.0, 0.0);
  points->InsertNextPoint(0.0, 1.0, 0.0);
 
  // Create the polygon
  vtkSmartPointer<vtkPolygon> polygon =
    vtkSmartPointer<vtkPolygon>::New();
  polygon->GetPointIds()->SetNumberOfIds(4); //make a quad
  polygon->GetPointIds()->SetId(0, 0);
  polygon->GetPointIds()->SetId(1, 1);
  polygon->GetPointIds()->SetId(2, 2);
  polygon->GetPointIds()->SetId(3, 3);
 
  // Add the polygon to a list of polygons
  vtkSmartPointer<vtkCellArray> polygons =
    vtkSmartPointer<vtkCellArray>::New();
  polygons->InsertNextCell(polygon);
 
  // Create a PolyData
  vtkSmartPointer<vtkPolyData> polygonPolyData =
    vtkSmartPointer<vtkPolyData>::New();
  polygonPolyData->SetPoints(points);
  polygonPolyData->SetPolys(polygons);
  
  
  // Apply linear extrusion 
  vtkSmartPointer<vtkLinearExtrusionFilter> extrude = 
      vtkSmartPointer<vtkLinearExtrusionFilter>::New();
  extrude->SetInputConnection( polygonPolyData->GetProducerPort());
  extrude->SetExtrusionTypeToNormalExtrusion();
  extrude->SetVector(0, 0, 1 );
  extrude->SetScaleFactor (0.5);
  extrude->Update();
  /*
  vtkSmartPointer<vtkTriangleFilter> triangleFilter =
    vtkSmartPointer<vtkTriangleFilter>::New();
  triangleFilter->SetInputConnection(extrude->GetOutputPort());
  triangleFilter->Update();
  */
  
  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInputConnection(extrude->GetOutputPort());
  writer->SetFileName("test.vtp");
  writer->Write();
  
  return EXIT_SUCCESS;
}
