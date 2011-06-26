#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkTriangle.h>
#include <vtkFeatureEdges.h>

int main()
{
  //setup points (geometry)
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  Points->InsertNextPoint ( 0.0, 0.0, 0.0 );
  Points->InsertNextPoint ( 1.0, 0.0, 0.0 );
  Points->InsertNextPoint ( 1.0, 1.0, 0.0 );
  Points->InsertNextPoint ( 0.0, 1.0, 0.0 );

  //create a triangle on the three points in the polydata
  vtkSmartPointer<vtkTriangle> triangle1 = vtkSmartPointer<vtkTriangle>::New();
  triangle1->GetPointIds()->SetId ( 0, 0 );
  triangle1->GetPointIds()->SetId ( 1, 1 );
  triangle1->GetPointIds()->SetId ( 2, 2 );

  vtkSmartPointer<vtkTriangle> triangle2 = vtkSmartPointer<vtkTriangle>::New();
  triangle2->GetPointIds()->SetId ( 0, 2 );
  triangle2->GetPointIds()->SetId ( 1, 3 );
  triangle2->GetPointIds()->SetId ( 2, 0 );

  //add the triangles to the list of triangles
  vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
  triangles->InsertNextCell ( triangle1 );
  triangles->InsertNextCell ( triangle2 );

  //create a polydata object
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  //add the geometry and topology to the polydata
  polydata->SetPoints ( Points );
  polydata->SetPolys ( triangles );

  //write the polydata to a file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName ( "Triangles.vtp" );
  writer->SetInput ( polydata );
  writer->Write();
  
  vtkSmartPointer<vtkFeatureEdges> FeatureEdges = vtkSmartPointer<vtkFeatureEdges>::New();
  FeatureEdges->SetInput(polydata);
  FeatureEdges->BoundaryEdgesOn();
  FeatureEdges->FeatureEdgesOff();
  FeatureEdges->ManifoldEdgesOff();
  FeatureEdges->NonManifoldEdgesOff();
  
  vtkSmartPointer<vtkXMLPolyDataWriter> EdgeWriter = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  EdgeWriter->SetFileName("TriangleEdges.vtp");
  EdgeWriter->SetInput(FeatureEdges->GetOutput());
  EdgeWriter->Write();

}
