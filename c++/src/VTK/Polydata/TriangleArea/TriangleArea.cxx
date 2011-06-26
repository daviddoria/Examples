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

  for(unsigned int i = 0; i < polydata->GetNumberOfCells(); i++)
    {
    vtkCell* cell = polydata->GetCell(0);
    
    vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
    double p0[3];
    double p1[3];
    double p2[3];
    triangle->GetPoints()->GetPoint(0, p0);
    cout << "p0: " << p0[0] << " " << p0[1] << " " << p0[2] << endl;
    triangle->GetPoints()->GetPoint(1, p1);
    cout << "p1: " << p1[0] << " " << p1[1] << " " << p1[2] << endl;
    triangle->GetPoints()->GetPoint(2, p2);
    cout << "p2: " << p2[0] << " " << p2[1] << " " << p2[2] << endl;
    
    double area = vtkTriangle::TriangleArea(p0, p1, p2);
    
    cout << "area of triangle " << i << ": " << area << endl;
    }
  
}
