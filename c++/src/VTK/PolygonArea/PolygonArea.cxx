#include <vtkPoints.h>
#include <vtkPolygon.h>
#include <vtkTriangle.h>
#include <vtkSmartPointer.h>
 
int main(int argc, char *argv[])
{
  /*
  // triangle
  vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
  triangle->GetPoints()->SetPoint(0, 0.0, 0.0, 0.0);
  triangle->GetPoints()->SetPoint(1, 1.0, 0.0, 0.0);
  triangle->GetPoints()->SetPoint(2, 0.0, 1.0, 0.0);
  
  cout << "area: " << triangle->ComputeArea() << endl;
  */
  
  /*
    //Polygon
  vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
  
  polygon->GetPointIds()->SetNumberOfIds(4);
  polygon->GetPointIds()->SetId(0,0);
  polygon->GetPointIds()->SetId(1,1);
  polygon->GetPointIds()->SetId(2,2);
  polygon->GetPointIds()->SetId(3,3);

  polygon->GetPoints()->SetNumberOfPoints(4);
  polygon->GetPoints()->SetPoint(0, 0.0, 0.0, 0.0);
  polygon->GetPoints()->SetPoint(1, 2.0, 0.0, 0.0);
  polygon->GetPoints()->SetPoint(2, 2.0, 2.0, 0.0);
  polygon->GetPoints()->SetPoint(3, 0.0, 2.0, 0.0);
  
  cout << "area: " << polygon->ComputeArea() << endl;
*/
  
  //Polygon Normal
  vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
  
  polygon->GetPointIds()->SetNumberOfIds(4);
  polygon->GetPointIds()->SetId(0,0);
  polygon->GetPointIds()->SetId(1,1);
  polygon->GetPointIds()->SetId(2,2);
  polygon->GetPointIds()->SetId(3,3);

  polygon->GetPoints()->SetNumberOfPoints(4);
  polygon->GetPoints()->SetPoint(0, 0.0, 0.0, 0.0);
  polygon->GetPoints()->SetPoint(1, 2.0, 0.0, 0.0);
  polygon->GetPoints()->SetPoint(2, 2.0, 2.0, 0.0);
  polygon->GetPoints()->SetPoint(3, 0.0, 2.0, 0.0);
  
  double normal[3];
  vtkPolygon::ComputeNormal(4, polygon->GetPoints(), normal);
  
  return 0;
}
