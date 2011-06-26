#include <vtkSmartPointer.h>
#include <vtkTriangle.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>

void TriangleInfo ( vtkTriangle* Triangle );
void TriangleInfo (vtkPoints* Points, vtkTriangle* Triangle);

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkTriangle> T = vtkSmartPointer<vtkTriangle>::New();

	//setup points
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  Points->InsertNextPoint ( 1.0, 2.0, 3.0 );
  Points->InsertNextPoint ( 4.0, 5.0, 6.0 );
  Points->InsertNextPoint ( 7.0, 8.0, 9.0 );

  vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
  triangle->GetPointIds()->SetId ( 0, 0 );
  triangle->GetPointIds()->SetId ( 1, 1 );
  triangle->GetPointIds()->SetId ( 2, 2 );

  triangle->SetPoints(Points);
    
  TriangleInfo ( triangle );
    //TriangleInfo (Points, triangle);

  return 0;
}


void TriangleInfo ( vtkTriangle* Triangle )
{
  vtkPoints* Points = Triangle->GetPoints();

  for ( unsigned int i = 0; i < 3; i++ )
  {
    double p[3];
    Points->GetPoint ( i,p );
    std::cout << "p" << i << ": " << p[0] << " " << p[1] << " " << p[2] << std::endl;
  }
}


void TriangleInfo (vtkPoints* Points, vtkTriangle* Triangle)
{
  for ( unsigned int i = 0; i < 3; i++ )
  {
    double p[3];
    unsigned int id = Triangle->GetPointId(i);
    Points->GetPoint(id, p);
    std::cout << "p" << i << ": " << p[0] << " " << p[1] << " " << p[2] << std::endl;
  }
}
