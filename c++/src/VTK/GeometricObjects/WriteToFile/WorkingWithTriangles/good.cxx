#include <vtkSmartPointer.h>
#include <vtkTriangle.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>

void TriangleInfo ( vtkTriangle* Triangle );

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
  triangle->GetPointIds()->SetId ( 0, 0 );
  triangle->GetPointIds()->SetId ( 1, 1 );
  triangle->GetPointIds()->SetId ( 2, 2 );
    
    //setup points
  triangle->GetPoints()->InsertNextPoint ( 1.0, 2.0, 3.0 );
  triangle->GetPoints()->InsertNextPoint ( 4.0, 5.0, 6.0 );
  triangle->GetPoints()->InsertNextPoint ( 7.0, 8.0, 9.0 );

  TriangleInfo ( triangle );

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