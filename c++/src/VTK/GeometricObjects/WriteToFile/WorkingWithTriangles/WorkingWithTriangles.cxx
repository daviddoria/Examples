#include "vtkSmartPointer.h"
#include "vtkTriangle.h"
#include "vtkPoints.h"

void TriangleInfo ( vtkTriangle* Triangle );

main (int, char*[])
{

  vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
  triangle->GetPointIds()->SetId ( 0, 0 );
  triangle->GetPointIds()->SetId ( 1, 1 );
  triangle->GetPointIds()->SetId ( 2, 2 );

  /*
  //does the same thing, but performs range checking. 
  //Not necessary, we know we can use 0-2 for a triangle.
  triangle->GetPoints()->InsertPoint ( 0, 1.0, 2.0, 3.0 );
  triangle->GetPoints()->InsertPoint ( 1, 4.0, 5.0, 6.0 );
  triangle->GetPoints()->InsertPoint ( 2, 7.0, 8.0, 9.0 );
  */
  triangle->GetPoints()->SetPoint ( 0, 1.0, 2.0, 3.0 );
  triangle->GetPoints()->SetPoint ( 1, 4.0, 5.0, 6.0 );
  triangle->GetPoints()->SetPoint ( 2, 7.0, 8.0, 9.0 );

  TriangleInfo(triangle);
  return EXIT_SUCCESS;
}

void TriangleInfo ( vtkTriangle* Triangle )
{
  vtkPoints* Points = Triangle->GetPoints();

  for ( unsigned int i = 0; i < 3; i++ )
  {
    double p[3];
    Points->GetPoint ( i,p );
    std::cout << "p" << i << ": " << p[0] << " " << p[1] << " " << p[2]
        << std::endl;
  }
}