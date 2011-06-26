#include <vtkSmartPointer.h>
#include <vtkDoubleArray.h>
#include <vtkIntArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>

int main ( int argc, char *argv[] )
{
  //create points
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();

  unsigned int numberOfPoints = 3;
  points->InsertNextPoint ( 0.0, 0.0, 0.0 );
  points->InsertNextPoint ( 1.0, 0.0, 0.0 );
  points->InsertNextPoint ( 0.0, 1.0, 0.0 );

  //add the points to a polydata
  vtkSmartPointer<vtkPolyData> polydata = 
      vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints ( points );

  //add distances to each point
  vtkSmartPointer<vtkDoubleArray> distances = vtkSmartPointer<vtkDoubleArray>::New();
  distances->SetNumberOfComponents ( 1 );
  distances->SetName ( "Distances" );

  distances->InsertNextValue ( 1.1 );
  distances->InsertNextValue ( 2.2 );
  distances->InsertNextValue ( 3.3 );

  polydata->GetPointData()->AddArray ( distances );

  //get the distances from the polydata
  vtkDoubleArray* array = vtkDoubleArray::SafeDownCast ( 
      polydata->GetPointData()->GetArray ( "Distances" ) );

  if ( array )
    {
    for ( unsigned int i = 0; i < numberOfPoints; i++ )
      {
      double dist;
      dist = array->GetValue ( i );
      cout << "Distance: " << dist << endl;
      }
    }

  //cast the double distances to ints
  vtkDoubleArray* doubleDistances = vtkDoubleArray::SafeDownCast ( 
      polydata->GetPointData()->GetArray ( "Distances" ) );
  vtkIntArray* intDistances = vtkIntArray::SafeDownCast ( doubleDistances );

  if ( intDistances )
    {
    for ( unsigned int i = 0; i < numberOfPoints; i++ )
      {
      int dist;
      dist = intDistances->GetValue ( i );
      cout << "Distance: " << dist << endl;
      }
    }
  else
    {
    cout << "invalid cast." << endl;
    }

  return EXIT_SUCCESS;
}

