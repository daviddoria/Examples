#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkXMLStructuredGridReader.h>
#include <vtkStructuredGrid.h>
#include <vtkSmartPointer.h>

int main ( int argc, char *argv[] )
{
  //ensure a filename was specified
  if ( argc != 2 )
    {
    cout << "Required arguments: InputFilename" << endl;
    return EXIT_FAILURE;
    }

  //get the filename from the command line
  vtkstd::string inputFilename = argv[1];

  //get all data from the file
  vtkSmartPointer<vtkXMLStructuredGridReader> reader = vtkSmartPointer<vtkXMLStructuredGridReader>::New();
  reader->SetFileName ( inputFilename.c_str() );
  reader->Update();
  vtkStructuredGrid* structuredGrid = reader->GetOutput();

  //get the number of points the file contains
  vtkIdType numPoints = structuredGrid->GetNumberOfPoints();

  cout << "There are " << numPoints << " points." << endl;

  //if there are no points, quit
  if ( ! ( numPoints > 0 ) )
    {
    return EXIT_FAILURE;
    }

  //display all of the points
  double point[3];
  for ( vtkIdType i = 0; i < numPoints; i++ )
    {
    structuredGrid->GetPoint ( i, point );
    cout << "Point " << i << ": " << point[0] << " " << point[1] << " " << point[2] << endl;
    }

  return EXIT_SUCCESS;
}
