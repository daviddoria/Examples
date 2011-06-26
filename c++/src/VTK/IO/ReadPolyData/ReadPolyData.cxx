#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkToolkits.h>
#include <vtkstd/string>

int main ( int argc, char *argv[] )
{
  // Ensure a filename was specified
  std::string inputFilename;

  bool hasVTKData = false;
#ifdef VTK_DATA_ROOT
  hasVTKData = true;
#endif
  if(argc != 2 && !hasVTKData) //if the user did not pass the right arguments
    // and doesn't have VTKData
    {
      cout << "Required arguments: InputFilename" << endl;
      return EXIT_FAILURE;
    }
    
  if(argc != 2 && hasVTKData) // the user didn't pass the right arguments, but
    // does have VTKData
    {
    inputFilename = vtkstd::string(VTK_DATA_ROOT) + "/Data/political.vtp";
    cout << "Using file " << inputFilename << " from VTKData." << endl;
    }
    
  if(argc == 2)//the user passed the correct arguments
    {
    inputFilename = argv[1];
    cout << "Using file " << inputFilename << " from command line." << endl;
    }

      
  //get all data from the file
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName ( inputFilename.c_str() );
  reader->Update();
  vtkPolyData* polydata = reader->GetOutput();

  //get the number of points the file contains
  vtkIdType numPoints = polydata->GetNumberOfPoints();

  vtkstd::cout << "There are " << numPoints << " points." << vtkstd::endl;

  //if there are no points, quit
  if ( ! ( numPoints > 0 ) )
    {
    return EXIT_FAILURE;
    }

  //display all of the points
  double point[3];
  for ( vtkIdType i = 0; i < numPoints; i++ )
    {
    polydata->GetPoint ( i, point );
    //cout << "Point " << i << ": " << point[0] << " " << point[1] << " " << point[2] << endl;
    }

  //get the triangles (if there are any)
  unsigned int triangleCounter = 0;
  vtkIdType numPolys = polydata->GetNumberOfPolys();

  cout << "There are " << numPolys << " triangles." << endl;

  if ( numPolys > 0 )
    {
    vtkCellArray* triangleCells = polydata->GetPolys();
    vtkIdType npts;
    vtkIdType *pts;

    while ( triangleCells->GetNextCell ( npts, pts ) )
      {
      //cout << "Triangle " << triangleCounter << " consists of points of index: " << pts[0] << " " << pts[1] << " " << pts[2] << endl;
      triangleCounter++;
      }
    }

  return 0;
}
