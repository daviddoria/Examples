#include <vtkSmartPointer.h>
#include <vtkStructuredPointsReader.h>
#include <vtkStructuredPoints.h>

int main ( int argc, char *argv[] )
{
  if(argc != 2) //if the user did not pass the right arguments
    {
    cout << "Required arguments: InputFilename" << endl;
    return EXIT_FAILURE;
    }
    
  vtkstd::string inputFilename = argv[1];
  
  //get all data from the file
  vtkSmartPointer<vtkStructuredPointsReader> reader = 
      vtkSmartPointer<vtkStructuredPointsReader>::New();
  reader->SetFileName(inputFilename.c_str());
  reader->Update();
  
  vtkStructuredPoints* structuredPoints = reader->GetOutput();

  //get the number of points the file contains
  vtkIdType numPoints = structuredPoints->GetNumberOfPoints();

  cout << "There are " << numPoints << " points." << endl;

  return EXIT_SUCCESS;
}
