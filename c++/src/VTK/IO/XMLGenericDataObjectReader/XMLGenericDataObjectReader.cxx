#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>

#include <vtkXMLGenericDataObjectReader.h>

int main(int argc, char *argv[])
{
  //ensure a filename was specified
  if(argc < 2)
    {
    std::cerr << "Required arguments: InputFilename" << std::endl;
    return EXIT_FAILURE;
    }

  //get the filename from the command line
  vtkstd::string inputFilename = argv[1];

  //get all data from the file
  vtkSmartPointer<vtkXMLGenericDataObjectReader> reader = 
      vtkSmartPointer<vtkXMLGenericDataObjectReader>::New();
  reader->SetFileName(inputFilename.c_str());
  reader->Update();
  
  //all of the standard data types can be checked and obtained like this:
  if(vtkPolyData::SafeDownCast(reader->GetOutput()))
    {
    std::cout << "File is a polydata" << std::endl;
    }
  else if(vtkUnstructuredGrid::SafeDownCast(reader->GetOutput()))
    {
    std::cout << "File is an unstructured grid" << std::endl;
    }  
  return EXIT_SUCCESS;
}
