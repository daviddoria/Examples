#include <vtkSmartPointer.h>
#include <vtkOBJReader.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCell.h>
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkXMLPolyDataWriter.h>

int main(int argc, char *argv[])
{
  //parse command line arguments
  if(argc != 3)
    {
    vtkstd::cout << "Required arguments: inputFilename outputFilename" << vtkstd::endl;
    return EXIT_FAILURE;
    }
  
  vtkstd::string filename = argv[1];
  vtkSmartPointer<vtkOBJReader> reader = 
      vtkSmartPointer<vtkOBJReader>::New();
  reader->SetFileName(filename.c_str());
  reader->Update();
    
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(argv[2]);
  writer->SetInputConnection(reader->GetOutputPort());
  writer->Write();
  
  return EXIT_SUCCESS;
}
