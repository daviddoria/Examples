#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkDelimitedTextWriter.h>
#include <vtkSphereSource.h>
#include <vtkVariantArray.h>
#include <vtkTable.h>

int main(int argc, char *argv[])
{
  if(argc != 2)
    {
    std::cout << "Required parameters: OutputFilename" << std::endl;
    return EXIT_FAILURE;
    }

  std::string outputFilename = argv[1];
  
    //construct an empty table
  vtkSmartPointer<vtkTable> table = 
    vtkSmartPointer<vtkTable>::New();
 
  for ( unsigned int i = 0; i < 3; i++ )
    {
    vtkSmartPointer<vtkVariantArray> col = 
      vtkSmartPointer<vtkVariantArray>::New();
 
    col->InsertNextValue ( vtkVariant ( 0.0 ) );
    col->InsertNextValue ( vtkVariant ( 0.0 ) );
    col->InsertNextValue ( vtkVariant ( 0.0 ) );
    table->AddColumn ( col );
    }
 
  //fill the table with values
  unsigned int counter = 0;
  for(vtkIdType r = 0; r < table->GetNumberOfRows(); r++ )
    {
    for(vtkIdType c = 0; c < table->GetNumberOfColumns(); c++ )
      {
      table->SetValue ( r,c, vtkVariant ( counter ) );
      counter++;
      }
    }
    
  vtkSmartPointer<vtkDelimitedTextWriter> writer = 
    vtkSmartPointer<vtkDelimitedTextWriter>::New();
  writer->SetFileName(outputFilename.c_str());
  writer->SetInputConnection(table->GetProducerPort());
  writer->Write();

  return EXIT_SUCCESS;
}
