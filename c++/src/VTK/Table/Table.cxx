#include <vtkTable.h>
#include <vtkVariant.h>
#include <vtkVariantArray.h>
#include <vtkSmartPointer.h>

int main ( int argc, char *argv[] )
{
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
  for ( unsigned int r = 0; r < table->GetNumberOfRows(); r++ )
    {
    for ( unsigned int c = 0; c < table->GetNumberOfColumns(); c++ )
      {
      table->SetValue ( r,c, vtkVariant ( counter ) );
      counter++;
      }
    }

  //print information about the table
  cout << "NumRows: " << table->GetNumberOfRows() << endl;
  cout << "NumCols: " << table->GetNumberOfColumns() << endl;

  //display the table
  table->Dump ( 3 );
  
  //access elements of the table
  for ( unsigned int r = 0; r < table->GetNumberOfRows(); r++ )
    {
    for ( unsigned int c = 0; c < table->GetNumberOfColumns(); c++ )
      {
      vtkVariant v = table->GetValue(r,c);
      cout << "(r,c) = (" << r << "," << c << ") = " << v << endl;
      }
    }
  
  return EXIT_SUCCESS;
}
