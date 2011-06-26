#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkXMLStructuredGridWriter.h>
#include <vtkMath.h>

int main(int argc, char *argv[])
{
  //create a grid
  vtkSmartPointer<vtkStructuredGrid> structuredGrid = 
      vtkSmartPointer<vtkStructuredGrid>::New();
  
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  for(unsigned int i = 0; i < 2*3*1; i++)
    {
    points->InsertNextPoint(vtkMath::Random(0, 1), vtkMath::Random(0, 1), vtkMath::Random(0, 1));
    }
  
  //specify the dimensions of the grid
  structuredGrid->SetDimensions(2,3,1);
  structuredGrid->SetPoints(points);
  
  int* dims = structuredGrid->GetDimensions();
  
  //retrieve the entries from the grid and print them to the screen
  unsigned int counter = 0;
  
  for (int k = 0; k < dims[2]; k++)
    {
    for (int j = 0; j < dims[1]; j++)
      {
      for (int i = 0; i < dims[0]; i++)
        {
        double p[3];
        structuredGrid->GetPoint(counter, p);
        
        double pNew[3];
        structuredGrid->GetPoint(i, j, k, pNew);
        
        cout << "P   : " << p[0] << " " << p[1] << " " << p[2] << endl;
        cout << "PNew: " << pNew[0] << " " << pNew[1] << " " << pNew[2] << endl;
        
        counter++;
        
        cout << endl << endl;
        }
      
      }
    }

}