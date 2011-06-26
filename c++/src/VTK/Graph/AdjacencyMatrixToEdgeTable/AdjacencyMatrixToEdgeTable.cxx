#include <vtkSmartPointer.h>
#include <vtkDenseArray.h>
#include <vtkArrayToTable.h>
#include <vtkTable.h>
#include <vtkArrayData.h>
#include <vtkAdjacencyMatrixToEdgeTable.h>
#include <vtkArrayPrint.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkDenseArray<double> > array =
    vtkSmartPointer<vtkDenseArray<double> >::New();

  array->Resize(3,3);

  for(vtkIdType i = 0; i < array->GetExtents()[0].GetEnd(); i++)
    {
    for(vtkIdType j = 0; j < array->GetExtents()[1].GetEnd(); j++)
      {
      array->SetValue(i, j, (i+j)*10);
      }
    }
  vtkPrintMatrixFormat(std::cout, array.GetPointer());

  vtkSmartPointer<vtkArrayData> arrayData =
    vtkSmartPointer<vtkArrayData>::New();
  arrayData->AddArray(array);

  vtkSmartPointer<vtkAdjacencyMatrixToEdgeTable> adjacencyMatrixToEdgeTable =
    vtkSmartPointer<vtkAdjacencyMatrixToEdgeTable>::New();
  adjacencyMatrixToEdgeTable->SetInputConnection(arrayData->GetProducerPort());
  adjacencyMatrixToEdgeTable->Update();

  adjacencyMatrixToEdgeTable->GetOutput()->Dump();

  return EXIT_SUCCESS;
}