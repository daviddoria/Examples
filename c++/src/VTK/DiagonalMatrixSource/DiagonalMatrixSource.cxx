#include <vtkSmartPointer.h>
#include <vtkDiagonalMatrixSource.h>
#include <vtkArrayPrint.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkDiagonalMatrixSource> diagonalMatrixSource =
      vtkSmartPointer<vtkDiagonalMatrixSource>::New();
  diagonalMatrixSource->SetExtents(5);
  diagonalMatrixSource->SetArrayType(vtkDiagonalMatrixSource::DENSE);
  diagonalMatrixSource->Update();

  diagonalMatrixSource->GetOutput()->GetArray(0)->Print(std::cout);
  //vtkPrintMatrixFormat(std::cout, diagonalMatrixSource->GetOutput()->GetArray(0)->GetPointer());

  std::cout << "Class name: " << diagonalMatrixSource->GetOutput()->GetArray(0)->GetClassName() << std::endl;
  return EXIT_SUCCESS;
}
