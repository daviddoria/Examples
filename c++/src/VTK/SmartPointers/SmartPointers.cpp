#include <vtkFloatArray.h>
#include <vtkSmartPointer.h>

#include <iostream>

void WithSmartPointers();
void WithoutSmartPointers();
void DeleteSmartPointers();

int main(int, char *[])
{
  //WithSmartPointers();
  //WithoutSmartPointers();
	DeleteSmartPointers();
	return EXIT_SUCCESS;
}

void WithSmartPointers()
{
  vtkSmartPointer<vtkFloatArray> distances =
    vtkSmartPointer<vtkFloatArray>::New();
}

void WithoutSmartPointers()
{
  vtkFloatArray* distances = vtkFloatArray::New();
  distances->Delete();
}

void DeleteSmartPointers()
{
  vtkSmartPointer<vtkFloatArray> distances =
    vtkSmartPointer<vtkFloatArray>::New();
  std::cout << distances->GetNumberOfComponents() << std::endl;

  distances.TakeReference(vtkFloatArray::New());
  std::cout << distances->GetNumberOfComponents() << std::endl;
}