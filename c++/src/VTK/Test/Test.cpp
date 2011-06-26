#include <vtkSmartPointer.h>
#include <vtkImageData.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkImageData> image =
    vtkSmartPointer<vtkImageData>::New();
  image->SetExtent(0, 9, 0, 9, 0, 0);
  image->SetNumberOfScalarComponents(1);
  image->SetScalarTypeToInt();

  int* pixel = static_cast<int*>(image->GetScalarPointer(0,9,0));

  image->SetExtent(0, 9, 0, 100, 0, 0);

  int* pixel2 = static_cast<int*>(image->GetScalarPointer(0,11,0));
  
  return EXIT_SUCCESS;
}