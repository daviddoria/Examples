#include <vtkActor.h>
#include <vtkImageData.h>
#include <vtkIntArray.h>
#include <vtkProperty2D.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkImageReslice.h>
 
int main(int, char *[] )
{

  vtkSmartPointer<vtkImageData> image =
    vtkSmartPointer<vtkImageData>::New();
  image->SetExtent(0, 9, 0, 9, 0, 0);
  image->SetNumberOfScalarComponents(1);
  image->SetScalarTypeToInt();

  int* pixel = static_cast<int*>(image->GetScalarPointer(0,9,0));

  vtkSmartPointer<vtkImageReslice> reslice =
    vtkSmartPointer<vtkImageReslice>::New();
  reslice->SetOutputExtent(0, 9, 0, 100, 0, 0);
  reslice->SetInputConnection(image->GetProducerPort());
  reslice->Update();

  int* pixel2 = static_cast<int*>(reslice->GetOutput()->GetScalarPointer(0,11,0));
  
  return EXIT_SUCCESS;
}