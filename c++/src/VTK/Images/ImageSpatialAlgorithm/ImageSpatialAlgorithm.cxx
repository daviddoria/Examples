#include <vtkSmartPointer.h>
#include <vtkObjectFactory.h>
#include <vtkImageData.h>
#include <vtkImageSpatialAlgorithm.h>

class vtkImageTraverse : public vtkImageSpatialAlgorithm
{
public:
  static vtkImageTraverse *New();
  vtkTypeMacro(vtkImageTraverse,vtkImageSpatialAlgorithm);
  
  void SetKernelSize(int size0, int size1, int size2);
};

vtkStandardNewMacro(vtkImageTraverse);

void vtkImageTraverse::SetKernelSize(int size0, int size1, int size2)
{
  this->KernelSize[0] = size0;
  this->KernelMiddle[0] = size0 / 2;
  
  this->KernelSize[1] = size1;
  this->KernelMiddle[1] = size1 / 2;
  
  this->KernelSize[2] = size2;
  this->KernelMiddle[2] = size2 / 2;
}

int main(int, char*[])
{
  vtkSmartPointer<vtkImageData> image =
    vtkSmartPointer<vtkImageData>::New();
  int extent[6] = {0,5,0,5,0,0};
  image->SetExtent(extent);
  image->SetNumberOfScalarComponents(1);
  image->SetScalarTypeToDouble();

  for(int y = extent[2]; y < extent[3]; y++)
    {
    for(int x = extent[0]; x < extent[1]; x++)
      {
      double* pixel = static_cast<double*>(image->GetScalarPointer(x,y,0));
      pixel[0] = x;
      pixel[1] = y;
      }
    }
    
  vtkSmartPointer<vtkImageTraverse> imageTraverse =
    vtkSmartPointer<vtkImageTraverse>::New();
  imageTraverse->SetKernelSize(3,3,1);
    
  return EXIT_SUCCESS;
}