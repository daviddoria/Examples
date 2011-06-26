#include <vtkSmartPointer.h>
#include <vtkImageData.h>

int main(int argc, char *argv[])
{
  //create an image data
  vtkSmartPointer<vtkImageData> imageData = 
      vtkSmartPointer<vtkImageData>::New();
  
  //specify the size of the image data
  imageData->SetDimensions(2,3,1);
  imageData->SetNumberOfScalarComponents(1);
  imageData->SetScalarTypeToDouble();
  
  int* dims = imageData->GetDimensions();
  // int dims[3]; // can't do this
  
  std::cout << "Dims: " << " x: " << dims[0] << " y: " << dims[1] << " z: " << dims[2] << std::endl;
  

  return EXIT_SUCCESS;
}
