#include <vtkImageAccumulate.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>

void CreateImage(vtkSmartPointer<vtkImageData> image);

int main( int argc, char *argv[] )
{
  vtkSmartPointer<vtkImageData> image = 
    vtkSmartPointer<vtkImageData>::New();
  CreateImage(image);
    
  vtkSmartPointer<vtkImageAccumulate> imageAccumulate = 
    vtkSmartPointer<vtkImageAccumulate>::New();
  imageAccumulate->SetInputConnection(image->GetProducerPort());
  
  int extent[6];
  image->GetExtent(extent);
  imageAccumulate->SetComponentExtent(extent);
  //imageAccumulate->SetComponentExtent(image->GetExtent());
  //imageAccumulate->SetComponentExtent(0,255,0,0,0,0);
  imageAccumulate->SetComponentOrigin(0,0,0);
  imageAccumulate->SetComponentSpacing(1,0,0);
  imageAccumulate->Update();
 
  double mean[3];
  imageAccumulate->GetMean(mean);

  return  EXIT_SUCCESS;
}

void CreateImage(vtkSmartPointer<vtkImageData> image)
{
  // Create an image data
  vtkSmartPointer<vtkImageData> imageData = 
    vtkSmartPointer<vtkImageData>::New();
 
  // Specify the size of the image data
  imageData->SetDimensions(2,3,1);
  imageData->SetNumberOfScalarComponents(1);
  imageData->SetScalarTypeToDouble();
 
  int* dims = imageData->GetDimensions();
  // int dims[3]; // can't do this
 
  std::cout << "Dims: " << " x: " << dims[0] << " y: " << dims[1] << " z: " << dims[2] << std::endl;
 
  // Fill every entry of the image data with "2.0"
  for (int z = 0; z < dims[2]; z++)
    {
    for (int y = 0; y < dims[1]; y++)
      {
      for (int x = 0; x < dims[0]; x++)
        {
        double* pixel = static_cast<double*>(imageData->GetScalarPointer(x,y,z));
        pixel[0] = 2.0;
        }
      }
    }
}