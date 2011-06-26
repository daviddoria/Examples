#include <vtkSmartPointer.h>
#include <vtkImageData.h>

void GreyscaleImage();
void ColorImage();

int main(int, char *[])
{
  //GreyscaleImage();
  ColorImage();

  return EXIT_SUCCESS;
}

void GreyscaleImage()
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
  
  std::cout << "Number of points: " << imageData->GetNumberOfPoints() << std::endl;
  std::cout << "Number of cells: " << imageData->GetNumberOfCells() << std::endl;
  
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
  
  // Retrieve the entries from the image data and print them to the screen
  for (int z = 0; z < dims[2]; z++)
    {
    for (int y = 0; y < dims[1]; y++)
      {
      for (int x = 0; x < dims[0]; x++)
        {
        double* pixel = static_cast<double*>(imageData->GetScalarPointer(x,y,z));
        // do something with v
        std::cout << pixel[0] << " ";
        }
      std::cout << std::endl;
      }
    std::cout << std::endl;
    }

}

void ColorImage()
{
  
  // Create an image data
  vtkSmartPointer<vtkImageData> imageData = 
      vtkSmartPointer<vtkImageData>::New();
  
  // Specify the size of the image data
  imageData->SetDimensions(2,3,1);
  imageData->SetNumberOfScalarComponents(3);
  imageData->SetScalarTypeToDouble();
  
  int* dims = imageData->GetDimensions();
  // int dims[3]; // can't do this
  
  std::cout << "Dims: " << " x: " << dims[0] << " y: " << dims[1] << " z: " << dims[2] << std::endl;
  
  std::cout << "Number of points: " << imageData->GetNumberOfPoints() << std::endl;
  std::cout << "Number of cells: " << imageData->GetNumberOfCells() << std::endl;
  
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
  
  // Retrieve the entries from the image data and print them to the screen
  for (int z = 0; z < dims[2]; z++)
    {
    for (int y = 0; y < dims[1]; y++)
      {
      for (int x = 0; x < dims[0]; x++)
        {
        double* pixel = static_cast<double*>(imageData->GetScalarPointer(x,y,z));
        // do something with v
        std::cout << pixel[0] << " ";
        }
      std::cout << std::endl;
      }
    std::cout << std::endl;
    }

}
