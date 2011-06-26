#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageIterator.h>

int main(int, char *[])
{
  // Create an image data
  vtkSmartPointer<vtkImageData> imageData =
      vtkSmartPointer<vtkImageData>::New();

  // Specify the size of the image data
  imageData->SetDimensions(2,3,1);

  // Fill every entry of the image data with "2.0"
  int* dims = imageData->GetDimensions();

  for (int z=0; z<dims[2]; z++)
    {
    for (int y=0; y<dims[1]; y++)
      {
      for (int x=0; x<dims[0]; x++)
        {
        imageData->SetScalarComponentFromDouble(x,y,z,0,2.0);
        }
      }
    }

  int extent[6];
  imageData->GetExtent(extent);

  // Retrieve the entries from the image data and print them to the screen
  vtkImageIterator<double> it(imageData, extent);
  double* val = it.BeginSpan();

  while(!it.IsAtEnd())
    {
    std::cout << "val: " << val[0] << " " << val[1] << " " << val[2] << std::endl;
    it.NextSpan();
    }

  return EXIT_SUCCESS;
}
