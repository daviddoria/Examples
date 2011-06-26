#include <vtkSmartPointer.h>
#include <vtkImageData.h>

int main()
{
  //create an image data
  vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
  
  //specify the size of the image data
  imageData->SetExtent(-5, 5, -5, 5, -5, 5);
  imageData->SetNumberOfScalarComponents(1);
  
  //fill every entry of the image data with "2.0"
  int* extent = imageData->GetExtent();
  
  //vtkstd::cout << "Dims: " << " x: " << dims[0] << " y: " << dims[1] << " z: " << dims[2] << vtkstd::endl;
  cout << "Extent: " << " xmin: " << extent[0] << " xmax: " << extent[1] << 
      " ymin: " << extent[2] << " ymax: " << extent[3] <<
      " zmin: " << extent[4] << " zmax: " << extent[5] << endl;
  
  
  for (int z = extent[4]; z < extent[5]; z++)
    {
    for (int y = extent[2]; y < extent[3]; y++)
      {
      for (int x = extent[0]; x < extent[1]; x++)
        {
        imageData->SetScalarComponentFromDouble(x,y,z,0,2.0);
        }
      }
    }
  
  //retrieve the entries from the image data and print them to the screen
  for (int z = extent[4]; z < extent[5]; z++)
    {
    for (int y = extent[2]; y < extent[3]; y++)
      {
      for (int x = extent[0]; x < extent[1]; x++)
        {
        // zero is the component, add another loop if you have more
        // than one component
        double v = imageData->GetScalarComponentAsDouble(x,y,z,0);
        // do something with v
        cout << v << " ";
        }
      cout << endl;
      }
    cout << endl;
    }
  
}
