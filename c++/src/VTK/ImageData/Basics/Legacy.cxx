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
  
  //fill every entry of the image data with "2.0"
  int* dims = imageData->GetDimensions();
  // int dims[3]; // can't do this
  
  cout << "Dims: " << " x: " << dims[0] << " y: " << dims[1] << " z: " << dims[2] << endl;
  
  cout << "Number of points: " << imageData->GetNumberOfPoints() << endl;
  cout << "Number of cells: " << imageData->GetNumberOfCells() << endl;
  
  for (int z = 0; z < dims[2]; z++)
    {
    for (int y = 0; y < dims[1]; y++)
      {
      for (int x = 0; x < dims[0]; x++)
        {
        imageData->SetScalarComponentFromDouble(x,y,z,0,2.0);
        }
      }
    }
  
  //retrieve the entries from the image data and print them to the screen
  for (int z = 0; z < dims[2]; z++)
    {
    for (int y = 0; y < dims[1]; y++)
      {
      for (int x = 0; x < dims[0]; x++)
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
    
      //retrieve the entries from the image data and print them to the screen (better way)
    for (int z = 0; z < dims[2]; z++)
    {
      for (int y = 0; y < dims[1]; y++)
      {
        for (int x = 0; x < dims[0]; x++)
        {
          unsigned char* v = static_cast<unsigned char*>(imageData->GetScalarPointer(x,y,z));
          cout << static_cast<int>(v[0]) << " " << static_cast<int>(v[1]) << " " << static_cast<int>(v[2]) << endl;
        cout << endl;
        }
      }
      cout << endl;
    }
  return EXIT_SUCCESS;
}
