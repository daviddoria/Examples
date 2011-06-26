#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>

int main()
{
  unsigned char Red[3] = {255, 0, 0};
  vtkSmartPointer<vtkUnsignedCharArray> Colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  Colors->SetNumberOfComponents ( 3 );
  Colors->SetName ( "Colors" );
   
  //create an image data
  vtkSmartPointer<vtkImageData> ImageData = vtkSmartPointer<vtkImageData>::New();
  
  //specify the size of the image data
  ImageData->SetDimensions(2,3,1);
  ImageData->SetNumberOfScalarComponents(2);
  
  //fill every entry of the image data with "1.1" and "2.2"
  int* dims = ImageData->GetDimensions();
  
  for (int z=0; z<dims[2]; z++)
  {
    for (int y=0; y<dims[1]; y++)
    {
      for (int x=0; x<dims[0]; x++)
      {
        ImageData->SetScalarComponentFromDouble(x,y,z,0,1.1);
        ImageData->SetScalarComponentFromDouble(x,y,z,1,2.2);
        Colors->InsertNextTupleValue(Red);
      }
    }
  }
  
  ImageData->GetPointData()->AddArray ( Colors );
  
  vtkUnsignedCharArray* ColorsRetrieved = vtkUnsignedCharArray::SafeDownCast ( ImageData->GetPointData()->GetArray("Colors"));
  
  unsigned int counter = 0;
  //retrieve the entries from the image data and print them to the screen
  for (int z=0; z<dims[2]; z++)
  {
    for (int y=0; y<dims[1]; y++)
    {
      for (int x=0; x<dims[0]; x++)
      {
    // zero is the component, add another loop if you have more
    // than one component
        double v1 = ImageData->GetScalarComponentAsDouble(x,y,z,0);
        double v2 = ImageData->GetScalarComponentAsDouble(x,y,z,1);
        // do something with v
        vtkstd::cout << v1 << " " << v2 << " ";
        
        unsigned char c[3];
        ColorsRetrieved->GetTupleValue(counter, c);
        vtkstd::cout << "c: " << (int)c[0] << " " << (int)c[1] << " " << (int)c[2] << vtkstd::endl;
        
        counter++;
      }
      vtkstd::cout << vtkstd::endl;
    }
    vtkstd::cout << vtkstd::endl;
  }
  
}
