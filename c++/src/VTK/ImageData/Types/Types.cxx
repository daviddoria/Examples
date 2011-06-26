#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageCast.h>
#include <vtkImageCanvasSource2D.h>

void UnsignedChar();
void Double();

int main(int argc, char *argv[])
{
  //UnsignedChar();
  Double();
  
  return EXIT_SUCCESS;
}

void UnsignedChar()
{
  vtkSmartPointer<vtkImageCanvasSource2D> imageSource = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  imageSource->SetNumberOfScalarComponents(3);
  imageSource->SetScalarTypeToUnsignedChar();
  imageSource->SetExtent(0, 4, 0, 4, 0, 0);
  //imageSource->SetDrawColor(0, 0, 0);
  imageSource->SetDrawColor(255, 0, 0);
  imageSource->FillBox(0, 4, 0, 4);
  imageSource->Update();
  
  vtkImageData* imageData = imageSource->GetOutput();
  
  int* dims = imageData->GetDimensions();
  cout << "dims: " << dims[0] << " " << dims[1] << endl;
  for (int y = 0; y < dims[1]; y++)
    {
    for (int x = 0; x < dims[0]; x++)
      {
      unsigned char* v = static_cast<unsigned char*>(imageData->GetScalarPointer(x,y,0));
      cout << (int)v[0] << " " << (int)v[1] << " " << (int)v[2] << endl;
      }
    }
}

void Double()
{
  vtkSmartPointer<vtkImageCanvasSource2D> imageSource = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  imageSource->SetNumberOfScalarComponents(3);
  imageSource->SetScalarTypeToDouble();
  imageSource->SetExtent(0, 4, 0, 4, 0, 0);
  imageSource->SetDrawColor(0.5, 0, 0);
  imageSource->FillBox(0, 4, 0, 4);
  imageSource->Update();
  
  vtkImageData* imageData = imageSource->GetOutput();
  
  int* dims = imageData->GetDimensions();
  cout << "dims: " << dims[0] << " " << dims[1] << endl;
  for (int y = 0; y < dims[1]; y++)
    {
    for (int x = 0; x < dims[0]; x++)
      {
      double* v = static_cast<double*>(imageData->GetScalarPointer(x,y,0));
      cout << v[0] << " " << v[1] << " " << v[2] << endl;
      }
    }
    
#if 0
    //changes everything to a zero
  vtkSmartPointer<vtkImageCast> castFilter =
      vtkSmartPointer<vtkImageCast>::New();
  castFilter->SetOutputScalarTypeToUnsignedChar();
  castFilter->SetInput(imageData);
  castFilter->Update();
  
  for (int y = 0; y < dims[1]; y++)
    {
    for (int x = 0; x < dims[0]; x++)
      {
      unsigned char* v = static_cast<unsigned char*>(castFilter->GetOutput()->GetScalarPointer(x,y,0));
      cout << (int)v[0] << " " << (int)v[1] << " " << (int)v[2] << endl;
      }
    }
#endif
                               
                                 
                                 
  for (int y = 0; y < dims[1]; y++)
    {
    for (int x = 0; x < dims[0]; x++)
      {
      double* v = static_cast<double*>(imageData->GetScalarPointer(x,y,0));
      for(unsigned int i = 0; i < 3; i++)
        {
        double newVal = 255.*v[i];
        //cout << "New val: " << newVal << endl;
        imageData->SetScalarComponentFromDouble(x,y,0,i, newVal);
        }
      }
    }

        //changes everything to a zero
  vtkSmartPointer<vtkImageCast> castFilter =
      vtkSmartPointer<vtkImageCast>::New();
  castFilter->SetOutputScalarTypeToUnsignedChar();
  castFilter->SetInput(imageData);
  castFilter->Update();
  
  for (int y = 0; y < dims[1]; y++)
    {
    for (int x = 0; x < dims[0]; x++)
      {
      unsigned char* v = static_cast<unsigned char*>(castFilter->GetOutput()->GetScalarPointer(x,y,0));
      cout << (int)v[0] << " " << (int)v[1] << " " << (int)v[2] << endl;
      }
    }
  
}