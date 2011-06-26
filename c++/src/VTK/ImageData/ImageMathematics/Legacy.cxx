#include <vtkSmartPointer.h>
#include <vtkJPEGWriter.h>
#include <vtkImageCast.h>
#include <vtkMath.h>
#include <vtkImageData.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageMathematics.h>

int main(int argc, char *argv[])
{
  //create an image
  vtkSmartPointer<vtkImageCanvasSource2D> imageSource1 = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  imageSource1->SetNumberOfScalarComponents(3);
  imageSource1->SetScalarTypeToUnsignedChar();
  imageSource1->SetExtent(0, 4, 0, 4, 0, 0);
  imageSource1->SetDrawColor(3.0, 0, 0);
  imageSource1->FillBox(0, 4, 0, 4);
  imageSource1->Update();
  
  vtkImageData* image1 = imageSource1->GetOutput();
  int* dims = image1->GetDimensions();
  for (int y = 0; y < dims[1]; y++)
    {
    for (int x = 0; x < dims[0]; x++)
      {
      //double* v = static_cast<double*>(image1->GetScalarPointer(x,y,0));
      unsigned char* v = static_cast<unsigned char*>(image1->GetScalarPointer(x,y,0));
      cout << (int)v[0] << " " << (int)v[1] << " " << (int)v[2] << endl;
      }
    }
    
    /*
  //create a second image
  vtkSmartPointer<vtkImageCanvasSource2D> imageSource2 = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  imageSource2->SetNumberOfScalarComponents(3);
  imageSource2->SetScalarTypeToUnsignedChar();
  imageSource2->SetExtent(0, 4, 0, 4, 0, 0);
  imageSource2->SetDrawColor(2.0, 0, 0);
  imageSource2->FillBox(0, 4, 0, 4);
  imageSource2->Update();
  
  vtkImageData* image2 = imageSource2->GetOutput();
  int* dims = image2->GetDimensions();
  for (int y = 0; y < dims[1]; y++)
    {
    for (int x = 0; x < dims[0]; x++)
      {
      unsigned char* v = static_cast<unsigned char*>(image2->GetScalarPointer(x,y,0));
      cout << v[0] << " " << v[1] << " " << v[2] << endl;
      }
    }
    */
    
  vtkSmartPointer<vtkImageMathematics> imageMath = 
      vtkSmartPointer<vtkImageMathematics>::New();
  imageMath->SetOperationToMultiplyByK();
  imageMath->SetConstantK(4.0);
  imageMath->SetInput(image1);
  imageMath->Update();
    
  vtkImageData* output = imageMath->GetOutput();
  int* outputDims = output->GetDimensions();
  for (int y = 0; y < outputDims[1]; y++)
    {
    for (int x = 0; x < outputDims[0]; x++)
      {
      //double* v = static_cast<double*>(image1->GetScalarPointer(x,y,0));
      unsigned char* v = static_cast<unsigned char*>(output->GetScalarPointer(x,y,0));
      cout << (int)v[0] << " " << (int)v[1] << " " << (int)v[2] << endl;
      }
    }
    
  return EXIT_SUCCESS;
}
