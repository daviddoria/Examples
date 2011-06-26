#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkJPEGWriter.h>
#include <vtkColor.h>
#include <vtkMath.h>

int main( int argc, char *argv[] )
{
  int extent[6] = {0, 9, 0, 9, 0, 0};
  
  vtkSmartPointer<vtkImageData> image = 
    vtkSmartPointer<vtkImageData>::New();
  image->SetExtent(extent);
  image->SetScalarTypeToUnsignedChar();
  image->SetNumberOfScalarComponents(3);
  
  for(unsigned int i = extent[0]; i < extent[1]; i++)
    {
    for(unsigned int j = extent[2]; j < extent[3]; j++)
      {
      vtkColor3ub color;
      color.SetRed(round(vtkMath::Random(0, 255)));
      color.SetGreen(round(vtkMath::Random(0, 255)));
      color.SetBlue(round(vtkMath::Random(0, 255)));
      
      unsigned char* pixel = static_cast<unsigned char*>(image->GetScalarPointer(i,j,0));
      std::cout << "R: " << (int)color.GetRed() << " G: " << (int)color.GetGreen() << " B: " 
		<< (int)color.GetBlue() << std::endl;
      
      pixel[0] = color.GetRed();
      pixel[1] = color.GetGreen();
      pixel[2] = color.GetBlue();
      
      }
    }
  
  vtkSmartPointer<vtkJPEGWriter> writer = 
    vtkSmartPointer<vtkJPEGWriter>::New();
  writer->SetFileName("output.jpg");
  writer->SetInput(image);
  writer->Write();

  return EXIT_SUCCESS;
}