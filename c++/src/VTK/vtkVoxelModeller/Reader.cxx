#include <vtkSmartPointer.h>
#include <vtkXMLImageDataReader.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkXMLImageDataReader> reader = 
      vtkSmartPointer<vtkXMLImageDataReader>::New();
  reader->SetFileName("test.vti");
  reader->Update();
  
  return EXIT_SUCCESS;
}
