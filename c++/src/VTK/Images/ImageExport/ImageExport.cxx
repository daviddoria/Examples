#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <vtkImageExport.h>
#include <vtkImageMandelbrotSource.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkImageMandelbrotSource> source =
    vtkSmartPointer<vtkImageMandelbrotSource>::New();
  source->Update();

  vtkSmartPointer<vtkImageExport> imageExport =
    vtkSmartPointer<vtkImageExport>::New();

  return EXIT_SUCCESS;
}
