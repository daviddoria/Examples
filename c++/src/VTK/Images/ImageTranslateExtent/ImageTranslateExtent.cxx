#include <vtkActor.h>
#include <vtkImageData.h>
#include <vtkIntArray.h>
#include <vtkProperty2D.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkImageTranslateExtent.h>

int main(int, char *[] )
{
  vtkSmartPointer<vtkImageData> image =
    vtkSmartPointer<vtkImageData>::New();
  image->SetExtent(0, 9, 0, 9, 0, 0);
  image->SetNumberOfScalarComponents(1);
  image->SetScalarTypeToInt();

  vtkSmartPointer<vtkImageTranslateExtent> translateExtent =
    vtkSmartPointer<vtkImageTranslateExtent>::New();
  translateExtent->SetTranslation(3,4,5);
  translateExtent->SetInputConnection(image->GetProducerPort());
  translateExtent->Update();

  int newExtent[6];
  translateExtent->GetOutput()->GetExtent(newExtent);

  std::cout << "New extent: " << newExtent[0] << " " << newExtent[1] << " " << newExtent[2]
            << " " << newExtent[3] << " " << newExtent[4] << " " << newExtent[5] << std::endl;
  return EXIT_SUCCESS;
}