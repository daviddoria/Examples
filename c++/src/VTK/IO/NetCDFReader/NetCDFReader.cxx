#include <vtkSmartPointer.h>
#include <vtkFloatArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkXMLImageDataWriter.h>
#include <vtkNetCDFReader.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkImageActor.h>
#include <vtkImageCast.h>

int main(int argc, char *argv[])
{
  // Parse command line arguments
  if(argc < 2)
    {
    std::cout << "Usage: " << argv[0]
              << " InputFilename(.nc)" << std::endl;
    return EXIT_FAILURE;
    }

  std::string inputFilename = argv[1];

  // Read JPG file
  vtkSmartPointer<vtkNetCDFReader> reader =
    vtkSmartPointer<vtkNetCDFReader>::New();
  reader->SetFileName(inputFilename.c_str());
  reader->Update();

  vtkSmartPointer<vtkImageData> image =
    vtkSmartPointer<vtkImageData>::New();
  image->ShallowCopy(reader->GetOutput());
  image->SetScalarTypeToUnsignedChar();

  vtkFloatArray* data = vtkFloatArray::SafeDownCast(image->GetPointData()->GetArray("data"));
  vtkSmartPointer<vtkUnsignedCharArray> newData =
    vtkSmartPointer<vtkUnsignedCharArray>::New();
  newData->SetNumberOfComponents(data->GetNumberOfComponents());
  newData->SetName("ImageScalars");

  for(vtkIdType i = 0; i < data->GetNumberOfTuples(); i++)
    {
    newData->InsertNextValue(static_cast<unsigned char>(data->GetValue(i)));
    }

  image->GetPointData()->AddArray(newData);
  image->GetPointData()->SetActiveScalars("ImageScalars");

  {
  vtkSmartPointer<vtkXMLImageDataWriter> writer =
    vtkSmartPointer<vtkXMLImageDataWriter>::New();
  writer->SetInputConnection(image->GetProducerPort());
  writer->SetFileName("test.vti");
  writer->Write();
  }

  // Visualize
  vtkSmartPointer<vtkImageActor> actor =
    vtkSmartPointer<vtkImageActor>::New();
  actor->SetInput(image);

  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderer->SetBackground(1,0,0);
  renderer->AddActor(actor);
  renderer->ResetCamera();

  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();

  renderWindowInteractor->SetInteractorStyle(style);

  renderWindowInteractor->SetRenderWindow(renderWindow);
  renderWindowInteractor->Initialize();

  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}