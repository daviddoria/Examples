#include <vtkActor.h>
#include <vtkLookupTable.h>
#include <vtkPolyDataMapper.h>
#include <vtkImageData.h>
#include <vtkIntArray.h>
#include <vtkProperty2D.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkImageTranslateExtent.h>
#include <vtkImageToPolyDataFilter.h>
#include <vtkImageQuantizeRGBToIndex.h>
#include <vtkCamera.h>
#include <vtkPNGReader.h>
#include <vtkTriangleFilter.h>


int main(int argc, char *argv[])
{
  if(argc < 2)
    {
    std::cerr << "Required arguments: filename.png" << std::endl;
    return EXIT_FAILURE;
    }
  vtkSmartPointer<vtkPNGReader> reader =
    vtkSmartPointer<vtkPNGReader>::New();
  reader->SetFileName(argv[1]);
  reader->Update();

  vtkSmartPointer<vtkImageToPolyDataFilter> imageToPolyDataFilter =
    vtkSmartPointer<vtkImageToPolyDataFilter>::New();
  imageToPolyDataFilter->SetInputConnection(reader->GetOutputPort());
  imageToPolyDataFilter->Update();

  vtkSmartPointer<vtkImageQuantizeRGBToIndex> quant =
    vtkSmartPointer<vtkImageQuantizeRGBToIndex>::New();
  quant->SetInputConnection(reader->GetOutputPort());
  quant->SetNumberOfColors(32);
  quant->Update();

  vtkSmartPointer<vtkImageToPolyDataFilter> i2pd =
    vtkSmartPointer<vtkImageToPolyDataFilter>::New();
  i2pd->SetInputConnection(quant->GetOutputPort());
  i2pd->SetLookupTable(quant->GetLookupTable());
  i2pd->SetColorModeToLUT();
  i2pd->SetOutputStyleToPolygonalize();
  i2pd->SetError(0);
  i2pd->DecimationOn();
  i2pd->SetDecimationError(0.0);
  i2pd->SetSubImageSize(25);

  // Need a triangle filter because the polygons are complex and concave
  vtkSmartPointer<vtkTriangleFilter> tf =
    vtkSmartPointer<vtkTriangleFilter>::New();
  tf->SetInputConnection(i2pd->GetOutputPort());

  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(tf->GetOutputPort());

  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // Visualize
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);

  renderer->SetBackground(1, 1, 1);
  renderWindow->SetSize(300, 250);

  vtkSmartPointer<vtkCamera> camera =
    vtkSmartPointer<vtkCamera>::New();
  camera->SetClippingRange(343.331, 821.78);
  camera->SetPosition(-139.802, -85.6604, 437.485);
  camera->SetFocalPoint(117.424, 106.656, -14.6);
  camera->SetViewUp(0.430481, 0.716032, 0.549532);
  camera->SetViewAngle(30);
  renderer->SetActiveCamera(camera);

  interactor->Start();

  return EXIT_SUCCESS;
}