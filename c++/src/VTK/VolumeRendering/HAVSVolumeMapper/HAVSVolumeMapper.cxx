#include <vtkSmartPointer.h>
#include <vtkHAVSVolumeMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkStructuredPointsReader.h>
#include <vtkSLCReader.h>
#include <vtkStructuredPoints.h>
#include <vtkUnstructuredGrid.h>
#include <vtkThreshold.h>
#include <vtkDataSetTriangleFilter.h>
#include <vtkPiecewiseFunction.h>
#include <vtkColorTransferFunction.h>
#include <vtkVolumeProperty.h>
#include <vtkVolume.h>
#include <vtkContourFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkStdString.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkTesting.h>

// The environment variable VTK_DATA_ROOT will be used.
// Make sure it is set accordingly!

int main(int, char*[])
{
  // Locate VTK_DATA_ROOT
  vtkSmartPointer<vtkTesting> testHelper =
    vtkSmartPointer<vtkTesting>::New();
  std::string dataRoot = testHelper->GetDataRoot();

  // Create the standard renderer, render window, and interactor.
  vtkSmartPointer<vtkRenderer> ren1 =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renWin =
    vtkSmartPointer<vtkRenderWindow>::New();
  renWin->AddRenderer(ren1);
  vtkSmartPointer<vtkRenderWindowInteractor> iren =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  iren->SetRenderWindow(renWin);
  iren->SetDesiredUpdateRate(3);

  // Create the reader for the data.
  // This is the data that will be volume rendered.
  std::string filename = dataRoot + "/Data/ironProt.vtk";

  vtkSmartPointer<vtkStructuredPointsReader> reader =
    vtkSmartPointer<vtkStructuredPointsReader>::New();
  reader->SetFileName(filename.c_str());

  // Create a reader for the other data that will be contoured and
  // displayed as a polygonal mesh.
  filename = dataRoot + "/Data/neghip.slc";

  vtkSmartPointer<vtkSLCReader> reader2 =
    vtkSmartPointer<vtkSLCReader>::New();
  reader2->SetFileName(filename.c_str());

  // Convert from vtkImageData to vtkUnstructuredGrid.
  // Remove any cells where all values are below 80.
  vtkSmartPointer<vtkThreshold> thresh =
    vtkSmartPointer<vtkThreshold>::New();
  thresh->ThresholdByUpper(80);
  thresh->AllScalarsOff();
  thresh->SetInputConnection(reader->GetOutputPort());

  // Make sure we have only tetrahedra.
  vtkSmartPointer<vtkDataSetTriangleFilter> trifilter =
    vtkSmartPointer<vtkDataSetTriangleFilter>::New();
  trifilter->SetInputConnection(thresh->GetOutputPort());

  // Create transfer mapping scalar value to opacity.
  vtkSmartPointer<vtkPiecewiseFunction> opacityTransferFunction =
    vtkSmartPointer<vtkPiecewiseFunction>::New();
  opacityTransferFunction->AddPoint(80.0,  0.0);
  opacityTransferFunction->AddPoint(120.0, 0.2);
  opacityTransferFunction->AddPoint(255.0, 0.2);

  // Create transfer mapping scalar value to color.
  vtkSmartPointer<vtkColorTransferFunction> colorTransferFunction =
    vtkSmartPointer<vtkColorTransferFunction>::New();
  colorTransferFunction->AddRGBPoint(80.0,  0.0, 0.0, 0.0);
  colorTransferFunction->AddRGBPoint(120.0, 0.0, 0.0, 1.0);
  colorTransferFunction->AddRGBPoint(160.0, 1.0, 0.0, 0.0);
  colorTransferFunction->AddRGBPoint(200.0, 0.0, 1.0, 0.0);
  colorTransferFunction->AddRGBPoint(255.0, 0.0, 1.0, 1.0);

  // The property describes how the data will look.
  vtkSmartPointer<vtkVolumeProperty> volumeProperty =
    vtkSmartPointer<vtkVolumeProperty>::New();
  volumeProperty->SetColor(colorTransferFunction);
  volumeProperty->SetScalarOpacity(opacityTransferFunction);
  volumeProperty->ShadeOff();
  volumeProperty->SetInterpolationTypeToLinear();

  // The mapper that renders the volume data.
  vtkSmartPointer<vtkHAVSVolumeMapper> volumeMapper =
    vtkSmartPointer<vtkHAVSVolumeMapper>::New();
  volumeMapper->SetInputConnection(trifilter->GetOutputPort());
  volumeMapper->SetLevelOfDetail(false);
  volumeMapper->SetGPUDataStructures(false);
  volumeMapper->SetKBufferSizeTo2();

  // The volume holds the mapper and the property and can be used to
  // position/orient the volume.
  vtkSmartPointer<vtkVolume> volume =
    vtkSmartPointer<vtkVolume>::New();
  volume->SetMapper(volumeMapper);
  volume->SetProperty(volumeProperty);

  // Contour the second dataset.
  vtkSmartPointer<vtkContourFilter> contour =
    vtkSmartPointer<vtkContourFilter>::New();
  contour->SetValue(0, 80);
  contour->SetInputConnection(reader2->GetOutputPort());

  // Create a mapper for the polygonal data.
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(contour->GetOutputPort());
  mapper->ScalarVisibilityOff();

  // Create an actor for the polygonal data.
  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // First test if mapper is supported
  renWin->SetSize(300, 300);
  renWin->Render();

  int supported = volumeMapper->SupportedByHardware(ren1);

  vtkSmartPointer<vtkTextActor> textActor =
    vtkSmartPointer<vtkTextActor>::New();
  textActor->SetInput("Required Extensions\nNot Supported");
  textActor->SetDisplayPosition( 150, 150 );
  textActor->GetTextProperty()->SetJustificationToCentered();
  textActor->GetTextProperty()->SetFontSize(30);


  if(!supported)
    {
    ren1->AddViewProp(textActor);
    }
  else
    {
    ren1->AddViewProp(actor);
    ren1->AddViewProp(volume);
    }

  ren1->ResetCamera();
  ren1->GetActiveCamera()->Azimuth(20.0);
  ren1->GetActiveCamera()->Elevation(10.0);
  ren1->GetActiveCamera()->Zoom(1.5);

  // Test default settings
  renWin->Render();

  // Test kbuffer size 6
  volumeMapper->SetKBufferSizeTo6();
  renWin->Render();

  // Test GPU Data structures
  volumeMapper->SetGPUDataStructures(true);
  renWin->Render();

  // Test Field Level of Detail
  volumeMapper->SetLevelOfDetail(true);
  volumeMapper->SetLevelOfDetailMethodField();
  renWin->Render();

  // Test Area Level of Detail
  volumeMapper->SetLevelOfDetailMethodArea();
  renWin->Render();

  // Return to default KBuffer size and Level of Detail
  volumeMapper->SetLevelOfDetail(false);
  volumeMapper->SetKBufferSizeTo2();
  renWin->Render();

  iren->Start();

  return EXIT_SUCCESS;
}