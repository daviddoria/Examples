#include <vtkSmartPointer.h>
#include <vtkDataSetMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>
#include <vtkDoubleArray.h>
#include <vtkImageData.h>
#include <vtkRectilinearGrid.h>
#include <vtkStructuredGrid.h>
#include <vtkXMLImageDataWriter.h>
#include <vtkXMLRectilinearGridWriter.h>
#include <vtkXMLStructuredGridWriter.h>

void ImageData(vtkImageData* data);
void RectilinearGrid(vtkRectilinearGrid* data);
void StructuredGrid(vtkStructuredGrid* data);

int GridSize = 5;

int main(int, char *[])
{
  vtkSmartPointer<vtkImageData> imageData =
    vtkSmartPointer<vtkImageData>::New();
  ImageData(imageData);
  vtkSmartPointer<vtkDataSetMapper> imageDataMapper =
    vtkSmartPointer<vtkDataSetMapper>::New();
  imageDataMapper->SetInputConnection(imageData->GetProducerPort());
  vtkSmartPointer<vtkActor> imageDataActor =
    vtkSmartPointer<vtkActor>::New();
  imageDataActor->GetProperty()->SetRepresentationToWireframe();
  imageDataActor->SetMapper(imageDataMapper);

  vtkSmartPointer<vtkRectilinearGrid> rectilinearGrid =
    vtkSmartPointer<vtkRectilinearGrid>::New();
  RectilinearGrid(rectilinearGrid);
  vtkSmartPointer<vtkDataSetMapper> rectilinearGridMapper =
    vtkSmartPointer<vtkDataSetMapper>::New();
  rectilinearGridMapper->SetInputConnection(rectilinearGrid->GetProducerPort());
  vtkSmartPointer<vtkActor> rectilinearGridActor =
    vtkSmartPointer<vtkActor>::New();
  rectilinearGridActor->GetProperty()->SetRepresentationToWireframe();
  rectilinearGridActor->SetMapper(rectilinearGridMapper);

  vtkSmartPointer<vtkStructuredGrid> structuredGrid =
    vtkSmartPointer<vtkStructuredGrid>::New();
  StructuredGrid(structuredGrid);
  vtkSmartPointer<vtkDataSetMapper> structuredGridMapper =
    vtkSmartPointer<vtkDataSetMapper>::New();
  structuredGridMapper->SetInputConnection(structuredGrid->GetProducerPort());
  vtkSmartPointer<vtkActor> structuredGridActor =
    vtkSmartPointer<vtkActor>::New();
  structuredGridActor->GetProperty()->SetRepresentationToWireframe();
  structuredGridActor->SetMapper(structuredGridMapper);

  // There will be one render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(600, 300);

  // And one interactor
  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double leftViewport[4] = {0.0, 0.0, 0.33, 1.0};
  double centerViewport[4] = {0.33, 0.0, 0.66, 1.0};
  double rightViewport[4] = {0.66, 0.0, 1.0, 1.0};

  // Setup both renderers
  vtkSmartPointer<vtkRenderer> leftRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(leftRenderer);
  leftRenderer->SetViewport(leftViewport);
  leftRenderer->SetBackground(.6, .5, .4);

  vtkSmartPointer<vtkRenderer> centerRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(centerRenderer);
  centerRenderer->SetViewport(centerViewport);
  centerRenderer->SetBackground(.4, .5, .6);

  vtkSmartPointer<vtkRenderer> rightRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(rightRenderer);
  rightRenderer->SetViewport(rightViewport);
  rightRenderer->SetBackground(.5, .4, .6);

  // Add the sphere to the left and the cube to the right
  leftRenderer->AddActor(imageDataActor);
  centerRenderer->AddActor(rectilinearGridActor);
  rightRenderer->AddActor(structuredGridActor);

  leftRenderer->ResetCamera();
  centerRenderer->ResetCamera();
  rightRenderer->ResetCamera();
  //rightRenderer->GetActiveCamera()->Azimuth(30);
  //rightRenderer->GetActiveCamera()->Elevation(30);

  renderWindow->Render();
  interactor->Start();
  return EXIT_SUCCESS;
}

void ImageData(vtkImageData* data)
{

  data->SetNumberOfScalarComponents(1);
  data->SetScalarTypeToDouble();
  data->SetExtent(0,GridSize-1,0,GridSize-1,0,GridSize-1);

  vtkSmartPointer<vtkXMLImageDataWriter> writer =
    vtkSmartPointer<vtkXMLImageDataWriter>::New();
  writer->SetFileName("imagedata.vti");
  writer->SetInputConnection(data->GetProducerPort());
  writer->Write();
}

void RectilinearGrid(vtkRectilinearGrid* data)
{

  data->SetExtent(0,GridSize-1,0,GridSize-1,0,GridSize-1);

  vtkSmartPointer<vtkDoubleArray> xCoords =
    vtkSmartPointer<vtkDoubleArray>::New();
  xCoords->SetNumberOfComponents(1);
  vtkSmartPointer<vtkDoubleArray> yCoords =
    vtkSmartPointer<vtkDoubleArray>::New();
  yCoords->SetNumberOfComponents(1);
  vtkSmartPointer<vtkDoubleArray> zCoords =
    vtkSmartPointer<vtkDoubleArray>::New();
  zCoords->SetNumberOfComponents(1);

  for(unsigned int i = 0; i < GridSize; i++)
    {
    if(i == 0)
      {
      xCoords->InsertNextValue(0);
      yCoords->InsertNextValue(0);
      zCoords->InsertNextValue(0);
      continue;
      }
    double oldX = xCoords->GetValue(i-1);
    double oldY = xCoords->GetValue(i-1);
    double oldZ = xCoords->GetValue(i-1);
    xCoords->InsertNextValue(oldX + i*i);
    yCoords->InsertNextValue(oldY + i*i);
    zCoords->InsertNextValue(oldZ + i*i);
    }

  data->SetXCoordinates(xCoords);
  data->SetYCoordinates(yCoords);
  data->SetZCoordinates(zCoords);

  vtkSmartPointer<vtkXMLRectilinearGridWriter> writer =
    vtkSmartPointer<vtkXMLRectilinearGridWriter>::New();
  writer->SetFileName("rectilineargrid.vtr");
  writer->SetInputConnection(data->GetProducerPort());
  writer->Write();
}

void StructuredGrid(vtkStructuredGrid* data)
{
  data->SetExtent(0,GridSize-1,0,GridSize-1,0,GridSize-1);

  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();

  vtkSmartPointer<vtkTransform> transform =
    vtkSmartPointer<vtkTransform>::New();
  transform->RotateZ(30);

  for(unsigned int k = 0; k < GridSize; k++)
    {
    for(unsigned int j = 0; j < GridSize; j++)
      {
      for(unsigned int i = 0; i < GridSize; i++)
        {
        double p[4];
        p[0] = i;
        p[1] = j;
        p[2] = k;
        p[3] = 1;
        double pout[4];
        transform->MultiplyPoint(p, pout);

        points->InsertNextPoint(pout[0], pout[1], pout[2]);
        }
      }
    }

  data->SetPoints(points);

  vtkSmartPointer<vtkXMLStructuredGridWriter> writer =
    vtkSmartPointer<vtkXMLStructuredGridWriter>::New();
  writer->SetFileName("structuredgrid.vts");
  writer->SetInputConnection(data->GetProducerPort());
  writer->Write();
}
