#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkColorTransferFunction.h>
#include <vtkContourFilter.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkShepardMethod.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  // Create a set of vertices (polydata)
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0.0, 0.0, 0.0);
  points->InsertNextPoint(1.0, 0.0, 0.0);
  points->InsertNextPoint(3.0, 0.0, 0.0);

  vtkSmartPointer<vtkCellArray> verts = 
      vtkSmartPointer<vtkCellArray>::New();
  verts->InsertNextCell(3);
  verts->InsertCellPoint(0);
  verts->InsertCellPoint(1);
  verts->InsertCellPoint(2);

  // Create a scalar array for the pointdata, each value represents the distance
  // of the vertices from the first vertex
  vtkSmartPointer<vtkFloatArray> distances = 
      vtkSmartPointer<vtkFloatArray>::New();
  distances->SetNumberOfComponents(1);
  distances->SetName("Distances"); 
  distances->InsertNextValue(0.0);
  distances->InsertNextValue(1.0);
  distances->InsertNextValue(3.0);

  vtkSmartPointer<vtkPolyData> polydata = 
      vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  polydata->SetVerts(verts);
  polydata->GetPointData()->SetScalars(distances);

  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> vertsMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  vertsMapper->ScalarVisibilityOff();
  vertsMapper->SetInputConnection(polydata->GetProducerPort());
  vtkSmartPointer<vtkActor> vertsActor =
      vtkSmartPointer<vtkActor>::New();
  vertsActor->SetMapper(vertsMapper);
  vertsActor->GetProperty()->SetColor(1,0,0);
  vertsActor->GetProperty()->SetPointSize(3);

  // Create a shepard filter to interpolate the vertices over a regularized image grid
  vtkSmartPointer<vtkShepardMethod> shepard = 
      vtkSmartPointer<vtkShepardMethod>::New();
  shepard->SetInput(polydata);
  shepard->SetSampleDimensions(100,2,2);
  shepard->SetModelBounds(0,3,-1,1,-1,1);
  shepard->SetMaximumDistance(.5);

  // Contour the shepard generated image at 4 isovalues
  // The accuracy of the results are highly dependent on how the shepard filter is set up
  vtkSmartPointer<vtkContourFilter> contourFilter = 
      vtkSmartPointer<vtkContourFilter>::New();
  contourFilter->SetNumberOfContours(4);
  contourFilter->SetValue(0, 0.5);
  contourFilter->SetValue(1, 1.5);
  contourFilter->SetValue(2, 2.0);
  contourFilter->SetValue(4, 2.5);
  contourFilter->SetInputConnection(shepard->GetOutputPort());
  contourFilter->Update();

  //Create a mapper and actor for the resulting isosurfaces
  vtkSmartPointer<vtkPolyDataMapper> contourMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  contourMapper->SetInputConnection(contourFilter->GetOutputPort());
  contourMapper->ScalarVisibilityOn();
  contourMapper->SetColorModeToMapScalars();

  vtkSmartPointer<vtkActor> contourActor =
      vtkSmartPointer<vtkActor>::New();
  contourActor->SetMapper(contourMapper);

  // Report the results of the interpolation
  double *range = contourFilter->GetOutput()->GetScalarRange();

  cout << "contour output scalar range: " << range[0] << ", " << range[1] << endl;

  vtkIdType nCells = contourFilter->GetOutput()->GetNumberOfCells();
  double bounds[6];
  for( vtkIdType i = 0; i < nCells; ++i )
    {
      if(i%2) // each isosurface value only has 2 cells to report on the odd ones
        {
        contourFilter->GetOutput()->GetCellBounds(i,bounds);
        cout << "cell " << i << ", x position: " << bounds[0] << endl;
        }
    }

  // Create a transfer function to color the isosurfaces
  vtkSmartPointer<vtkColorTransferFunction> lut = 
    vtkSmartPointer<vtkColorTransferFunction>::New();
  lut->SetColorSpaceToRGB();
  lut->AddRGBPoint(range[0],0,1,0);
  lut->AddRGBPoint(range[1],1,0,1);
  lut->SetScaleToLinear();

  contourMapper->SetLookupTable( lut );

  // Create a renderer, render window and interactor
  vtkSmartPointer<vtkRenderer> renderer =
      vtkSmartPointer<vtkRenderer>::New();
  renderer->GradientBackgroundOn();
  renderer->SetBackground(0,0,0);
  renderer->SetBackground2(1,1,1);

  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  renderer->AddActor(contourActor);
  renderer->AddActor(vertsActor);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderWindow->Render();

  renderWindowInteractor->Initialize();
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
