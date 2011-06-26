#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLImageDataWriter.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkMath.h>
#include <vtkCellArray.h>
#include <vtkThresholdPoints.h>
#include <vtkVoxel.h>
#include <vtkCell.h>
#include <vtkImageData.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkPointSource.h>
#include <vtkUnstructuredGrid.h>

void FindTheCellThatAPointIsIn();
void GetPointsInCell();
void CheckPointInsideCell();

int main(int, char *[])
{
  //FindTheCellThatAPointIsIn();
  //CheckPointInsideCell();
  GetPointsInCell();
  
  return EXIT_SUCCESS;
}

void FindTheCellThatAPointIsIn()
{
 //create an image data
  vtkSmartPointer<vtkImageData> imageData = 
      vtkSmartPointer<vtkImageData>::New();
  
  //specify the size of the image data
  imageData->SetDimensions(3,3,2);
  imageData->SetSpacing(1.0, 1.0, 1.0);
  imageData->SetOrigin(0.0, 0.0, 0.0);
  
  cout << "Number of points: " << imageData->GetNumberOfPoints() << endl;
  cout << "Number of cells: " << imageData->GetNumberOfCells() << endl;
  
  {
  double x[3] = {.5, .5, .5};
  int ijk[3];
  double pcoords[3];
  imageData->ComputeStructuredCoordinates(x, ijk, pcoords);

  cout << "ijk: " << ijk[0] << " " << ijk[1] << " " << ijk[2] << endl;
  }
  
  {
  double x[3] = {1.5, .5, .5};
  int ijk[3];
  double pcoords[3];
  imageData->ComputeStructuredCoordinates(x, ijk, pcoords);

  cout << "ijk: " << ijk[0] << " " << ijk[1] << " " << ijk[2] << endl;
  }
  
  {
  double x[3] = {1.5, 1.5, .5};
  int ijk[3];
  double pcoords[3];
  imageData->ComputeStructuredCoordinates(x, ijk, pcoords);

  cout << "ijk: " << ijk[0] << " " << ijk[1] << " " << ijk[2] << endl;
  }
}

void GetPointsInCell()
{
   //create an image data
  vtkSmartPointer<vtkImageData> imageData = 
      vtkSmartPointer<vtkImageData>::New();
  
  //specify the size of the image data
  imageData->SetDimensions(3,3,2);
  imageData->SetSpacing(1.0, 1.0, 1.0);
  imageData->SetOrigin(0.0, 0.0, 0.0);
  
  {
  vtkSmartPointer<vtkXMLImageDataWriter> writer = 
      vtkSmartPointer<vtkXMLImageDataWriter>::New();
  writer->SetInput(imageData);
  writer->SetFileName("grid.vti");
  writer->Write();
  }
  
  vtkMath::RandomSeed(time(NULL));
  
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(100);
  pointSource->Update();
  
  {
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInputConnection(pointSource->GetOutputPort());
  writer->SetFileName("points.vtp");
  writer->Write();
  }
  
  for(unsigned int i = 0; i < 10; i++)
    {
    double p[3];
    pointSource->GetOutput()->GetPoint(i,p);
    cout << "P: " << p[0] << " " << p[1] << " " << p[2] << endl;
    }
  
  int ijk[3] = {0,0,0};
  vtkIdType id = imageData->ComputeCellId(ijk);
  vtkVoxel* voxel = vtkVoxel::SafeDownCast(imageData->GetCell(id));
  
  vtkIdList* ids = voxel->GetPointIds();
  cout << "There are " << ids->GetNumberOfIds() << " ids : " << endl;

  vtkSmartPointer<vtkUnstructuredGrid> ug =
      vtkSmartPointer<vtkUnstructuredGrid>::New();
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();

  
  vtkSmartPointer<vtkVoxel> newVoxel = 
      vtkSmartPointer<vtkVoxel>::New();
  newVoxel->GetPointIds()->SetId(0, 0);
  newVoxel->GetPointIds()->SetId(1, 1);
  newVoxel->GetPointIds()->SetId(2, 2);
  newVoxel->GetPointIds()->SetId(3, 3);
  newVoxel->GetPointIds()->SetId(4, 4);
  newVoxel->GetPointIds()->SetId(5, 5);
  newVoxel->GetPointIds()->SetId(6, 6);
  newVoxel->GetPointIds()->SetId(7, 7);
  
  vtkSmartPointer<vtkCellArray> cells = 
      vtkSmartPointer<vtkCellArray>::New();
  
  cells->InsertNextCell(newVoxel);
  for(unsigned int i = 0; i < ids->GetNumberOfIds(); i++)
    {
    double p[3];
    imageData->GetPoint(ids->GetId(i), p);
    points->InsertNextPoint(p);
    }
  ug->SetPoints(points);
  //ug->InsertNextCell(voxel->GetCellType(),voxel->GetPointIds());
  //ug->SetCells(cells);
  ug->SetCells(voxel->GetCellType(), cells);
  
  vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter = 
      vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
  surfaceFilter->SetInput(ug);
  surfaceFilter->Update();
  
  vtkSmartPointer<vtkSelectEnclosedPoints> enclosedFilter =
      vtkSmartPointer<vtkSelectEnclosedPoints>::New();
  enclosedFilter->SetInputConnection(pointSource->GetOutputPort());
  enclosedFilter->SetSurfaceConnection(surfaceFilter->GetOutputPort());
  enclosedFilter->Update();
  
  //extract the enclosed points
  vtkSmartPointer<vtkThresholdPoints> threshold = 
      vtkSmartPointer<vtkThresholdPoints>::New();
  threshold->SetInputConnection(enclosedFilter->GetOutputPort());
  threshold->SetInputArrayToProcess(0,0,0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "SelectedPoints");
  
  threshold->ThresholdByUpper(0.9); //grab all the points that are marked "1"
  threshold->Update();
  
  cout << "thresholded points: " << threshold->GetOutput()->GetNumberOfPoints() << endl;
  
  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = 
      vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(threshold->GetOutputPort());
  glyphFilter->Update();
  
  {
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInputConnection(glyphFilter->GetOutputPort());
  writer->SetFileName("inside.vtp");
  writer->Write();
  }
}

void CheckPointInsideCell()
{
   //create an image data
  vtkSmartPointer<vtkImageData> imageData = 
      vtkSmartPointer<vtkImageData>::New();
  
  //specify the size of the image data
  imageData->SetDimensions(3,3,2);
  imageData->SetSpacing(1.0, 1.0, 1.0);
  imageData->SetOrigin(0.0, 0.0, 0.0);
  
  int ijk[3] = {0,0,0};
  
  vtkIdType id = imageData->ComputeCellId(ijk);
  //cout << imageData->GetCellType(id) << endl;
  cout << imageData->GetCell(id)->GetClassName() << endl;
  
  double x[3] = {.5,.5,.5};
  
  //int inside = imageData->GetCell(id)->EvaluatePosition(x, NULL, NULL, NULL, NULL, NULL);
  int subId;
  double dist2;
  double pcoords[3];
  double weights[8];
  double cp[3];
  int inside = imageData->GetCell(id)->EvaluatePosition(x, cp, subId, pcoords, dist2, weights);
  cout << "inside? " << inside << endl;
  
}
