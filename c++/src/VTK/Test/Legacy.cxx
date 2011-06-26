#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkLookupTable.h>
#include <vtkPointSource.h>
#include <vtkIntArray.h>
#include <vtkKdTree.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

int main(int, char *[])
{
  // Create a point cloud
  vtkSmartPointer<vtkPointSource> pointSource =
    vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(100);
  pointSource->Update();
 
  std::cout << "There are " << pointSource->GetOutput()->GetNumberOfPoints() 
            << " points." << std::endl;
  
  //Create the tree
  vtkSmartPointer<vtkKdTree> kDTree =
      vtkSmartPointer<vtkKdTree>::New();
  //kDTree->SetNumberOfRegionsOrMore(7);
  kDTree->AddDataSet(pointSource->GetOutput());
  kDTree->BuildLocator();
 
  // Order the cells based on a viewing direction
  double direction[3] = {2, 1, 0};
  vtkSmartPointer<vtkIntArray> cellOrder =
    vtkSmartPointer<vtkIntArray>::New();
  kDTree->ViewOrderAllRegionsInDirection(direction, cellOrder);
  
  std::cout << "cellOrder->GetNumberOfTuples(): " << cellOrder->GetNumberOfTuples() << std::endl;
  std::cout << "cellOrder->GetNumberOfComponents(): " << cellOrder->GetNumberOfComponents() << std::endl;
  
  std::cout << "Order of vtkKdTree regions is:\n";
  for(vtkIdType i = 0; i < cellOrder->GetNumberOfTuples() * cellOrder->GetNumberOfComponents(); i++)
    {
    std::cout << cellOrder->GetValue(i) << std::endl;
    }
    
  vtkSmartPointer<vtkPolyData> coloredPoints = 
    vtkSmartPointer<vtkPolyData>::New();
  coloredPoints->ShallowCopy(pointSource->GetOutput());
  
   //create the color map
  vtkSmartPointer<vtkLookupTable> colorLookupTable = 
    vtkSmartPointer<vtkLookupTable>::New();
  colorLookupTable->SetTableRange(0, cellOrder->GetNumberOfTuples());
  colorLookupTable->Build();
 
  //generate the colors for each point based on the color map
  vtkSmartPointer<vtkUnsignedCharArray> colors = 
    vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");
  
  for(vtkIdType i = 0; i < pointSource->GetOutput()->GetNumberOfPoints(); i++)
    {
    double p[3];
    pointSource->GetOutput()->GetPoint(i,p);
    int region = kDTree->GetRegionContainingPoint(p[0], p[1], p[2]);
    double dcolor[3];
    colorLookupTable->GetColor(p[2], dcolor);
    unsigned char color[3];
    for(unsigned int j = 0; j < 3; j++)
      {
      color[j] = 255 * dcolor[j]/1.0;
      }
    colors->InsertNextTupleValue(color);
    }
  
  coloredPoints->GetPointData()->SetScalars(colors);
 
  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(coloredPoints->GetProducerPort());
 
  vtkSmartPointer<vtkActor> actor = 
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetInterpolationToFlat();
  actor->GetProperty()->SetPointSize(4);
  
  //Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = 
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  //Add the actor to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(.3, .6, .3); // Background color green
 
  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}