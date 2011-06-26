#include <vtkSmartPointer.h>
#include <vtkInformation.h>
#include <vtkPointSource.h>
#include <vtkSphereSource.h>
#include <vtkExtractSelection.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkIdTypeArray.h>
#include <vtkDataSetMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataNormals.h>
#include <vtkPointData.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkSphereSource> pointSource =
    vtkSmartPointer<vtkSphereSource>::New();
//  pointSource->SetNumberOfPoints(50);
  pointSource->Update();
  
  vtkSmartPointer<vtkPolyDataNormals> polyDataNormals = 
    vtkSmartPointer<vtkPolyDataNormals>::New();
  polyDataNormals->SetInputConnection(pointSource->GetOutputPort());
  polyDataNormals->Update();
  
  vtkDataArray* normalsBefore = vtkDataArray::SafeDownCast(polyDataNormals->GetOutput()->GetPointData()->GetNormals());
  if(!normalsBefore)
    {
    std::cerr << "no normals before!" << std::endl;
    }
    else
    {
      std::cout << "Has normals before!" << std::endl;
    }
  std::cout << "There are " << pointSource->GetOutput()->GetNumberOfPoints()
            << " input points." << std::endl;

  vtkSmartPointer<vtkIdTypeArray> ids =
    vtkSmartPointer<vtkIdTypeArray>::New();
  ids->SetNumberOfComponents(1);

  //set values
  for(unsigned int i = 10; i < 20; i++)
    {
    ids->InsertNextValue(i);
    }

  vtkSmartPointer<vtkSelectionNode> selectionNode =
      vtkSmartPointer<vtkSelectionNode>::New();
  selectionNode->SetFieldType(vtkSelectionNode::POINT);
  selectionNode->SetContentType(vtkSelectionNode::INDICES);
  selectionNode->SetSelectionList(ids);

  vtkSmartPointer<vtkSelection> selection =
      vtkSmartPointer<vtkSelection>::New();
  selection->AddNode(selectionNode);

  vtkSmartPointer<vtkExtractSelection> extractSelection =
      vtkSmartPointer<vtkExtractSelection>::New();

  extractSelection->SetInput(0, polyDataNormals->GetOutput());
  extractSelection->SetInput(1, selection);
  extractSelection->Update();

  //in selection
  vtkSmartPointer<vtkUnstructuredGrid> selected =
    vtkSmartPointer<vtkUnstructuredGrid>::New();
  selected->ShallowCopy(extractSelection->GetOutput());

  std::cout << "There are " << selected->GetNumberOfPoints()
            << " points in the selection." << std::endl;
  std::cout << "There are " << selected->GetNumberOfCells()
            << " cells in the selection." << std::endl;

  //get points that are NOT in the selection
  selectionNode->GetProperties()->Set(vtkSelectionNode::INVERSE(), 1); //invert the selection
  extractSelection->Update();
  
  vtkDataArray* normalsAfter = vtkDataArray::SafeDownCast(vtkUnstructuredGrid::SafeDownCast(extractSelection->GetOutput())->GetPointData()->GetNormals());
  if(!normalsAfter)
    {
    std::cerr << "no normals after!" << std::endl;
    }
  else
    {
      std::cout << "Has normals after!" << std::endl;
    }
  vtkSmartPointer<vtkUnstructuredGrid> notSelected =
    vtkSmartPointer<vtkUnstructuredGrid>::New();
  notSelected->ShallowCopy(extractSelection->GetOutput());

  std::cout << "There are " << notSelected->GetNumberOfPoints()
            << " points NOT in the selection." << std::endl;
  std::cout << "There are " << notSelected->GetNumberOfCells()
            << " cells NOT in the selection." << std::endl;

  vtkSmartPointer<vtkDataSetMapper> inputMapper =
    vtkSmartPointer<vtkDataSetMapper>::New();
  inputMapper->SetInputConnection(pointSource->GetOutputPort());
  vtkSmartPointer<vtkActor> inputActor =
    vtkSmartPointer<vtkActor>::New();
  inputActor->SetMapper(inputMapper);

  vtkSmartPointer<vtkDataSetMapper> selectedMapper =
    vtkSmartPointer<vtkDataSetMapper>::New();
  selectedMapper->SetInputConnection(selected->GetProducerPort());
  vtkSmartPointer<vtkActor> selectedActor =
    vtkSmartPointer<vtkActor>::New();
  selectedActor->SetMapper(selectedMapper);

  vtkSmartPointer<vtkDataSetMapper> notSelectedMapper =
    vtkSmartPointer<vtkDataSetMapper>::New();
  notSelectedMapper->SetInputConnection(notSelected->GetProducerPort());
  vtkSmartPointer<vtkActor> notSelectedActor =
    vtkSmartPointer<vtkActor>::New();
  notSelectedActor->SetMapper(notSelectedMapper);


  // There will be one render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(900, 300);

  // And one interactor
  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double leftViewport[4] = {0.0, 0.0, 0.5, 1.0};
  double centerViewport[4] = {0.33, 0.0, .66, 1.0};
  double rightViewport[4] = {0.66, 0.0, 1.0, 1.0};

  // Setup the renderers
  vtkSmartPointer<vtkRenderer> leftRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(leftRenderer);
  leftRenderer->SetViewport(leftViewport);
  leftRenderer->SetBackground(.6, .5, .4);

  vtkSmartPointer<vtkRenderer> centerRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(centerRenderer);
  centerRenderer->SetViewport(centerViewport);
  centerRenderer->SetBackground(.3, .1, .4);

  vtkSmartPointer<vtkRenderer> rightRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(rightRenderer);
  rightRenderer->SetViewport(rightViewport);
  rightRenderer->SetBackground(.4, .5, .6);

  leftRenderer->AddActor(inputActor);
  centerRenderer->AddActor(selectedActor);
  rightRenderer->AddActor(notSelectedActor);

  leftRenderer->ResetCamera();
  centerRenderer->ResetCamera();
  rightRenderer->ResetCamera();

  renderWindow->Render();
  interactor->Start();

  return EXIT_SUCCESS;
}
