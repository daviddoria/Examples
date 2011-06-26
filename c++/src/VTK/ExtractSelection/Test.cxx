#include <vtkSmartPointer.h>
#include <vtkInformation.h>
#include <vtkPointSource.h>
#include <vtkExtractSelection.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkIdTypeArray.h>

void RemoveInliers(vtkSmartPointer<vtkPoints> points, vtkstd::vector<unsigned int> inlierIndices);

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(50);
  pointSource->Update();
  
  cout << "There are " << pointSource->GetOutput()->GetNumberOfPoints() << " input points." << endl;
  
  vtkstd::vector<unsigned int> ids;
  
  //set values
  for(unsigned int i = 10; i < 20; i++)
    { 
    ids.push_back(i);
    }
     
  vtkPoints* points = pointSource->GetOutput()->GetPoints();
  RemoveInliers(points, ids);
    
  cout << "There are " << points->GetNumberOfPoints() << " points not in the selection." << endl;
  
  return EXIT_SUCCESS;
}


void RemoveInliers(vtkSmartPointer<vtkPoints> points, vtkstd::vector<unsigned int> inlierIndices)
{
  
  vtkSmartPointer<vtkPolyData> pointsPolyData = 
      vtkSmartPointer<vtkPolyData>::New();
  pointsPolyData->SetPoints(points);
  
  vtkSmartPointer<vtkIdTypeArray> ids = 
      vtkSmartPointer<vtkIdTypeArray>::New();
  ids->SetNumberOfComponents(1);
  
  //set values
  for(unsigned int i = 0; i < inlierIndices.size(); i++)
    { 
    ids->InsertNextValue(inlierIndices[i]);
    }
  
  vtkSmartPointer<vtkSelectionNode> selectionNode = 
      vtkSmartPointer<vtkSelectionNode>::New();
  selectionNode->SetFieldType(vtkSelectionNode::POINT);
  selectionNode->SetContentType(vtkSelectionNode::INDICES);
  selectionNode->SetSelectionList(ids);
  selectionNode->GetProperties()->Set(vtkSelectionNode::INVERSE(), 1); //invert the selection
  
  vtkSmartPointer<vtkSelection> selection = 
      vtkSmartPointer<vtkSelection>::New();
  selection->AddNode(selectionNode);
  
  vtkSmartPointer<vtkExtractSelection> extractSelection = 
      vtkSmartPointer<vtkExtractSelection>::New();
  
  extractSelection->SetInput(0, pointsPolyData);
  extractSelection->SetInput(1, selection);
  extractSelection->Update();
  
  //points->ShallowCopy(vtkDataSet::SafeDownCast (extractSelection->GetOutput())->GetPoints());
  points->ShallowCopy(vtkPointSet::SafeDownCast (extractSelection->GetOutput())->GetPoints());
  
  
  /*
  vtkDataSet* ds = vtkDataSet::SafeDownCast (extractSelection->GetOutput());
  vtkPolyData* pd = vtkPolyData::SafeDownCast (extractSelection->GetOutput());
  
  cout << "stop";
  */
}
