#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkMultiBlockMergeFilter.h>
#include <vtkMultiBlockDataSet.h>

int main(int argc, char **argv)
{

  vtkSmartPointer<vtkSphereSource> sphereSource1 =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource1->Update();
  
  vtkSmartPointer<vtkSphereSource> sphereSource2 =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource2->SetCenter(10,10,10);
  sphereSource2->Update();

  vtkSmartPointer<vtkMultiBlockDataSet> multiBlockDataSet1 =
    vtkSmartPointer<vtkMultiBlockDataSet>::New();
  multiBlockDataSet1->SetNumberOfBlocks(1);
  multiBlockDataSet1->SetBlock(0, sphereSource1->GetOutput());
  multiBlockDataSet1->Update();

  vtkSmartPointer<vtkMultiBlockDataSet> multiBlockDataSet2 =
    vtkSmartPointer<vtkMultiBlockDataSet>::New();
  multiBlockDataSet2->SetNumberOfBlocks(1);
  multiBlockDataSet2->SetBlock(0, sphereSource2->GetOutput());
  multiBlockDataSet2->Update();
  
  vtkSmartPointer<vtkMultiBlockMergeFilter> multiBlockMergeFilter =
      vtkSmartPointer<vtkMultiBlockMergeFilter>::New();
  multiBlockMergeFilter->AddInput(multiBlockDataSet1);
  multiBlockMergeFilter->AddInput(multiBlockDataSet2);
  multiBlockMergeFilter->Update();
  
  return EXIT_SUCCESS;
}
