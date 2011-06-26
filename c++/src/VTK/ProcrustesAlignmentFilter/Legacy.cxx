#include <vtkSmartPointer.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkProcrustesAlignmentFilter.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkLandmarkTransform.h>
#include <vtkXMLPolyDataWriter.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPlaneSource> planeSource1 = 
      vtkSmartPointer<vtkPlaneSource>::New();
  
  vtkSmartPointer<vtkPlaneSource> planeSource2 = 
      vtkSmartPointer<vtkPlaneSource>::New();
  planeSource2->SetCenter(10,0,0);
  planeSource2->Update();
      
  vtkSmartPointer<vtkProcrustesAlignmentFilter> procrustesAlignmentFilter = 
      vtkSmartPointer<vtkProcrustesAlignmentFilter>::New();
  procrustesAlignmentFilter->SetNumberOfInputs(2);
  procrustesAlignmentFilter->SetInput(0, planeSource1->GetOutput());
  procrustesAlignmentFilter->SetInput(1, planeSource2->GetOutput());
  procrustesAlignmentFilter->Update();
  
  
  return EXIT_SUCCESS;
}