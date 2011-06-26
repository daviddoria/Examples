#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkParallelCoordinatesView.h>
#include <vtkParallelCoordinatesRepresentation.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

int main(int, char*[])
{

  vtkSmartPointer<vtkFloatArray> array1 =
    vtkSmartPointer<vtkFloatArray>::New();
  array1->SetName("Array1");
  array1->SetNumberOfComponents(1);
  array1->InsertNextValue(1);
  array1->InsertNextValue(2);

  vtkSmartPointer<vtkFloatArray> array2 =
    vtkSmartPointer<vtkFloatArray>::New();
  array2->SetName("Array2");
  array2->SetNumberOfComponents(1);
  array2->InsertNextValue(1);
  array2->InsertNextValue(2);

  vtkSmartPointer<vtkPolyData> polydata =
    vtkSmartPointer<vtkPolyData>::New();
  polydata->GetPointData()->AddArray(array1);
  polydata->GetPointData()->AddArray(array2);

  // Set up the parallel coordinates Representation to be used in the View
  vtkSmartPointer<vtkParallelCoordinatesRepresentation> rep =
    vtkSmartPointer<vtkParallelCoordinatesRepresentation>::New();

  rep->SetInputConnection(polydata->GetProducerPort());

  // List all of the attribute arrays you want plotted in parallel coordinates
  rep->SetInputArrayToProcess(0,0,0,0,"Array1");
  rep->SetInputArrayToProcess(1,0,0,0,"Array2");

  rep->SetUseCurves(0);//     # set to 1 to use smooth curves
  rep->SetLineOpacity(0.5);

  // Set up the Parallel Coordinates View and hook in the Representation
  vtkSmartPointer<vtkParallelCoordinatesView> view =
    vtkSmartPointer<vtkParallelCoordinatesView>::New();
  view->SetRepresentation(rep);
  view->SetInspectMode(1);
  // Brush Mode determines the type of interaction you perform to select data
  view->SetBrushModeToLasso();
  view->SetBrushOperatorToReplace();

  // Set up render window
  view->GetRenderWindow()->SetSize(600,300);
  view->ResetCamera();
  view->Render();

  // Start interaction event loop
  view->GetInteractor()->Start();

  return EXIT_SUCCESS;
}