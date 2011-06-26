#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyDataMapper.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>

#include <vtkKdTreePointLocator.h>
#include <vtkOctreePointLocator.h>
#include <vtkOBBTree.h>
#include <vtkModifiedBSPTree.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPolyData> originalMesh =
    vtkSmartPointer<vtkPolyData>::New();

  if(argc > 1) //If a file name is specified, open and use the file.
    {
    vtkSmartPointer<vtkXMLPolyDataReader> reader =
      vtkSmartPointer<vtkXMLPolyDataReader>::New();
    reader->SetFileName(argv[1]);
    reader->Update();
    originalMesh->ShallowCopy(reader->GetOutput());
    }
  else //If a file name is not specified, create a sphere
    {
    vtkSmartPointer<vtkSphereSource> sphereSource =
      vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->Update();
    originalMesh->ShallowCopy(sphereSource->GetOutput());
    }

  double numberOfViewports = 4.;

  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(200* numberOfViewports,200); //(width, height)

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();

  renderWindowInteractor->SetRenderWindow(renderWindow);

  int level = 1;
  
  for(unsigned i = 0; i < numberOfViewports; i++)
    {
    
    vtkSmartPointer<vtkLocator> tree;
  
    switch(i)
      {
      case 0: //KDTree
        tree = vtkSmartPointer<vtkKdTreePointLocator>::New();
        break;
      case 1: //OBBTree
        tree = vtkSmartPointer<vtkOBBTree>::New();
        break;
      case 2: //Octree
        tree = vtkSmartPointer<vtkOctreePointLocator>::New();
        break;
      case 3: //Modified BSP Tree
        tree = vtkSmartPointer<vtkModifiedBSPTree>::New();
        break;
      default:
        break;
      }

    tree->SetDataSet(originalMesh);
    tree->BuildLocator();

    vtkSmartPointer<vtkPolyData> polydata =
      vtkSmartPointer<vtkPolyData>::New();
    tree->GenerateRepresentation(level, polydata);

    vtkSmartPointer<vtkRenderer> renderer =
      vtkSmartPointer<vtkRenderer>::New();

    renderWindow->AddRenderer(renderer);
    renderer->SetViewport(static_cast<double>(i)/numberOfViewports,0,static_cast<double>(i+1)/numberOfViewports,1);

    //Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(polydata->GetProducerPort());
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetRepresentationToWireframe();
    renderer->AddActor(actor);
    renderer->ResetCamera();

    renderWindow->Render();
    renderWindow->SetWindowName("Multiple ViewPorts");

    }

  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
