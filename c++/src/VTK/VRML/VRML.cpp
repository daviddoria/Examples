#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"

#include "vtkVRMLImporter.h"
#include "vtkDataSet.h"
#include "vtkActorCollection.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyDataNormals.h"
#include "vtkActor.h"
#include "vtkSmartPointer.h"

#include <iostream>

int main ( int argc, char **argv )
{
  if ( argc != 2 )
  {
    vtkstd::cout << "Required arguments: Filename" << vtkstd::endl;
    exit ( -1 );
  }

  vtkstd::string Filename = argv[1];
  vtkstd::cout << "Reading " << Filename << vtkstd::endl;

  // VRML Import
  vtkSmartPointer<vtkVRMLImporter> importer = vtkSmartPointer<vtkVRMLImporter>::New();
  importer->SetFileName ( Filename.c_str() );
  importer->Read();
  importer->Update();

  //Convert to vtkDataSet
  vtkDataSet *pDataset;
  vtkActorCollection *actors = importer->GetRenderer()->GetActors();
  actors->InitTraversal();
  pDataset = actors->GetNextActor()->GetMapper()->GetInput(); //Problem happened here

  //Convert to vtkPolyData
  vtkPolyData *polyData = vtkPolyData::SafeDownCast ( pDataset );
  polyData->Update();

  // Mapper
  vtkSmartPointer<vtkPolyDataMapper> SolidMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  SolidMapper->SetInput ( polyData );
  SolidMapper->ScalarVisibilityOff();

  // Actor
  vtkSmartPointer<vtkActor> SolidActor = vtkSmartPointer<vtkActor>::New();
  SolidActor->SetMapper ( SolidMapper );

  // a renderer and render window
  vtkSmartPointer<vtkRenderer> Renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> RenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  RenderWindow->AddRenderer ( Renderer );

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  RenderWindowInteractor->SetRenderWindow ( RenderWindow );

  // add the actors to the scene
  Renderer->AddActor ( SolidActor );

  // render an image (lights and cameras are created automatically)
  RenderWindow->Render();

  // begin mouse interaction
  RenderWindowInteractor->Start();

  return 0;
}
