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
 
int main (int argc, char **argv)
{
  if(argc != 2)
  {
    vtkstd::cout << "Required arguments: Filename" << vtkstd::endl;
    exit(-1);
  }
 
  vtkstd::string Filename = argv[1];
 
  vtkstd::cout << "Reading " << Filename << vtkstd::endl;
 
 // VRML Import
  vtkSmartPointer<vtkVRMLImporter> importer = vtkSmartPointer<vtkVRMLImporter>::New();
//importer->SetFileName(argv[1]);
  importer->SetFileName(Filename.c_str());
  importer->Read();
  importer->Update();
 
//Convert to vtkDataSet
  vtkDataSet *pDataset;
  vtkActorCollection *actors = importer->GetRenderer()->GetActors();
  actors->InitTraversal();
  pDataset = actors->GetNextActor()->GetMapper()->GetInput(); //Problem happened here
 
//Convert to vtkPolyData
  vtkPolyData *polyData = vtkPolyData::SafeDownCast(pDataset);
  polyData->Update();
 
/*
// Create Normal Vectors to enhance smoothness & illumination
  vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();
  normals->SetInput(polyData);
  normals->SetFeatureAngle(60.0);
 */
  
// Mapper
  vtkSmartPointer<vtkPolyDataMapper> SolidMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  //SolidMapper->SetInput(normals->GetOutput());
  SolidMapper->SetInput(polydata);
  SolidMapper->ScalarVisibilityOff();
 
// Actor
  vtkSmartPointer<vtkActor> SolidActor = vtkSmartPointer<vtkActor>::New();
  SolidActor->SetMapper(SolidMapper);
//SolidActor->GetProperty()->SetOpacity(1);
 
// Render
  vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New();
  ren->AddActor(SolidActor);
  ren->SetBackground(1,1,1);
 
// RenderWindow
  vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
  renWin->AddRenderer(ren);
 
// RenderWindowInteractor
  vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  iren->SetRenderWindow(renWin);
 
  renWin->Start();
  renWin->SetSize(800-5,600-5); // Must be Called after Start() and before Render()
// interact with data
  //iren->Initialize();
  iren->Start();

 
  return 0;
}