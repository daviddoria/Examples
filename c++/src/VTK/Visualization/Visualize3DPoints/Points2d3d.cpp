#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkActor2D.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkMath.h"
#include "vtkProperty.h"
#include "vtkCellArray.h"
#include "vtkPolyData.h"
#include "vtkXMLPolyDataReader.h"

void Render(vtkRenderer* ren);
void PointsActor(vtkPoints* points, vtkActor* pointsActor, double color[3]);
void PointsVisualize(vtkPoints *points1);
		
int main(int argc, char* argv[])
{
  if(argc != 2)
  {
    std::cout << "Required parameters: Filename" << std::endl;
    exit(-1);
  }
	
  std::string InputFilename = argv[1];
	
  vtkXMLPolyDataReader* reader = vtkXMLPolyDataReader::New();
  reader->SetFileName(InputFilename.c_str());
  reader->Update();
  vtkPolyData* polydata = reader->GetOutput();
  vtkPoints* points = polydata->GetPoints();
	
  PointsVisualize(points);
    
  reader->Delete();
    

  return 0 ;
}

////////////////////////////////////////////////

void PointsActor(vtkPoints* points, vtkActor* pointsActor, double color[3])
{
	// Create vtkCellArray
  vtkCellArray *verts = vtkCellArray::New();

  for( int i=0; i<points->GetNumberOfPoints(); i++)
  {
    verts->InsertNextCell(1, &i );
  }

	// Create a PolyData dummy variable
  vtkPolyData *dummyPolyData = vtkPolyData::New(); 
  dummyPolyData->SetPoints( points );
  dummyPolyData->SetVerts( verts );

  vtkPolyDataMapper *pointsMapper = vtkPolyDataMapper::New();
  pointsMapper->SetInput( dummyPolyData );
  pointsMapper->ScalarVisibilityOff();
  pointsActor->SetMapper( pointsMapper );
  pointsActor->GetProperty()->SetColor( color );

    //cleanup
  pointsMapper->Delete();
  dummyPolyData->Delete();
  verts->Delete();

}



void Render(vtkRenderer* ren)
{
	// Renderer and RenderWindow
  ren->SetBackground( 0, 0, 0);
  ren->ResetCamera();
  vtkRenderWindow *renWin = vtkRenderWindow::New();
  renWin->SetSize( 512,512 );
  renWin->AddRenderer( ren );
  vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
  iren->SetRenderWindow( renWin );

  renWin->Render();

  iren->Initialize();

  iren->Start();

    //cleanup
  iren->Delete();
  renWin->Delete();
}



void PointsVisualize(vtkPoints* points1)
{
  vtkActor* points1Actor = vtkActor::New();
  double color1[3] = {1,1,1};
  PointsActor( points1, points1Actor, color1);
  vtkRenderer *ren = vtkRenderer::New();
  ren->AddActor(points1Actor);
  Render( ren );

    //cleanup
  ren->Delete();
  points1Actor->Delete();
}

