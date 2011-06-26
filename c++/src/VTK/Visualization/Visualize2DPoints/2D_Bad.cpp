#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper2D.h"
#include "vtkActor2D.h"
#include "vtkMath.h"
#include "vtkProperty2D.h"
#include "vtkCellArray.h"
#include "vtkPolyData.h"
#include "vtkXMLPolyDataReader.h"

int main(int argc, char* argv[])
{
	if(argc != 2)
	{
		std::cout << "Required parameters: Filename" << std::endl;
		exit(-1);
	}
  
	std::string InputFilename = argv[1];
  
	vtkXMLPolyDataReader* Reader = vtkXMLPolyDataReader::New();
	Reader->SetFileName(InputFilename.c_str());
	Reader->Update();
	vtkPolyData* Polydata = Reader->GetOutput();
  //vtkPoints* Points = Polydata->GetPoints();
  
	vtkActor2D* Actor = vtkActor2D::New();
	double color[3] = {1,1,1};
  
	vtkPolyDataMapper2D* Mapper = vtkPolyDataMapper2D::New();
	Mapper->SetInput( Polydata );
	Mapper->ScalarVisibilityOff();
	Actor->SetMapper( Mapper );
	Actor->GetProperty()->SetColor( color );

	vtkRenderer *Renderer = vtkRenderer::New();
	Renderer->AddActor(Actor);
  
  // Renderer and RenderWindow
	Renderer->SetBackground( 0, 0, 0);
	Renderer->ResetCamera();
	vtkRenderWindow *RenderWindow = vtkRenderWindow::New();
  
	RenderWindow->SetSize( 200,200 );
	RenderWindow->AddRenderer( Renderer );
	vtkRenderWindowInteractor* RenderWindowInteractor = vtkRenderWindowInteractor::New();
	RenderWindowInteractor->SetRenderWindow( RenderWindow );

	RenderWindow->Render();
	RenderWindowInteractor->Initialize();
	RenderWindowInteractor->Start();

  //cleanup
	RenderWindowInteractor->Delete();
	RenderWindow->Delete();
	Mapper->Delete();
	Renderer->Delete();
	Actor->Delete();
	Reader->Delete();

	return 0 ;
}
