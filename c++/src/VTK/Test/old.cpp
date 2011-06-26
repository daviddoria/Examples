
#include "vtkConeSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkCamera.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkInteractorStyleTrackballCamera.h"

#include "vtkSmartPointer.h"
#include  "vtkPolyDataNormals.h"
#include  "vtkAlgorithmOutput.h"
#include "vtkPolyDataReader.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkCellData.h"
#include "vtkPointData.h"

int main()
{
	return 0;
}

#if 0
int main()
{
	vtkImageReader *img = vtkImageReader::New();
	img->SetFileName(fn);
	img->Update();
	return 0;
}
#endif

#if 0
int main()
{
	// 1. --------------------------------------------------------
	vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();
	const char* filename="cube.vtk";
					     reader->SetFileName(filename);
					     reader->Update();
					     vtkPolyData* polydata=reader->GetOutput();
	
					     vtkPolyDataMapper *Mapper = vtkPolyDataMapper::New();
					     Mapper->ScalarVisibilityOff();
					     Mapper->SetInput(polydata);

					     vtkPoints* Points=polydata->GetPoints();
					     unsigned int numOfMovedPoints=Points->GetNumberOfPoints();
					     for (unsigned int i=0;i<numOfMovedPoints;i++)
{
					     double point[3];
					     Points->GetPoint(i,point);
					     point[0] = point[0] * 1.6;
					     point[1] = point[1] * 0.8;
					     point[2] = point[2] * 0.91;
					     Points->SetPoint(i,point);
}

//######################################################################
//####Std operations##########

					     vtkActor *Actor = vtkActor::New();
					     Actor->SetMapper(Mapper);

					     vtkRenderer *ren1= vtkRenderer::New();
					     ren1->AddActor( Actor );
					     ren1->SetBackground( .1, 0.2, 0.4 );


					     vtkRenderWindow *renWin = vtkRenderWindow::New();
					     renWin->AddRenderer( ren1 );
					     renWin->SetSize( 300, 300 );


					     vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
					     iren->SetRenderWindow(renWin);

					     vtkInteractorStyleTrackballCamera *style =
					     vtkInteractorStyleTrackballCamera::New();
					     iren->SetInteractorStyle(style);



					     iren->Initialize();
					     iren->Start();


//delete
					     Mapper->Delete();
					     Actor->Delete();
					     ren1->Delete();
					     renWin->Delete();
					     iren->Delete();
					     style->Delete();

					     return 0;
}
#endif