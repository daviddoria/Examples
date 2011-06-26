#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkProperty.h>
#include <vtkCursor2D.h>

int main(int argc, char* argv[])
{
    // create sphere geometry
    vtkSphereSource *sphere = vtkSphereSource::New();
    //sphere->SetCenter(0.0, 0.0, 0.0);
    //sphere->SetRadius(1.0);
    //sphere->SetThetaResolution(18);
    //sphere->SetPhiResolution(18);
    sphere->Update();
    
    // map to graphics library
    vtkPolyDataMapper *map = vtkPolyDataMapper::New();
    //map->SetInput(sphere->GetOutput());
    map->SetInputConnection(sphere->GetOutputPort());

    // actor coordinates geometry, properties, transformation
    vtkActor *actor = vtkActor::New();
    actor->SetMapper(map);
    //actor->GetProperty()->SetColor(0,0,1); // sphere color blue

    // renderers and render window
    vtkRenderer *renA = vtkRenderer::New();
//    renA->SetViewport(0.0, 0.0, 1.0, 1.0);
    vtkRenderWindow* win = vtkRenderWindow::New();
    //win->SetSize(300,300);
    win->AddRenderer(renA);

    // an interactor
    vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
    iren->SetRenderWindow(win);

    renA->SetBackground(1,1,1); // Background color white

    renA->AddActor(actor);

    // add a 2D cursor to each Renderer
    vtkCursor2D* cursor = vtkCursor2D::New();
    //cursor->SetTranslationMode(1);
    //cursor->SetWrap(1);
    //cursor->SetModelBounds(-0.5, 0.5, -0.5, 0.5, 0.0, 0.0);
    //cursor->SetModelBounds(-0.5, 0.5, -0.5, 0.5, -0.5, 0.5);
    //cursor->SetModelBounds(-10, 10, -10, 10, 0, 0);
    cursor->SetModelBounds(-1, 1, -1, 1, 0, 0);
    //cursor->SetFocalPoint(0.0, 0.0, 0.0);
    //cursor->AllOff();
    //cursor->AxesOn();
    cursor->Update();

    vtkPolyDataMapper* mapper = vtkPolyDataMapper::New();
    mapper->SetInputConnection(cursor->GetOutputPort());
    actor = vtkActor::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.0, 0.0);

    renA->AddActor(actor);

    // render an image (lights and cameras are created automatically)
    win->Render();

    iren->Start();

    return 0;
}