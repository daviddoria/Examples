from vtk import*

sphereSource =vtkSphereSource();
sphereSource.SetCenter(0.0, 0.0, 0.0);
sphereSource.SetRadius(5.0);

#Create a mapper and actor
mapper = vtkPolyDataMapper();
mapper.SetInputConnection(sphereSource.GetOutputPort());

actor = vtkActor();
actor.SetMapper(mapper);

#Create a renderer, render window, and interactor
renderer =vtkRenderer();
renderWindow =vtkRenderWindow();
renderWindow.AddRenderer(renderer);
renderWindowInteractor =vtkRenderWindowInteractor();
renderWindowInteractor.SetRenderWindow(renderWindow);

#Add the actor to the scene
renderer.AddActor(actor);
renderer.SetBackground(0,0,0); # Background color white

cursor =vtkCursor2D();
cursor.SetModelBounds(-10,10,-10,10,0,0)
cursor.AllOn()
cursor.OutlineOff()
cursor.Update();
cursormapper=vtkPolyDataMapper()
cursormapper.SetInputConnection(cursor.GetOutputPort())
cursoractor=vtkActor()
cursoractor.GetProperty().SetColor(1,1,0)
cursoractor.SetMapper(cursormapper)
renderer.AddActor(cursoractor);

handleRep =vtkPointHandleRepresentation2D();
handleRep.SetDisplayPosition(cursoractor.GetPosition());
handleRep.ActiveRepresentationOn();
handleRep.SetCursorShape(cursor.GetOutput());

handleWidget =vtkHandleWidget();
handleWidget.SetInteractor(renderWindowInteractor);
handleWidget.SetRepresentation(handleRep);


handleWidget.On();

#Render and interact
renderWindow.Render();
renderWindowInteractor.Start();
