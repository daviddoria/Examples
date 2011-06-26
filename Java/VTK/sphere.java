//package HelloVtkMain;
import vtk.*;
public class sphere
{
  // in the static contructor we load in the native code
  // The libraries must be in your path to work
  static 
  {
    System.loadLibrary("vtkCommonJava");
    System.loadLibrary("vtkFilteringJava");
    System.loadLibrary("vtkIOJava");
    System.loadLibrary("vtkImagingJava");
    System.loadLibrary("vtkGraphicsJava");
    System.loadLibrary("vtkRenderingJava");
  }
  // the main function
  public static void main (String[] args)
  {
    // create sphere geometry
    vtkSphereSource sphere = new vtkSphereSource();
    sphere.SetRadius(1.0);
    sphere.SetThetaResolution(18);
    sphere.SetPhiResolution(18);
    // map to graphics objects
    vtkPolyDataMapper map = new vtkPolyDataMapper();
    map.SetInput(sphere.GetOutput());
    // actor coordinates geometry, properties, transformation
    vtkActor aSphere = new vtkActor();
    aSphere.SetMapper(map);
    aSphere.GetProperty().SetColor(0,0,1); // color blue
    // a renderer for the data
    vtkRenderer ren1 = new vtkRenderer();
    ren1.AddActor(aSphere);
    ren1.SetBackground(1,1,1); // background color white
    // a render window to display the contents
    vtkRenderWindow renWin = new vtkRenderWindow();
    renWin.AddRenderer(ren1);
    renWin.SetSize(300,300);
    // an interactor to allow control of the objects
    vtkRenderWindowInteractor iren = new vtkRenderWindowInteractor();
    iren.SetRenderWindow(renWin);
    // trigger the rendering and start the interaction
    renWin.Render();
    iren.Start();
  }
}

