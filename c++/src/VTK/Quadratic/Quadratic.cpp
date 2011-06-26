#include <vtkQuadric.h>
#include <vtkSampleFunction.h>
#include <vtkContourFilter.h>
#include <vtkOutlineFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>

void Other();
void Sphere();
void Cone();
void Ellipsoid();
void Cylinder();
void HyperboloidOneSheet();
void HyperboloidTwoSheets();
void HyperbolicParaboloid();
void EllipticParaboloid();
void Fit();

void PlotFunction(vtkQuadric* quadric, double value);


int main ()
{
  //Other();
  //Sphere();
  //Cone();
  //Ellipsoid();
  //Cylinder();
  //HyperboloidOneSheet();
  //HyperboloidTwoSheets();
  //HyperbolicParaboloid();
  //EllipticParaboloid();
  Fit();
  
  return 0;
}


void Sphere()
{
  // create the quadric function definition
  vtkSmartPointer<vtkQuadric> quadric = vtkSmartPointer<vtkQuadric>::New();
  quadric->SetCoefficients(1,1,1,0,0,0,0,0,0,0);
  
  // F(x,y,z) = a0*x^2 + a1*y^2 + a2*z^2 + a3*x*y + a4*y*z + a5*x*z + a6*x + a7*y + a8*z + a9
  // F(x,y,z) = 1*x^2 + 1*y^2 + 1*z^2
 
  PlotFunction(quadric, 1.0);
}

void EllipticParaboloid()
{
  // create the quadric function definition
  vtkSmartPointer<vtkQuadric> quadric = vtkSmartPointer<vtkQuadric>::New();
  quadric->SetCoefficients(1,1,0,0,0,0,0,0,-1,0);
  
  // F(x,y,z) = a0*x^2 + a1*y^2 + a2*z^2 + a3*x*y + a4*y*z + a5*x*z + a6*x + a7*y + a8*z + a9
  // F(x,y,z) = 1*x^2 + 1*y^2
 
  PlotFunction(quadric, 0.0);
}

void HyperbolicParaboloid()
{
  // create the quadric function definition
  vtkSmartPointer<vtkQuadric> quadric = vtkSmartPointer<vtkQuadric>::New();
  quadric->SetCoefficients(1,-1,0,0,0,0,0,0,0,0);
  
  // F(x,y,z) = a0*x^2 + a1*y^2 + a2*z^2 + a3*x*y + a4*y*z + a5*x*z + a6*x + a7*y + a8*z + a9
  // F(x,y,z) = 1*x^2 - 1*y^2
 
  PlotFunction(quadric, 1.0);
}

void Cylinder()
{
  // create the quadric function definition
  vtkSmartPointer<vtkQuadric> quadric = vtkSmartPointer<vtkQuadric>::New();
  quadric->SetCoefficients(1,1,0,0,0,0,0,0,0,0);
  
  // F(x,y,z) = a0*x^2 + a1*y^2 + a2*z^2 + a3*x*y + a4*y*z + a5*x*z + a6*x + a7*y + a8*z + a9
  // F(x,y,z) = 1*x^2 + 1*y^2
  
  PlotFunction(quadric, 1.0);
}

void HyperboloidOneSheet()
{
  // create the quadric function definition
  vtkSmartPointer<vtkQuadric> quadric = vtkSmartPointer<vtkQuadric>::New();
  quadric->SetCoefficients(1,1,-1,0,0,0,0,0,0,0);
  
  // F(x,y,z) = a0*x^2 + a1*y^2 + a2*z^2 + a3*x*y + a4*y*z + a5*x*z + a6*x + a7*y + a8*z + a9
  // F(x,y,z) = 1*x^2 + 1*y^2
  
  PlotFunction(quadric, 1.0);
}

void HyperboloidTwoSheets()
{
    // create the quadric function definition
  vtkSmartPointer<vtkQuadric> quadric = vtkSmartPointer<vtkQuadric>::New();
  quadric->SetCoefficients(1,1,-1,0,0,0,0,0,0,0);
  
  // F(x,y,z) = a0*x^2 + a1*y^2 + a2*z^2 + a3*x*y + a4*y*z + a5*x*z + a6*x + a7*y + a8*z + a9
  // F(x,y,z) = 1*x^2 + 1*y^2
  
  PlotFunction(quadric, -1.0);
}

void Ellipsoid()
{
  // create the quadric function definition
  vtkSmartPointer<vtkQuadric> quadric = vtkSmartPointer<vtkQuadric>::New();
  quadric->SetCoefficients(1,1,2,0,0,0,0,0,0,0);
  
  // F(x,y,z) = a0*x^2 + a1*y^2 + a2*z^2 + a3*x*y + a4*y*z + a5*x*z + a6*x + a7*y + a8*z + a9
  // F(x,y,z) = 1*x^2 + 1*y^2 + 1*z^2
 
  PlotFunction(quadric, 1.0);
}

void Cone()
{
  // create the quadric function definition
  vtkSmartPointer<vtkQuadric> quadric = vtkSmartPointer<vtkQuadric>::New();
  quadric->SetCoefficients(1,1,-1,0,0,0,0,0,0,0);
  
  // F(x,y,z) = a0*x^2 + a1*y^2 + a2*z^2 + a3*x*y + a4*y*z + a5*x*z + a6*x + a7*y + a8*z + a9
  // F(x,y,z) = 1*x^2 + 1*y^2 - 1*z^2
  PlotFunction(quadric, 0.0);
 
}

void Other()
{
  // create the quadric function definition
  vtkSmartPointer<vtkQuadric> quadric = vtkSmartPointer<vtkQuadric>::New();
  quadric->SetCoefficients(.5,1,.2,0,.1,0,0,.2,0,0);
  
  // F(x,y,z) = a0*x^2 + a1*y^2 + a2*z^2 + a3*x*y + a4*y*z + a5*x*z + a6*x + a7*y + a8*z + a9
  // F(x,y,z) = 0.5*x^2 + 1*y^2 + 0.2*z^2 + 0*x*y + 0.1*y*z + 0*x*z + 0*x + 0.2*y + 0*z + 0
  PlotFunction(quadric, 1.0);
  

}

void Fit()
{
 // create the quadric function definition
  vtkSmartPointer<vtkQuadric> quadric = vtkSmartPointer<vtkQuadric>::New();
  quadric->SetCoefficients(2.32007,2.17675,0,1.48577,0,0,0.347806,0.284521,-1,-0.511625);
  vtkstd::cout << "quadric: " << *quadric << vtkstd::endl;
  
  // F(x,y,z) = a0*x^2 + a1*y^2 + a2*z^2 + a3*x*y + a4*y*z + a5*x*z + a6*x + a7*y + a8*z + a9
  //F(x,y,z) = 2.32007*x^2 + 2.17675*y^2 + 0 + 1.48577*x*y + 0 + 0 + 0.347806*x + 0.284521*y -1*z + -0.511625

  PlotFunction(quadric, 0.0);
  

}


void PlotFunction(vtkQuadric* quadric, double value)
{
 
  // sample the quadric function
  vtkSmartPointer<vtkSampleFunction> sample = vtkSmartPointer<vtkSampleFunction>::New();
  sample->SetSampleDimensions(50,50,50);
  sample->SetImplicitFunction(quadric);
  //double xmin = 0, xmax=1, ymin=0, ymax=1, zmin=0, zmax=1;
  double xmin = -10, xmax=11, ymin=-10, ymax=10, zmin=-10, zmax=10;
  sample->SetModelBounds(xmin, xmax, ymin, ymax, zmin, zmax);
  
  // Create five surfaces F(x,y,z) = constant between range specified
  /*
  vtkContourFilter *contours = vtkContourFilter::New();
  contours->SetInput(sample->GetOutput());
  contours->GenerateValues(5, 0.0, 1.2);
  */
  
  //create the 0 isosurface
  vtkSmartPointer<vtkContourFilter> contours = vtkSmartPointer<vtkContourFilter>::New();
  contours->SetInput(sample->GetOutput());
  contours->GenerateValues(1, value, value);
  
  // map the contours to graphical primitives
  vtkSmartPointer<vtkPolyDataMapper> contourMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  contourMapper->SetInput(contours->GetOutput());
  contourMapper->SetScalarRange(0.0, 1.2);
  
  // create an actor for the contours
  vtkSmartPointer<vtkActor> contourActor = vtkSmartPointer<vtkActor>::New();
  contourActor->SetMapper(contourMapper);
  
  // -- create a box around the function to indicate the sampling volume --
  
  // create outline
  vtkSmartPointer<vtkOutlineFilter> outline = vtkSmartPointer<vtkOutlineFilter>::New();
  outline->SetInput(sample->GetOutput());
  
  // map it to graphics primitives
  vtkSmartPointer<vtkPolyDataMapper> outlineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  outlineMapper->SetInput(outline->GetOutput());
  
  // create an actor for it
  vtkSmartPointer<vtkActor> outlineActor = vtkSmartPointer<vtkActor>::New();
  outlineActor->SetMapper(outlineMapper);
  outlineActor->GetProperty()->SetColor(0,0,0);
  
  // setup the window
  vtkSmartPointer<vtkRenderer> ren1 = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
  renWin->AddRenderer(ren1);
  vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  iren->SetRenderWindow(renWin);
  
  // add the actors to the scene
  ren1->AddActor(contourActor);
  ren1->AddActor(outlineActor);
  ren1->SetBackground(1,1,1); // Background color white
  
  // render and interact
  renWin->Render();
  iren->Start();
}