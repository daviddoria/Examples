#include <vtkImageData.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkJPEGReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTexture.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkFloatArray.h>
#include <vtkPolygon.h>

int main ( int argc, char *argv[] )
{
  //parse command line arguments
  if ( argc != 2 )
  {
    vtkstd::cout << "Required parameters: Filename" << vtkstd::endl;
    exit ( -1 );
  }

  vtkstd::string InputFilename = argv[1];

  //read the image which will be the texture
  vtkSmartPointer<vtkJPEGReader> JPEGReader = vtkSmartPointer<vtkJPEGReader>::New();
  JPEGReader->SetFileName ( InputFilename.c_str() );
  
  //create a plane
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  Points->InsertNextPoint(0.0, 0.0, 0.0);
  Points->InsertNextPoint(1.0, 0.0, 0.0);
  Points->InsertNextPoint(1.0, 1.0, 0.0);
  Points->InsertNextPoint(0.0, 2.0, 0.0);
  
  vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
  polygon->GetPointIds()->SetNumberOfIds(4); //make a quad
  polygon->GetPointIds()->SetId(0, 0);
  polygon->GetPointIds()->SetId(1, 1);
  polygon->GetPointIds()->SetId(2, 2);
  polygon->GetPointIds()->SetId(3, 3);
  
  polygons->InsertNextCell(polygon);

  vtkSmartPointer<vtkPolyData> Quad = vtkSmartPointer<vtkPolyData>::New();
  Quad->SetPoints(Points);
  Quad->SetPolys(polygons);
  
  vtkSmartPointer<vtkFloatArray> TextureCoordinates = vtkSmartPointer<vtkFloatArray>::New();
  TextureCoordinates->SetNumberOfComponents(3);
  TextureCoordinates->SetName("TextureCoordinates");
  
  float tuple[3];
  tuple[0] = 0.0; tuple[1] = 0.0; tuple[2] = 0.0;
  TextureCoordinates->InsertNextTuple(tuple);
  tuple[0] = 1.0; tuple[1] = 0.0; tuple[2] = 0.0;
  TextureCoordinates->InsertNextTuple(tuple);
  tuple[0] = 1.0; tuple[1] = 1.0; tuple[2] = 0.0;
  TextureCoordinates->InsertNextTuple(tuple);
  tuple[0] = 0.0; tuple[1] = 1.0; tuple[2] = 0.0;
  TextureCoordinates->InsertNextTuple(tuple);
  
  Quad->GetPointData()->SetTCoords(TextureCoordinates);
  
  //apply the texture
  vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
  texture->SetInput(JPEGReader->GetOutput());

  vtkSmartPointer<vtkPolyDataMapper> Mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  Mapper->SetInput(Quad);

  vtkSmartPointer<vtkActor> TexturedQuad = vtkSmartPointer<vtkActor>::New();
  TexturedQuad->SetMapper(Mapper);
  TexturedQuad->SetTexture(texture);

  // visualize the textured plane
  vtkSmartPointer<vtkRenderer> Renderer = vtkSmartPointer<vtkRenderer>::New();
  Renderer->AddActor(TexturedQuad);
  Renderer->SetBackground(1,1,1); // Background color white
  Renderer->ResetCamera();
  
  vtkSmartPointer<vtkRenderWindow> RenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  RenderWindow->AddRenderer(Renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> RenderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  RenderWindowInteractor->SetRenderWindow(RenderWindow);
  
  RenderWindow->Render();
  
  RenderWindowInteractor->Start();

  return 0;
}
