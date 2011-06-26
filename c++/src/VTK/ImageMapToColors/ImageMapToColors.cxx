#include <vtkPNGReader.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkDataSetMapper.h>
#include <vtkDataSet.h>
#include <vtkImageData.h>
#include <vtkImageBlend.h>
#include <vtkLookupTable.h>
#include <vtkImageMapToColors.h>
#include <vtkSmartPointer.h>

int main( int argc, char *[]) 
{
  vtkSmartPointer<vtkLookupTable> imgFirstColorMap = 
      vtkSmartPointer<vtkLookupTable>::New(); // hot color map
  imgFirstColorMap->SetRange( 0.0, 255.0 );
  imgFirstColorMap->SetHueRange( 0.0, 0.1 );
  imgFirstColorMap->SetValueRange( 0.4, 0.8 );
  imgFirstColorMap->Build();

  vtkSmartPointer<vtkLookupTable> imgSecondColorMap = 
      vtkSmartPointer<vtkLookupTable>::New(); // cold color map
  imgFirstColorMap->SetRange( 0.0, 255.0 );
  imgFirstColorMap->SetHueRange( 0.67, 0.68 );
  imgFirstColorMap->SetValueRange( 0.4, 0.8 );
  imgFirstColorMap->Build();

  vtkSmartPointer<vtkPNGReader> imgReader = 
      vtkSmartPointer<vtkPNGReader>::New();
  imgReader->SetFileName( "reference.png" );

  vtkSmartPointer<vtkPNGReader> imgReaderMoving = 
      vtkSmartPointer<vtkPNGReader>::New();
  imgReaderMoving->SetFileName( "moving.png" );

  vtkSmartPointer<vtkImageMapToColors> firstColorMapper = 
      vtkSmartPointer<vtkImageMapToColors>::New();
  firstColorMapper->SetInput( imgReader->GetOutput() );
  firstColorMapper->SetLookupTable( imgFirstColorMap );

  vtkSmartPointer<vtkImageMapToColors> secondColorMapper = 
      vtkSmartPointer<vtkImageMapToColors>::New();
  secondColorMapper->SetInput( imgReaderMoving->GetOutput() );
  secondColorMapper->SetLookupTable( imgSecondColorMap );

  vtkSmartPointer<vtkImageBlend> imgBlender = 
      vtkSmartPointer<vtkImageBlend>::New();
  imgBlender->SetOpacity( 0, 0.5 );
  imgBlender->SetOpacity( 1, 0.5 );
  imgBlender->SetInput( 0, firstColorMapper->GetOutput() );
  imgBlender->SetInput( 1, secondColorMapper->GetOutput() );

  vtkSmartPointer<vtkDataSetMapper> imgDataSetMapper = 
      vtkSmartPointer<vtkDataSetMapper>::New();
  imgDataSetMapper->SetInput( reinterpret_cast< vtkDataSet* >( imgBlender->GetOutput() ) );

  vtkSmartPointer<vtkActor> imgActor = 
      vtkSmartPointer<vtkActor>::New();
  imgActor->SetMapper( imgDataSetMapper );

  vtkSmartPointer<vtkRenderer> imgRenderer = 
      vtkSmartPointer<vtkRenderer>::New();
  imgRenderer->AddActor( imgActor );

  vtkSmartPointer<vtkRenderWindow> imgRenderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  imgRenderWindow->AddRenderer( imgRenderer );

  vtkSmartPointer<vtkRenderWindowInteractor> imgInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  imgInteractor->SetRenderWindow( imgRenderWindow );
  imgInteractor->Initialize();
  imgInteractor->Start();

  return EXIT_SUCCESS;;
}
