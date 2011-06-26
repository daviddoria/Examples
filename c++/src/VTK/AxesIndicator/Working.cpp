#include "vtkActor.h"
#include "vtkAnnotatedCubeActor.h"
#include "vtkAppendPolyData.h"
#include "vtkAxesActor.h"
#include "vtkCamera.h"
#include "vtkCaptionActor2D.h"
#include "vtkCellArray.h"
#include "vtkInteractorEventRecorder.h"
#include "vtkMath.h"
#include "vtkOrientationMarkerWidget.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkPropAssembly.h"
#include "vtkPropCollection.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkTextProperty.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkTubeFilter.h"

#include "vtkRegressionTestImage.h"
#include "vtkDebugLeaks.h"


int main( int argc, char *argv[] )
{


  // the final actor the widget will follow
	//
	vtkAxesActor* axes = vtkAxesActor::New();

	axes->SetTotalLength( 1.2, 1.2 , 1.2 );
	axes->SetNormalizedShaftLength( 0.85, 0.85, 0.85 );
	axes->SetNormalizedTipLength( 0.15, 0.15, 0.15 );
	axes->AxisLabelsOff();

	vtkProperty* property = axes->GetXAxisTipProperty();
	property->SetRepresentationToWireframe();
	property->SetDiffuse(0);
	property->SetAmbient(1);
	property->SetColor( 1, 0, 1 );

	property = axes->GetYAxisTipProperty();
	property->SetRepresentationToWireframe();
	property->SetDiffuse(0);
	property->SetAmbient(1);
	property->SetColor( 1, 1, 0 );

	property = axes->GetZAxisTipProperty();
	property->SetRepresentationToWireframe();
	property->SetDiffuse(0);
	property->SetAmbient(1);
	property->SetColor( 0, 1, 1 );

  // set up the renderer, window, and interactor
	//
	vtkRenderer* renderer = vtkRenderer::New();
	renderer->SetBackground( 0.0980, 0.0980, 0.4392 );

	vtkRenderWindow* renWin = vtkRenderWindow::New();
	renWin->AddRenderer( renderer );
	renWin->SetSize( 400, 400 );

	vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow( renWin );

	renderer->AddViewProp( axes );

  // set up an interesting view
	//
	vtkCamera* camera = renderer->GetActiveCamera();
	camera->SetViewUp( 0, 0, 1 );
	camera->SetFocalPoint( 0, 0, 0 );
	camera->SetPosition( 4.5, 4.5, 2.5 );
	renderer->ResetCameraClippingRange();

  // this static function improves the appearance of the text edges
  // since they are overlaid on a surface rendering of the cube's faces
	//
	vtkMapper::SetResolveCoincidentTopologyToPolygonOffset();


	vtkAxesActor* axes2 = vtkAxesActor::New();

  // simulate a left-handed coordinate system
	//
	axes2->SetShaftTypeToCylinder();
	axes2->SetXAxisLabelText( "w" );
	axes2->SetYAxisLabelText( "v" );
	axes2->SetZAxisLabelText( "u" );

	axes2->SetTotalLength( 1.5, 1.5, 1.5 );
	axes2->SetCylinderRadius( 0.500 * axes2->GetCylinderRadius() );
	axes2->SetConeRadius    ( 1.025 * axes2->GetConeRadius() );
	axes2->SetSphereRadius  ( 1.500 * axes2->GetSphereRadius() );

	vtkTextProperty* tprop = axes2->GetXAxisCaptionActor2D()->
			GetCaptionTextProperty();
	tprop->ItalicOn();
	tprop->ShadowOn();
	tprop->SetFontFamilyToTimes();

	axes2->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->ShallowCopy( tprop );
	axes2->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->ShallowCopy( tprop );

  // combine orientation markers into one with an assembly
	//
	vtkPropAssembly* assembly = vtkPropAssembly::New();
	assembly->AddPart( axes2 );

  // set up the widget
	//
	vtkOrientationMarkerWidget* widget = vtkOrientationMarkerWidget::New();
	widget->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
	widget->SetOrientationMarker( assembly );
	widget->SetInteractor( iren );
	widget->SetViewport( 0.0, 0.0, 0.4, 0.4 );
	widget->SetEnabled( 1 );
	widget->InteractiveOff();
	widget->InteractiveOn();

	iren->Initialize();
	renWin->Render();

	iren->Start();

  // clean up
	//
	renderer->Delete();
	renWin->Delete();
	iren->Delete();
	axes->Delete();
	axes2->Delete();
	assembly->Delete();
	widget->Delete();

	return 0;
}