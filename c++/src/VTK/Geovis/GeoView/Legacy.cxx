#include "vtkGeoAlignedImageRepresentation.h"
#include "vtkGeoAlignedImageSource.h"
#include "vtkGeoEdgeStrategy.h"
#include "vtkGeoFileImageSource.h"
#include "vtkGeoFileTerrainSource.h"
#include "vtkGeoGlobeSource.h"
#include "vtkGeoProjection.h"
#include "vtkGeoProjectionSource.h"
#include "vtkGeoRandomGraphSource.h"
#include "vtkGeoSphereTransform.h"
#include "vtkGeoTerrain.h"
#include "vtkGeoTerrainNode.h"
#include "vtkGeoTerrain2D.h"
#include "vtkGeoTransform.h"
#include "vtkGeoView.h"
#include "vtkGeoView2D.h"
#include "vtkGraphLayoutView.h"
#include "vtkJPEGReader.h"
#include "vtkRegressionTestImage.h"
#include "vtkRenderedGraphRepresentation.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkSmartPointer.h"
#include "vtkStdString.h"
#include "vtkTestUtilities.h"
#include "vtkTIFFReader.h"
#include "vtkViewTheme.h"
#include "vtkViewUpdater.h"

int main(int argc, char* argv[])
{
  vtkStdString imageReadPath = ".";
  vtkStdString imageSavePath = ".";
  vtkStdString terrainReadPath = ".";
  vtkStdString terrainSavePath = ".";
 
  // Create the geo view.
  vtkSmartPointer<vtkGeoView> view = vtkSmartPointer<vtkGeoView>::New();
  view->DisplayHoverTextOff();
  view->GetRenderWindow()->SetMultiSamples(0);
  view->GetRenderWindow()->SetSize(400,400);

  vtkSmartPointer<vtkGeoTerrain> terrain =
      vtkSmartPointer<vtkGeoTerrain>::New();
  vtkSmartPointer<vtkGeoSource> terrainSource;
  vtkGeoGlobeSource* globeSource = vtkGeoGlobeSource::New();
  terrainSource.TakeReference(globeSource);
  terrainSource->Initialize();
  terrain->SetSource(terrainSource);
  view->SetTerrain(terrain);

  vtkSmartPointer<vtkGeoAlignedImageRepresentation> imageRep =
      vtkSmartPointer<vtkGeoAlignedImageRepresentation>::New();
  vtkSmartPointer<vtkGeoSource> imageSource;
  vtkGeoAlignedImageSource* alignedSource = vtkGeoAlignedImageSource::New();
  vtkSmartPointer<vtkJPEGReader> reader =
      vtkSmartPointer<vtkJPEGReader>::New();
  reader->SetFileName(imageFile.c_str());
  reader->Update();
  alignedSource->SetImage(reader->GetOutput());
  imageSource.TakeReference(alignedSource);
  imageSource->Initialize();
  imageRep->SetSource(imageSource);
  view->AddRepresentation(imageRep);

  // Load databases
  if (terrainReadPath.length() > 0)
  {
    terrainSource->ShutDown();
    vtkGeoFileTerrainSource* source = vtkGeoFileTerrainSource::New();
    source->SetPath(terrainReadPath.c_str());
    terrainSource.TakeReference(source);
    terrainSource->Initialize();
  }
  terrain->SetSource(terrainSource);
  if (imageReadPath.length() > 0)
  {
    imageSource->ShutDown();
    vtkGeoFileImageSource* source = vtkGeoFileImageSource::New();
    source->SetPath(imageReadPath.c_str());
    imageSource.TakeReference(source);
    imageSource->Initialize();
  }
  imageRep->SetSource(imageSource);

  view->ResetCamera();

  // Add a graph representation
  vtkSmartPointer<vtkGeoRandomGraphSource> graphSource =
      vtkSmartPointer<vtkGeoRandomGraphSource>::New();
  graphSource->SetNumberOfVertices(100);
  graphSource->StartWithTreeOn();
  graphSource->SetNumberOfEdges(0);
  vtkSmartPointer<vtkRenderedGraphRepresentation> graphRep =
      vtkSmartPointer<vtkRenderedGraphRepresentation>::New();
  graphRep->SetInputConnection(graphSource->GetOutputPort());
  graphRep->SetLayoutStrategyToAssignCoordinates("longitude", "latitude");
  vtkSmartPointer<vtkGeoEdgeStrategy> edgeStrategy =
      vtkSmartPointer<vtkGeoEdgeStrategy>::New();
  graphRep->SetEdgeLayoutStrategy(edgeStrategy);
  view->AddRepresentation(graphRep);

  vtkViewTheme* theme = vtkViewTheme::New();
  view->ApplyViewTheme(theme);
  theme->Delete();

  //int retVal = vtkRegressionTestImage(win);
  view->Render();
  int retVal = vtkRegressionTestImageThreshold(view->GetRenderWindow(), 11);
  if (retVal == vtkRegressionTester::DO_INTERACTOR)
  {
    // Interact with data.
    view->GetInteractor()->Initialize();
    view->GetInteractor()->Start();

    retVal = vtkRegressionTester::PASSED;
  }

  // Shut down sources
  terrainSource->ShutDown();
  imageSource->ShutDown();
  imageSource2->ShutDown();

  delete [] image;
  delete [] image2;
  return !retVal;
}
