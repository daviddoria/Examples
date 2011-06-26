#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkIntArray.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkPoints.h>
#include <vtkTable.h>
#include <vtkDoubleArray.h>
#include <vtkKMeansStatistics.h>

//display
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <sstream>

int main(int, char*[])
{
  //create 2 clusters, one near (0,0,0) and the other near (3,3,3)
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();

  points->InsertNextPoint ( 0.0, 0.0, 0.0 );
  points->InsertNextPoint ( 3.0, 3.0, 3.0 );
  points->InsertNextPoint ( 0.1, 0.1, 0.1 );
  points->InsertNextPoint ( 3.1, 3.1, 3.1 );
  points->InsertNextPoint ( 0.2, 0.2, 0.2 );
  points->InsertNextPoint ( 3.2, 3.2, 3.2 );
  
  //get the points into the format needed for KMeans
  vtkSmartPointer<vtkTable> inputData = 
      vtkSmartPointer<vtkTable>::New();
  
  for ( int c = 0; c < 3; ++c )
    {
    vtkstd::stringstream colName;
    colName << "coord " << c;
    vtkSmartPointer<vtkDoubleArray> doubleArray = 
        vtkSmartPointer<vtkDoubleArray>::New();
    doubleArray->SetNumberOfComponents(1);
    doubleArray->SetName( colName.str().c_str() );
    doubleArray->SetNumberOfTuples(points->GetNumberOfPoints());

    
    for ( int r = 0; r < points->GetNumberOfPoints(); ++ r )
      {
      double p[3];
      points->GetPoint(r, p);
      
      doubleArray->SetValue( r, p[c] );
      }

    inputData->AddColumn( doubleArray );
    }


  vtkSmartPointer<vtkKMeansStatistics> kMeansStatistics = 
      vtkSmartPointer<vtkKMeansStatistics>::New();

  kMeansStatistics->SetInput( vtkStatisticsAlgorithm::INPUT_DATA, inputData );
  kMeansStatistics->SetColumnStatus( inputData->GetColumnName( 0 ) , 1 );
  kMeansStatistics->SetColumnStatus( inputData->GetColumnName( 1 ) , 1 );
  kMeansStatistics->SetColumnStatus( inputData->GetColumnName( 2 ) , 1 );
  //kMeansStatistics->SetColumnStatus( "Testing", 1 );
  kMeansStatistics->RequestSelectedColumns();
  kMeansStatistics->SetAssessOption( true );
  kMeansStatistics->SetDefaultNumberOfClusters( 2 );
  kMeansStatistics->Update() ;
  
  //display the results
  kMeansStatistics->GetOutput()->Dump();
  
  vtkSmartPointer<vtkIntArray> clusterArray = 
        vtkSmartPointer<vtkIntArray>::New();
  clusterArray->SetNumberOfComponents(1);
  clusterArray->SetName( "ClusterId" );
      
  for(unsigned int r = 0; r < kMeansStatistics->GetOutput()->GetNumberOfRows(); r++)
    {
    vtkVariant v = kMeansStatistics->GetOutput()->GetValue(r,kMeansStatistics->GetOutput()->GetNumberOfColumns() - 1);
    std::cout << "Point " << r << " is in cluster " << v.ToInt() << std::endl;
    clusterArray->InsertNextValue(v.ToInt());
    }
    
  vtkSmartPointer<vtkPolyData> polydata =
      vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  //polydata->GetPointData()->AddArray(clusterArray);
  polydata->GetPointData()->SetScalars(clusterArray);
  
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("output.vtp");
  writer->SetInputConnection(polydata->GetProducerPort());
  writer->Write();
  
  //display
    
  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
    vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(polydata->GetProducerPort());
  glyphFilter->Update();
  
  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(glyphFilter->GetOutputPort());
 
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetPointSize(3);
  
  //Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  //Add the actor to the scene
  renderer->AddActor(actor);
  //renderer->SetBackground(.3, .6, .3); // Background color green
 
  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = 
    vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
   renderWindowInteractor->SetInteractorStyle( style );
  
  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}