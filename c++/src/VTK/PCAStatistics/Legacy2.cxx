#include "vtkSmartPointer.h"
#include "vtkDoubleArray.h"
#include "vtkMultiBlockDataSet.h"
#include "vtkPCAStatistics.h"
#include "vtkStringArray.h"
#include "vtkTable.h"
#include "vtkTestUtilities.h"

int main( int argc, char* argv[] )
{
  const char m0Name[] = "M0";
  vtkSmartPointer<vtkDoubleArray> dataset1Arr = 
      vtkSmartPointer<vtkDoubleArray>::New();
  dataset1Arr->SetNumberOfComponents( 1 );
  dataset1Arr->SetName( m0Name );
  dataset1Arr->InsertNextValue(1);
  dataset1Arr->InsertNextValue(0);  
  dataset1Arr->InsertNextValue(0);
  
  const char m1Name[] = "M1";
  vtkSmartPointer<vtkDoubleArray> dataset2Arr = 
      vtkSmartPointer<vtkDoubleArray>::New();
  dataset2Arr->SetNumberOfComponents( 1 );
  dataset2Arr->SetName( m1Name );
  dataset2Arr->InsertNextValue(0);
  dataset2Arr->InsertNextValue(0);
  dataset2Arr->InsertNextValue(1);
  
  const char m2Name[] = "M2";
  vtkSmartPointer<vtkDoubleArray> dataset3Arr = 
      vtkSmartPointer<vtkDoubleArray>::New();
  dataset3Arr->SetNumberOfComponents( 1 );
  dataset3Arr->SetName( m2Name );
  dataset3Arr->InsertNextValue(0);
  dataset3Arr->InsertNextValue(0);
  dataset3Arr->InsertNextValue(0);
  
  vtkSmartPointer<vtkTable> datasetTable = 
      vtkSmartPointer<vtkTable>::New();
  datasetTable->AddColumn( dataset1Arr );
  datasetTable->AddColumn( dataset2Arr );
  datasetTable->AddColumn( dataset3Arr );

  vtkSmartPointer<vtkPCAStatistics> pcas = 
      vtkSmartPointer<vtkPCAStatistics>::New();
  pcas->SetInput( vtkStatisticsAlgorithm::INPUT_DATA, datasetTable );
  pcas->SetNormalizationSchemeByName( VTK_NORMALIZE_COVARIANCE );
  pcas->SetBasisSchemeByName( "FixedBasisEnergy" );
  pcas->SetFixedBasisEnergy( 1. - 1e-8 );

  // -- Select Column Pairs of Interest ( Learn Mode ) -- 
  pcas->SetColumnStatus( m0Name, 1 );
  pcas->SetColumnStatus( m1Name, 1 );
  pcas->RequestSelectedColumns();
  pcas->ResetAllColumnStates();
  pcas->SetColumnStatus( m0Name, 1 );
  pcas->SetColumnStatus( m1Name, 1 );
  pcas->SetColumnStatus( m2Name, 1 );
  pcas->SetColumnStatus( m2Name, 0 );
  pcas->SetColumnStatus( m2Name, 1 );
  pcas->RequestSelectedColumns();
  pcas->RequestSelectedColumns(); // Try a duplicate entry. This should have no effect.
  pcas->SetColumnStatus( m0Name, 0 );
  pcas->SetColumnStatus( m2Name, 0 );
  pcas->SetColumnStatus( "Metric 3", 1 ); // An invalid name. This should result in a request for metric 1's self-correlation.
  // pcas->RequestSelectedColumns(); will get called in RequestData()

  // -- Test Learn Mode -- 
  pcas->SetLearnOption( true );
  pcas->SetDeriveOption( true );
  pcas->SetAssessOption( false );

  pcas->Update();
  vtkSmartPointer<vtkMultiBlockDataSet> outputMetaDS = 
      vtkMultiBlockDataSet::SafeDownCast( pcas->GetOutputDataObject( vtkStatisticsAlgorithm::OUTPUT_MODEL ) );
  for ( unsigned int b = 0; b < outputMetaDS->GetNumberOfBlocks(); ++ b )
    {
    vtkSmartPointer<vtkTable> outputMeta = vtkTable::SafeDownCast( outputMetaDS->GetBlock( b ) );

    if ( b == 0 )
      {
      cout << "Raw sums\n";
      }
    else
      {
      cout << "Request " << ( b - 1 ) << "\n";
      }

    outputMeta->Dump();
    }

  // -- Test Assess Mode -- 
  vtkSmartPointer<vtkMultiBlockDataSet> paramsTables = 
      vtkSmartPointer<vtkMultiBlockDataSet>::New();
  paramsTables->ShallowCopy( outputMetaDS );

  pcas->SetInput( vtkStatisticsAlgorithm::INPUT_MODEL, paramsTables );

  // Test Assess only (Do not recalculate nor rederive a model)
  // Use SetParameter method
  pcas->SetParameter( "Learn", 0, false );
  pcas->SetParameter( "Derive", 0, false );
  pcas->SetParameter( "Assess", 0, true );
  pcas->Update();

  vtkSmartPointer<vtkTable> outputData = pcas->GetOutput();
  outputData->Dump();

  return EXIT_SUCCESS;
}
