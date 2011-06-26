#include "itkVector.h"
#include "itkListSample.h"
#include "itkWeightedCentroidKdTreeGenerator.h"
#include "itkEuclideanDistanceMetric.h"

int main()
{
  typedef itk::Vector< float, 2 > MeasurementVectorType;

  typedef itk::Statistics::ListSample< MeasurementVectorType > SampleType;
  SampleType::Pointer sample = SampleType::New();
  sample->SetMeasurementVectorSize( 2 );

  MeasurementVectorType mv;
  for (unsigned int i = 0 ; i < 1000 ; ++i )
    {
    mv[0] = (float) i;
    mv[1] = (float) ((1000 - i) / 2 );
    sample->PushBack( mv );
    }

  typedef itk::Statistics::KdTreeGenerator< SampleType > TreeGeneratorType;
  TreeGeneratorType::Pointer treeGenerator = TreeGeneratorType::New();

  treeGenerator->SetSample( sample );
  treeGenerator->SetBucketSize( 16 );
  treeGenerator->Update();

  typedef itk::Statistics::WeightedCentroidKdTreeGenerator< SampleType >
    CentroidTreeGeneratorType;

  CentroidTreeGeneratorType::Pointer centroidTreeGenerator =
                                         CentroidTreeGeneratorType::New();

  centroidTreeGenerator->SetSample( sample );
  centroidTreeGenerator->SetBucketSize( 16 );
  centroidTreeGenerator->Update();

  typedef TreeGeneratorType::KdTreeType TreeType;
  typedef TreeType::NearestNeighbors NeighborsType;
  typedef TreeType::KdTreeNodeType NodeType;

  TreeType::Pointer tree = treeGenerator->GetOutput();
  TreeType::Pointer centroidTree = centroidTreeGenerator->GetOutput();

  NodeType* root = tree->GetRoot();

  if ( root->IsTerminal() )
    {
    std::cout << "Root node is a terminal node." << std::endl;
    }
  else
    {
    std::cout << "Root node is not a terminal node." << std::endl;
    }

  unsigned int partitionDimension;
  float partitionValue;
  root->GetParameters( partitionDimension, partitionValue);
  std::cout << "Dimension chosen to split the space = "
            << partitionDimension << std::endl;
  std::cout << "Split point on the partition dimension = "
            << partitionValue << std::endl;

  std::cout << "Address of the left chile of the root node = "
            << root->Left() << std::endl;

  std::cout << "Address of the right chile of the root node = "
            << root->Right() << std::endl;

  root = centroidTree->GetRoot();
  std::cout << "Number of the measurement vectors under the root node"
            << " in the tree hierarchy = " << root->Size() << std::endl;

  NodeType::CentroidType centroid;
  root->GetWeightedCentroid( centroid );
  std::cout << "Sum of the measurement vectors under the root node = "
            << centroid << std::endl;

  std::cout << "Number of the measurement vectors under the left child"
            << " of the root node = " << root->Left()->Size() << std::endl;

  MeasurementVectorType queryPoint;
  queryPoint[0] = 10.0;
  queryPoint[1] = 7.0;

  typedef itk::Statistics::EuclideanDistanceMetric< MeasurementVectorType >
    DistanceMetricType;
  DistanceMetricType::Pointer distanceMetric = DistanceMetricType::New();

  DistanceMetricType::OriginType origin( 2 );
  for ( unsigned int i = 0 ; i < sample->GetMeasurementVectorSize() ; ++i )
    {
    origin[i] = queryPoint[i];
    }
  distanceMetric->SetOrigin( origin );

  unsigned int numberOfNeighbors = 3;
  TreeType::InstanceIdentifierVectorType neighbors;
  tree->Search( queryPoint, numberOfNeighbors, neighbors ) ;

  std::cout << "kd-tree knn search result:" << std::endl
            << "query point = [" << queryPoint << "]" << std::endl
            << "k = " << numberOfNeighbors << std::endl;
  std::cout << "measurement vector : distance" << std::endl;
  for ( unsigned int i = 0 ; i < numberOfNeighbors ; ++i )
    {
    std::cout << "[" << tree->GetMeasurementVector( neighbors[i] )
              << "] : "
              << distanceMetric->Evaluate(
                  tree->GetMeasurementVector( neighbors[i] ))
              << std::endl;
    }

  centroidTree->Search( queryPoint, numberOfNeighbors, neighbors ) ;
  std::cout << "weighted centroid kd-tree knn search result:" << std::endl
            << "query point = [" << queryPoint << "]" << std::endl
            << "k = " << numberOfNeighbors << std::endl;
  std::cout << "measurement vector : distance" << std::endl;
  for ( unsigned int i = 0 ; i < numberOfNeighbors ; ++i )
    {
    std::cout << "[" << centroidTree->GetMeasurementVector( neighbors[i] )
              << "] : "
              << distanceMetric->Evaluate(
                  centroidTree->GetMeasurementVector( neighbors[i]))
              << std::endl;
    }

  double radius = 437.0;

  tree->Search( queryPoint, radius, neighbors ) ;

  std::cout << "kd-tree radius search result:" << std::endl
            << "query point = [" << queryPoint << "]" << std::endl
            << "search radius = " << radius << std::endl;
  std::cout << "measurement vector : distance" << std::endl;
  for ( unsigned int i = 0 ; i < neighbors.size() ; ++i )
    {
    std::cout << "[" << tree->GetMeasurementVector( neighbors[i] )
              << "] : "
              << distanceMetric->Evaluate(
                  tree->GetMeasurementVector( neighbors[i]))
              << std::endl;
    }

  centroidTree->Search( queryPoint, radius, neighbors ) ;
  std::cout << "weighted centroid kd-tree radius search result:" << std::endl
            << "query point = [" << queryPoint << "]" << std::endl
            << "search radius = " << radius << std::endl;
  std::cout << "measurement vector : distance" << std::endl;
  for ( unsigned int i = 0 ; i < neighbors.size() ; ++i )
    {
    std::cout << "[" << centroidTree->GetMeasurementVector( neighbors[i] )
              << "] : "
              << distanceMetric->Evaluate(
                  centroidTree->GetMeasurementVector( neighbors[i]))
              << std::endl;
    }

  return EXIT_SUCCESS;
}