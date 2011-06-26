#include "itkMesh.h"

#include "itkVector.h"
#include "itkListSample.h"

#include "itkKdTree.h"
#include "itkWeightedCentroidKdTreeGenerator.h"
//#include "itkKdTreeGenerator.h"

#include "itkKdTreeBasedKmeansEstimator.h"

#include "itkMinimumDecisionRule.h"
#include "itkEuclideanDistance.h"
#include "itkSampleClassifier.h"

#include "itkNormalVariateGenerator.h"

//see also Examples/Statistics/KdTreeBasedKMeansClustering.cxx

int main()
{
	/*
	typedef   float   PixelType;
	const unsigned int Dimension = 3;
	typedef itk::Mesh< PixelType, Dimension >   MeshType;
	MeshType::Pointer  mesh = MeshType::New();
	*/
	
	const unsigned int dim = 3;
	typedef itk::Vector< double, dim > MeasurementVectorType;

	
	//create points
	MeasurementVectorType p0, p1, p2, p3, p4, p5, p6, p7, p8;

	//cluster 1
	p0[0]=  0.0; p0[1]= 0.0; p0[2]= 0.0;
	p1[0]=  0.1; p1[1]= 0.0; p1[2]= 0.0;
	p2[0]=  0.0; p2[1]= 0.1; p2[2]= 0.0;
	
	//cluster 2
	p3[0]=  5.0; p3[1]=  5.0; p3[2]= 5.0;
	p4[0]=  5.1; p4[1]=  5.0; p4[2]= 5.0;
	p5[0]=  5.0; p5[1]=  5.1; p5[2]= 5.0;
	
	//cluster 3
	p6[0]=  -5.0; p6[1]=  -5.0; p6[2]= -5.0;
	p7[0]=  -5.1; p7[1]=  -5.0; p7[2]= -5.0;
	p8[0]=  -5.0; p8[1]=  -5.1; p8[2]= -5.0;
	
	typedef itk::Statistics::ListSample< MeasurementVectorType > SampleType;
	SampleType::Pointer sample = SampleType::New();
	
	sample->PushBack(p0);
	sample->PushBack(p1);
	sample->PushBack(p2);
	sample->PushBack(p3);
	sample->PushBack(p4);
	sample->PushBack(p5);
	sample->PushBack(p6);
	sample->PushBack(p7);
	sample->PushBack(p8);
	
	typedef itk::Statistics::WeightedCentroidKdTreeGenerator< SampleType > TreeGeneratorType;
	//typedef itk::Statistics::KdTreeGenerator< SampleType > TreeGeneratorType;
	TreeGeneratorType::Pointer treeGenerator = TreeGeneratorType::New();
	treeGenerator->SetSample( sample );
	treeGenerator->SetBucketSize( 3 ); //number of measurement vectors in a terminal node
	treeGenerator->Update();
	

	// Once we have the k-d tree, it is a simple procedure to produce k mean estimates
	
	typedef TreeGeneratorType::KdTreeType TreeType;
	typedef itk::Statistics::KdTreeBasedKmeansEstimator<TreeType> EstimatorType;
	EstimatorType::Pointer estimator = EstimatorType::New();
	
	MeasurementVectorType InitialMean0;
	InitialMean0[0] = 1.0;
	InitialMean0[1] = 1.0;
	InitialMean0[2] = 1.0;
	std::cout << "InitialMean0: " << InitialMean0 << std::endl;
	
	MeasurementVectorType InitialMean1;
	InitialMean1[0] = -1.0;
	InitialMean1[1] = -1.0;
	InitialMean1[2] = -1.0;
	std::cout << "InitialMean1: " << InitialMean1 << std::endl;
	
	MeasurementVectorType InitialMean2;
	InitialMean2[0] = 0.0;
	InitialMean2[1] = 0.0;
	InitialMean2[2] = 0.0;
	std::cout << "InitialMean2: " << InitialMean2 << std::endl;
	
	unsigned int NumClasses = 3;
	
	EstimatorType::ParametersType initialMeans(NumClasses * dim);
	/*
	//expects a linear array, so can't do this:
	initialMeans[0] = InitialMean0;
	initialMeans[1] = InitialMean1;
	initialMeans[2] = InitialMean2;
	*/
	initialMeans[0] = InitialMean0[0];
	initialMeans[1] = InitialMean0[1];
	initialMeans[2] = InitialMean0[2];
	initialMeans[3] = InitialMean1[0];
	initialMeans[4] = InitialMean1[1];
	initialMeans[5] = InitialMean1[2];
	initialMeans[6] = InitialMean2[0];
	initialMeans[7] = InitialMean2[1];
	initialMeans[8] = InitialMean2[2];
		
	estimator->SetParameters( initialMeans );
	estimator->SetKdTree( treeGenerator->GetOutput() );
	estimator->SetMaximumIteration( 200 );
	estimator->SetCentroidPositionChangesThreshold(0.0);
	estimator->StartOptimization();
	
	EstimatorType::ParametersType estimatedMeans = estimator->GetParameters();
	

	for ( unsigned int i = 0 ; i < NumClasses * dim ; ++i )
	{
		std::cout << "cluster[" << i << "] " << std::endl;
		std::cout << "    estimated mean : " << estimatedMeans[i] << std::endl;
	}

	typedef itk::Statistics::EuclideanDistance< MeasurementVectorType > 
	MembershipFunctionType;
	typedef itk::MinimumDecisionRule DecisionRuleType;
	DecisionRuleType::Pointer decisionRule = DecisionRuleType::New();
	
	typedef itk::Statistics::SampleClassifier< SampleType > ClassifierType;
	ClassifierType::Pointer classifier = ClassifierType::New();
	
	classifier->SetDecisionRule( (itk::DecisionRuleBase::Pointer) decisionRule);
	classifier->SetSample( sample );
	classifier->SetNumberOfClasses( dim );
	
	
	std::vector< unsigned int > classLabels;
	classLabels.resize( dim );
	classLabels[0] = 10;
	classLabels[1] = 20;
	classLabels[2] = 30;
	
	
	/*
	//cant do this:
	std::vector< unsigned char > classLabels;
	classLabels.resize( dim );
	classLabels[0] = 'a';
	classLabels[1] = 'b';
	classLabels[2] = 'c';
	*/
	
	classifier->SetMembershipFunctionClassLabels( classLabels );
	
	std::vector< MembershipFunctionType::Pointer > membershipFunctions;
	MembershipFunctionType::OriginType origin( sample->GetMeasurementVectorSize() );
	int index = 0;
	for ( unsigned int i = 0 ; i < dim ; i++ ) 
	{
		membershipFunctions.push_back( MembershipFunctionType::New() );
		for ( unsigned int j = 0 ; j < sample->GetMeasurementVectorSize(); j++ )
		{
			origin[j] = estimatedMeans[index++];
		}
		membershipFunctions[i]->SetOrigin( origin );
		classifier->AddMembershipFunction( membershipFunctions[i].GetPointer() );
	}
	
	classifier->Update();
	
	ClassifierType::OutputType* membershipSample = classifier->GetOutput();
	ClassifierType::OutputType::ConstIterator iter = membershipSample->Begin();
	
	while ( iter != membershipSample->End() )
	{
		std::cout << "measurement vector = " << iter.GetMeasurementVector()
		<< "class label = " << iter.GetClassLabel()
		<< std::endl;
		++iter;
	}

  return 0;
}

