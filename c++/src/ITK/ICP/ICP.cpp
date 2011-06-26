// This example illustrates how to perform Iterative Closest Point (ICP) 
// registration in ITK using sets of 3D points.

#include "itkEuclideanDistancePointMetric.h"
#include "itkLevenbergMarquardtOptimizer.h"
#include "itkEuler3DTransform.h" // 6 dof

//#include "itkMesh.h"
//#include "itkTransformMeshFilter.h"
#include "itkPointSetToPointSetRegistrationMethod.h"

#include "itkTransformPointSetFilter.h"

#include "itkMatrix.h"
#include "itkVector.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>


const unsigned int Dimension = 3;
typedef itk::PointSet< float, Dimension >   PointSetType;
typedef PointSetType::PointType     PointType;
typedef PointSetType::PointsContainer  PointsContainer;
typedef itk::Matrix<double, 3, 3> MatrixType;
typedef itk::Vector<double, 3> VectorType;
	  
PointSetType::Pointer ReadCSV(const std::string &strInputFile);
void WriteCSV(PointSetType::Pointer PointSet, const std::string &Filename);
PointSetType::Pointer ManuallyTransformPointSet(PointSetType::Pointer PointSet, MatrixType R, VectorType T);
	
int main(int argc, char * argv[] )
{
	//Must have 12 points to fufill 9 DOF in rotation matrix and 3 DOF in translation vector
	
	std::string strFixedFile = argv[1];
	std::string strPerturbedFile = argv[2];
	std::string strOutputFilename = argv[3];
	
	PointSetType::Pointer fixedPointSet  = ReadCSV(strFixedFile);
	std::cout << "Number of fixed Points = " << fixedPointSet->GetNumberOfPoints() << std::endl;
	
	PointSetType::Pointer movingPointSet  = ReadCSV(strPerturbedFile);
	std::cout << "Number of moving Points = " << movingPointSet->GetNumberOfPoints() << std::endl;
	
//-----------------------------------------------------------
// Set up  the Metric
//-----------------------------------------------------------
	typedef itk::EuclideanDistancePointMetric<  PointSetType,  PointSetType>  MetricType;

   typedef MetricType::TransformType                 TransformBaseType;
   typedef TransformBaseType::ParametersType         ParametersType;
   typedef TransformBaseType::JacobianType           JacobianType;

   MetricType::Pointer  metric = MetricType::New();


//-----------------------------------------------------------
// Set up a Transform
//-----------------------------------------------------------

   typedef itk::Euler3DTransform< double >      TransformType;

   TransformType::Pointer transform = TransformType::New();


  // Optimizer Type
   typedef itk::LevenbergMarquardtOptimizer OptimizerType;

   OptimizerType::Pointer      optimizer     = OptimizerType::New();
   optimizer->SetUseCostFunctionGradient(false);

  // Registration Method
   typedef itk::PointSetToPointSetRegistrationMethod< PointSetType, PointSetType > RegistrationType;


     RegistrationType::Pointer   registration  = RegistrationType::New();
     
     unsigned long   numberOfIterations =  2000;
     double          gradientTolerance  =  1e-4;   // convergence criterion
     double          valueTolerance     =  1e-4;   // convergence criterion
     double          epsilonFunction    =  1e-5;   // convergence criterion


     optimizer->SetNumberOfIterations( numberOfIterations );
     optimizer->SetValueTolerance( valueTolerance );
     optimizer->SetGradientTolerance( gradientTolerance );
     optimizer->SetEpsilonFunction( epsilonFunction );

  // Start from an Identity transform (in a normal case, the user 
  // can probably provide a better guess than the identity...
     transform->SetIdentity();

     registration->SetInitialTransformParameters( transform->GetParameters() );

  //------------------------------------------------------
  // Connect all the components required for Registration
  //------------------------------------------------------
     registration->SetMetric(        metric        );
     registration->SetOptimizer(     optimizer     );
     registration->SetTransform(     transform     );
     registration->SetFixedPointSet( fixedPointSet );
     registration->SetMovingPointSet(   movingPointSet   );

     try 
     {
	     registration->StartRegistration();
     }
     catch( itk::ExceptionObject & e )
     {
	     std::cout << e << std::endl;
	     return EXIT_FAILURE;
     }

     //std::cout << "Solution = " << transform->GetParameters() << std::endl;
     
     
     MatrixType R = transform->GetMatrix();
     std::cout << "R: " << R << std::endl;
	  
     VectorType T;
     /*
	  T[0] = transform->GetParameters()[3];
     T[1] = transform->GetParameters()[4];
     T[2] = transform->GetParameters()[5];
     */
	  T = transform->GetTranslation();
	  std::cout << "T: " << T << std::endl;
 

      // Create a Filter
	    // Declare the type for the filter
	  typedef itk::TransformPointSetFilter< PointSetType, PointSetType, TransformType  >       FilterType;
	  FilterType::Pointer filter = FilterType::New();

		// Create an  Transform
	  //TransformType::Pointer   affineTransform = TransformType::New();

	  filter->SetInput( movingPointSet );
	  
	  filter->SetTransform( transform );
	  filter->Update();

	  PointSetType::Pointer registeredPointSet = filter->GetOutput();
	  
	  PointSetType::Pointer correctedPointSet = ManuallyTransformPointSet(movingPointSet,R,T);
	  WriteCSV(correctedPointSet, "corrected.csv");
	  
     WriteCSV(registeredPointSet, strOutputFilename);
	  
     return 0;

}


PointSetType::Pointer ReadCSV(const std::string &strInputFile)
{
	PointSetType::Pointer PointSet  = PointSetType::New();
	PointsContainer::Pointer PointContainer  = PointsContainer::New();
	
	std::ifstream fin(strInputFile.c_str());
	
	if(fin == NULL)
	{	
		std::cout << "Cannot open file." << std::endl;
	}

	std::string line;
	std::stringstream linestream;
		
	unsigned int counter = 0;
	while(getline(fin, line))
	{
		PointType p;
		std::string comma;
		
		linestream.clear();
		linestream << line;
		linestream >> p[0] >> comma >> p[1] >> comma >> p[2];	
		std::cout << "Read point from " << strInputFile << ": " << p << std::endl;
		PointContainer->InsertElement(counter, p);
		counter++;
	}
		
	PointSet->SetPoints( PointContainer );
	
	return PointSet;
	
}

void WriteCSV(PointSetType::Pointer PointSet, const std::string &Filename)
{
	std::cout << "Writing " << PointSet->GetNumberOfPoints() << "Points..." << std::endl;
	
	std::ofstream fout(Filename.c_str());
	
	for(unsigned int i = 0; i < PointSet->GetNumberOfPoints(); i++)
	{
		PointType p;
		bool pointExists = PointSet->GetPoint(i, &p);
		if(!pointExists)
		{
			std::cout << "error!" << std::endl;
			exit(-1);
		}
		fout << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
	}

	fout.close();
}

PointSetType::Pointer ManuallyTransformPointSet(PointSetType::Pointer PointSet, MatrixType R, VectorType T)
{
	PointSetType::Pointer NewPointSet  = PointSetType::New();
	PointsContainer::Pointer PointContainer  = PointsContainer::New();
	
	for(unsigned int i = 0; i < PointSet->GetNumberOfPoints(); i++)
	{
		PointType p;
		PointSet->GetPoint(i, &p);
		PointContainer->InsertElement(i, R*p+T);
	}
		
	PointSet->SetPoints( PointContainer );
	
	return PointSet;
}