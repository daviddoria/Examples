#include "itkPoint.h"
#include "itkPointSet.h"

#include "itkPointLocator2.h" //from QuadEdgeMeshRigidRegistration

int main()
{
	typedef float PixelType;
	const unsigned int Dimension = 3;
	typedef itk::PointSet< PixelType, Dimension >   PointSetType;
	typedef PointSetType::PointType PointType;
	typedef PointSetType::PointsContainerPointer PointsContainerPointer;
		
	PointSetType::Pointer  PointSet = PointSetType::New();
	PointsContainerPointer  points = PointSet->GetPoints();
	
	//create points
	PointType p0, p1, p2, p3, p4, p5, p6, p7, p8;

	p0[0]=  0.0; p0[1]= 0.0; p0[2]= 0.0;
	points->InsertElement( 0, p0 );
	p1[0]=  0.1; p1[1]= 0.0; p1[2]= 0.0;
	points->InsertElement( 1, p1 );
	p2[0]=  0.0; p2[1]= 0.1; p2[2]= 0.0;
	points->InsertElement( 2, p2 );
	p3[0]=  5.0; p3[1]=  5.0; p3[2]= 5.0;
	points->InsertElement( 3, p3 );
	p4[0]=  5.1; p4[1]=  5.0; p4[2]= 5.0;
	points->InsertElement( 4, p4 );
	p5[0]=  5.0; p5[1]=  5.1; p5[2]= 5.0;
	points->InsertElement( 5, p5 );
	p6[0]=  -5.0; p6[1]=  -5.0; p6[2]= -5.0;
	points->InsertElement( 6, p6 );
	p7[0]=  -5.1; p7[1]=  -5.0; p7[2]= -5.0;
	points->InsertElement( 7, p7 );
	p8[0]=  -5.0; p8[1]=  -5.1; p8[2]= -5.0;
	points->InsertElement( 8, p8 );
	
	typedef itk::PointLocator2<PointSetType> PointLocatorType;
	PointLocatorType::Pointer locator = PointLocatorType::New();
		
	locator->SetPointSet( PointSet );
	locator->Initialize();
		
	PointType query;
	query[0] = 0.12; query[1] = 0.0; query[2] = 0.0;
	PointLocatorType::PointType point( query );
	
	typedef PointLocatorType::InstanceIdentifierVectorType InstanceIdentifierVectorType;
	InstanceIdentifierVectorType result1;
	
	locator->Search( point, static_cast<unsigned int> (2), result1 );
	
	/*
	//equivalent to this: (must do one of these or the Search() overload is ambiguous
	unsigned int NumNeighbors = 2;
	locator->Search( point, NumNeighbors, result );
	*/
	
	std::cout << "Result: " << result1[0] << " " << result1[1] << std::endl;

	///////////////////////////////////////
	PointType query2;
	query2[0] = -5.1; query2[1] = -4.8; query2[2] = -4.9;
	PointLocatorType::PointType point2( query2 );
	
	InstanceIdentifierVectorType result2;
	locator->Search( point2, 1.0, result2 );
	for(unsigned int i = 0; i < result2.size(); i++)
	{
		std::cout << result2[i] << " ";
	}
	
	return 0;
}

