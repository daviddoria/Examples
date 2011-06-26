#include "itkMesh.h"
#include "itkVector.h"
#include "itkMatrix.h"
#include "itkVersor.h"
#include "itkTransformMeshFilter.h"
//#include "itkVersorRigid3DTransform.h" //6 dof
//#include "itkEuler3DTransform.h" // 7 dof
#include "itkRigid3DTransform.h" // 6 dof

typedef   float   PixelType;
const unsigned int Dimension = 3;
typedef itk::Mesh< PixelType, Dimension >   MeshType;

void DisplayMeshPoints(MeshType::Pointer  mesh);

int main(int, char *[])
{
	
	MeshType::Pointer  mesh = MeshType::New();
	
	//create points
	//corners
	MeshType::PointType p0;
	MeshType::PointType p1;
	MeshType::PointType p2;
	MeshType::PointType p3;
	MeshType::PointType p4;
	MeshType::PointType p5;
	MeshType::PointType p6;
	MeshType::PointType p7;
	//faces
	MeshType::PointType p8;
	MeshType::PointType p9;
	MeshType::PointType p10;
	MeshType::PointType p11;
	
	p0[0]= 0.0; p0[1]= 0.0; p0[2]= 0.0;
	p1[0]= 1.0; p1[1]= 0.0; p1[2]= 0.0;
	p2[0]= 0.0; p2[1]= 1.0; p2[2]= 0.0;
	p3[0]= 1.0; p3[1]= 1.0; p3[2]= 0.0;
	p4[0]= 0.0; p4[1]= 0.0; p4[2]= 1.0;
	p5[0]= 1.0; p5[1]= 0.0; p5[2]= 1.0;
	p6[0]= 0.0; p6[1]= 1.0; p6[2]= 1.0;
	p7[0]= 1.0; p7[1]= 1.0; p7[2]= 1.0;
	p8[0]= 0.5; p8[1]= 0.5; p8[2]= 0.0;
	p9[0]= 0.5; p9[1]= 0.0; p9[2]= 0.5;
	p10[0]= 0.0; p10[1]= 0.5; p10[2]= 0.5;
	p11[0]= 0.5; p11[1]= 0.5; p11[2]= 1.0;
	
	mesh->SetPoint( 0, p0 );
	mesh->SetPoint( 1, p1 );
	mesh->SetPoint( 2, p2 );
	mesh->SetPoint( 3, p3);
	mesh->SetPoint( 4, p4);
	mesh->SetPoint( 5, p5);
	mesh->SetPoint( 6, p6);
	mesh->SetPoint( 7, p7);
	mesh->SetPoint( 8, p8);
	mesh->SetPoint( 9, p9);
	mesh->SetPoint( 10, p10);
	mesh->SetPoint( 11, p11);
	
	std::cout << "Before transform:" << std::endl;
	std::cout << "Points = " << mesh->GetNumberOfPoints() << std::endl;
	
	DisplayMeshPoints(mesh);
	
	//create a transform
	typedef itk::Rigid3DTransform< double > TransformType;
	
	//rotation
	typedef itk::Matrix<double,3,3> MatrixType;
		
	typedef itk::Versor<double> VersorType;
	VersorType versor;
	versor.SetRotationAroundX(0.0);
	versor.SetRotationAroundY(0.0);
	versor.SetRotationAroundZ(M_PI/30.0);
		
	/*
	itk::Vector<double> axis;
	axis[0] = 0.0;
	axis[1] = 0.0;
	axis[2] = 1.0;
	double angle = M_PI/7.0;
	versor.Set(axis, angle);
	*/
	
	MatrixType R = versor.GetMatrix();
	std::cout << "R: \n" << R << std::endl;
	
	TransformType::Pointer transform = TransformType::New();
	transform->SetMatrix(R);
		
	//translation (offset)
	TransformType::OffsetType::ValueType ioffsetInit[3] = {5.0,0.0, 0.0};
	TransformType::OffsetType ioffset = ioffsetInit;
	transform->SetOffset( ioffset );
	
	typedef itk::TransformMeshFilter<MeshType, MeshType, TransformType> FilterType;
	FilterType::Pointer filter = FilterType::New();
	filter->SetInput(mesh);
	filter->SetTransform(transform);
	filter->Update();
	
	std::cout << "After transform:" << std::endl;
	
	DisplayMeshPoints(filter->GetOutput());
	
	//this assigns the output to a variable
	//MeshType::Pointer NewMesh = filter->GetOutput();
	//DisplayMeshPoints(NewMesh);
		
	return 0;
}

void DisplayMeshPoints(MeshType::Pointer  mesh)
{
	//access points
	typedef MeshType::PointsContainer::Iterator     PointsIterator;

	PointsIterator  pointIterator = mesh->GetPoints()->Begin(); 

	PointsIterator end = mesh->GetPoints()->End();
	while( pointIterator != end ) 
	{
		MeshType::PointType p = pointIterator.Value();  // access the point
		std::cout << p << std::endl;                    // print the point
		++pointIterator;                                // advance to next point
	}
}
