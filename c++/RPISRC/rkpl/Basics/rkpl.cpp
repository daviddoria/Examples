#include <iostream>
#include <fstream>

//VNL
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_inverse.h>

#include <vnl/vnl_real.h>

#include <vnl/vnl_double_3.h>
#include <vnl/vnl_double_2.h>
#include <vnl/vnl_double_3x3.h>
#include <vnl/vnl_double_2x2.h>

#include <vnl/algo/vnl_real_eigensystem.h>

//VCL
//#include <vcl/vcl_complex.h>

//VGL
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_vector_3d.h>

using std::cout;
using std::endl;

void TestBoolMatrix();
void TestMatrix();
void TestVector();
void TestMatrixInverse();
void TestIdentity();
void TestOuterProduct();
void TestCrossProduct();
void TestTranspose();
void TestFixedSizeVector();
void TestFixedSizeMatrix();
vgl_vector_3d<double> vgl_point_to_vgl_vector(vgl_point_3d<double> &p);
void TestEigenValues();
void WriteReadMatrix();

//////// Helper Declarations ////////
vnl_matrix<double> VectorToMatrix(const vnl_double_3 &A);

int main()
{
	cout << "VNL Test" << endl;
	
	//TestVector();
	TestMatrix();
	
	//TestIdentity();
	//TestOuterProduct();
	//TestTranspose();

	//TestFixedSizeVector();
	//TestFixedSizeMatrix();

	//TestEigenValues();

	//WriteReadMatrix();
	return 0;
}

void TestCrossProduct()
{
	vnl_double_3 a(1.0, 2.0, 3.0);
	vnl_double_3 b(1.1, 2.5, 3.6);
	vnl_double_3 c = vnl_cross_3d(a,b);
	cout << a << endl;
	cout << b << endl;
	cout << c << endl;
}

void TestBoolMatrix()
{
	//use VBL!!

}

void TestMatrix()
{
	
	vnl_matrix<double> A(3,3); 
	cout << "Rows: " << A.rows() << endl;
	cout << "Cols: " << A.columns() << endl;
	A(0,0) = 1.0;
	A(0,1) = 2.0;
	A(0,2) = 3.0;
	A(1,0) = 3.0;
	A(1,1) = 2.0;
	A(1,2) = 4.0;
	A(2,0) = 5.0;
	A(2,1) = 6.0;
	A(2,2) = 9.0;
	cout << "A: " << A << endl;
	
	vnl_vector< double > p(3);
	p(0) = 2.0;
	p(1) = 4.0;
	p(2) = 6.0;

	cout << "p: " << p << endl;
	
	cout << "A*p: " << A*p << endl;
	
	//should be (28, 38, 88)
	
}

void TestVector()
{
	vnl_vector<double> x(3);
	x(0) = 10;
	x(1) = 20;
	x(2) = 30;
	
	cout << "x = " << x << endl;
	
}


void TestMatrixInverse()
{
	vnl_matrix<double> A(3,2); 
	cout << "Rows: " << A.rows() << endl;
	cout << "Cols: " << A.columns() << endl;
	A(0,0) = 1;
	A(0,1) = 2;
	A(1,0) = 4;
	A(1,1) = 3;
	A(2,0) = 6;
	A(2,1) = 3;
	cout << "A = " << A << endl;
	
	vnl_vector< double > b(3);
	b(0) = 4;
	b(1) = 5;
	b(2) = 6;
	
	cout << "b = " << b << endl;
	
	//Ax - b = 0, solution should be x = (.0563, 1.7817)
	vnl_vector<double> x(3);
	//x = vnl_matrix_inverse<double> (A) * b;
	x = vnl_inverse(A) * b;
	
	cout << "x = " << x << endl;
	
}

void TestIdentity()
{
	vnl_matrix<double> A(3,3); 
	A.set_identity();
	cout << "A = " << A << endl;
	 
}

void TestOuterProduct()
{

	vnl_double_3 A(1,2,3);
	vnl_matrix<double> A_mat = VectorToMatrix(A);
	cout << "A = " << endl << A << endl;
	cout << "A = " << endl << A_mat << endl;
	
	//vnl_double_3 AT = A.transpose();
	vnl_double_3 B(4,5,6);
	vnl_matrix<double> B_mat = VectorToMatrix(B);
	cout << "B = " << endl << B << endl;
	cout << "B = " << endl << B_mat << endl;
	cout << "Bt = " << endl << B_mat.transpose() << endl;
	
	vnl_matrix<double> OP = A_mat*B_mat.transpose();
	
	cout << "OP = " << endl << OP << endl;
}

void TestTranspose()
{
	vnl_matrix<double> A(3,2); 
	cout << "Rows: " << A.rows() << endl;
	cout << "Cols: " << A.columns() << endl;
	A(0,0) = 1;
	A(0,1) = 2;
	A(1,0) = 4;
	A(1,1) = 3;
	A(2,0) = 6;
	A(2,1) = 3;
	cout << "A = " << endl << A << endl;
	
	cout << "A^T = " << endl << A.transpose() << endl;
}

void TestFixedSizeVector()
{
	vnl_double_3 a(1,2,3);
	std::cout << a << std::endl;
	
	vnl_double_3 b(4,5,6);

	double dot = dot_product(a,b);

	cout << "Dot: " << dot << endl;

}

void TestFixedSizeMatrix()
{
	vnl_double_3x3 A;
	A(0,0) = 1;
	A(0,1) = 2;
	A(0,2) = 3;
	A(1,0) = 4;
	A(1,1) = 5;
	A(1,2) = 6;
	A(2,0) = 7;
	A(2,1) = 8;
	A(2,2) = 9;

	cout << "A: " << endl << A << endl;

	cout << "A^T = " << endl << A.transpose() << endl;

}

void WriteReadMatrix()
{
	vnl_double_3x3 A;
	A(0,0) = 1;
	A(0,1) = 2;
	A(0,2) = 3;
	A(1,0) = 4;
	A(1,1) = 5;
	A(1,2) = 6;
	A(2,0) = 7;
	A(2,1) = 8;
	A(2,2) = 9;

	std::cout << "A: " << std::endl << A << endl;
	std::ofstream OutputStream("Test.txt");
	OutputStream << A;
	OutputStream.close();

	std::ifstream InputStream("Test.txt");
	vnl_double_3x3 R; //no member named 'read
	InputStream >> R;
	std::cout << "Input: " << std::endl << R;

}

void TestEigenValues()
{
	//create a matrix
	vnl_double_2x2 A;
	A(0,0) = 13;
	A(0,1) = 5;
	A(1,0) = 2;
	A(1,1) = 4;	
	
	cout << "A: " << endl << A << endl;
	
	//Compute the Eigen Values and Eigen Vectors
	vnl_real_eigensystem Eigs(A);

	//Get the Eigen Values into a workable form
	cout << "Vals: " << endl << "----------" << endl;
	vnl_matrix<vcl_complex<double> > Vals = Eigs.D;

	double val1 = vnl_real(Vals)(0,0);
	double val2 = vnl_real(Vals)(1,1);
	cout << val1 << endl;
	cout << val2 << endl;
	
	std::vector<double> EigenValues;
	EigenValues.push_back(val1);
	EigenValues.push_back(val2);
	
	cout << "Should be 3 and 14." << endl << endl;
	
	//Get the Eigen Vectors into a workable form
	
	cout << "Vecs: " << "-------------" << endl;
	cout << Eigs.V << endl;
	cout << "should be: vec with 3: (2 -4)" << endl;
	cout << "should be: vec with 14: (10 2)" << endl;
	
	vnl_double_2x2 Vecs = Eigs.Vreal;
	vnl_double_2 V1(Vecs(0,0), Vecs(1,0));
	vnl_double_2 V2(Vecs(0,1), Vecs(1,1));
	
	cout << "V1: " << V1 << endl;
	cout << "V2: " << V2 << endl;
	
	std::vector<vnl_double_2> EigenVectors;
	EigenVectors.push_back(V1);
	EigenVectors.push_back(V2);
		/*
	cout << Vals << endl;
	cout << Vals(0,0) << endl;
	cout << vnl_real(Vals) << endl;
		*/
	
	//double E1 = Eigs.D(0,0);
	
	//cout << Eigs.D << endl;
}

////////// Helpers /////////////
vnl_matrix<double> VectorToMatrix(const vnl_double_3 &A)
{
	vnl_matrix<double> M(3,1);
	M(0,0) = A(0);
	M(1,0) = A(1);
	M(2,0) = A(2);

	return M;
}