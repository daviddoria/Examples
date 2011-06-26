//#include <iostream>
//#include <fstream>

//VNL
//#include <vnl/vnl_real.h>
//#include <vnl/vnl_double_2.h>
#include <vnl/vnl_double_2x2.h>
#include <vnl/algo/vnl_real_eigensystem.h>

void TestEigenValuesShort();

int main()
{
	TestEigenValuesShort();
	return 0;
}

void TestEigenValuesShort()
{
	//create a matrix
	vnl_double_2x2 A;
	A(0,0) = 13;
	A(0,1) = 5;
	A(1,0) = 2;
	A(1,1) = 4;	
	
	vnl_real_eigensystem Eigs(A);
}

/*
//VGL
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_vector_3d.h>
*/

#if 0
void TestEigenValues()
{
	//create a matrix
	vnl_double_2x2 A;
	A(0,0) = 13;
	A(0,1) = 5;
	A(1,0) = 2;
	A(1,1) = 4;	
	
	std::cout << "A: " << std::endl << A << std::endl;
	
	//Compute the Eigen Values and Eigen Vectors
	vnl_real_eigensystem Eigs(A);

	//Get the Eigen Values into a workable form
	std::cout << "Vals: " << std::endl << "----------" << std::endl;
	vnl_matrix<vcl_complex<double> > Vals = Eigs.D;

	double val1 = vnl_real(Vals)(0,0);
	double val2 = vnl_real(Vals)(1,1);
	std::cout << val1 << std::endl;
	std::cout << val2 << std::endl;
	
	std::vector<double> EigenValues;
	EigenValues.push_back(val1);
	EigenValues.push_back(val2);
	
	std::cout << "Should be 3 and 14." << std::endl << std::endl;
	
	//Get the Eigen Vectors into a workable form
	
	std::cout << "Vecs: " << "-------------" << std::endl;
	std::cout << Eigs.V << std::endl;
	std::cout << "should be: vec with 3: (2 -4)" << std::endl;
	std::cout << "should be: vec with 14: (10 2)" << std::endl;
	
	vnl_double_2x2 Vecs = Eigs.Vreal;
	vnl_double_2 V1(Vecs(0,0), Vecs(1,0));
	vnl_double_2 V2(Vecs(0,1), Vecs(1,1));
	
	std::cout << "V1: " << V1 << std::endl;
	std::cout << "V2: " << V2 << std::endl;
	
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

#endif