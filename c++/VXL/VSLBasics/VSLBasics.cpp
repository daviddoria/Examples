#include <iostream>
#include <vector>
#include <cstdio>

#include <assert.h>

//for random numbers
#include <cstdlib>
#include <ctime>

#include <vnl/vnl_double_3x3.h>

#include <vsl/vsl_binary_io.h>
#include <vsl/vsl_vector_io.h>
#include <vnl/io/vnl_io_matrix_fixed.h>
#include <vnl/io/vnl_io_matrix.h>
#include <vnl/io/vnl_io_vector.h>

std::string Filename = "Test.bin";
vnl_double_3x3 RandomFixedMatrix();
vnl_matrix<double> RandomMatrix();

void WriteDouble();
void ReadDouble();

void WriteMatrix();
void ReadMatrix();

void WriteVector();
void ReadVector();

void WriteVectorOfFixedMatrix();
void ReadVectorOfFixedMatrix();

void WriteVectorOfMatrix();
void ReadVectorOfMatrix();

template <typename T>
void OutputVector(const std::vector<T> &V)
{
	for(unsigned int i = 0; i < V.size(); i++)
		std::cout << V[i] << std::endl;

	std::cout << std::endl;
}

int main(int argc, char **argv)
{
	std::cout << "VSL Basics" << "--------------" << std::endl;
	
	//WriteDouble();
	//ReadDouble();

	//WriteVector();
	//ReadVector();
		
	//WriteMatrix();
	//ReadMatrix();

	//WriteVectorOfFixedMatrix();
	//ReadVectorOfFixedMatrix();
	
	WriteVectorOfMatrix();
	ReadVectorOfMatrix();

	return 0;
}


void WriteDouble()
{
	double a = 4.5;
	vsl_b_ofstream bfs_out(Filename);
	vsl_b_write(bfs_out, a);
	bfs_out.close();
}

void ReadDouble()
{
	vsl_b_ifstream input(Filename);
	double a;
	if(!(!input))
	{
		vsl_b_read(input, a);
		std::cout << a << std::endl;
	}
	else
	{
		assert(0);
	}
	
	input.close();
}

void WriteVector()
{	
	
	unsigned int num = 10;
	std::vector<double> a(num);
	
	srand48((unsigned)time(0));
	for(unsigned int i = 0; i < num; i++)
		a[i] = drand48();
	
	OutputVector(a);
	std::cout << std::endl << std::endl;
	
	vsl_b_ofstream bfs_out(Filename);
	vsl_b_write(bfs_out, a);
	bfs_out.close();
	
}

void ReadVector()
{
	vsl_b_ifstream input(Filename);
	std::vector<double> a;
	if(!(!input)) //for some reason, you cannot do if(input)
	{
		vsl_b_read(input, a);
	}
	else
	{
		assert(0);
	}
	
	OutputVector(a);
	std::cout << std::endl << std::endl;
	
	input.close();
}


void WriteMatrix()
{
	//create an identity matrix
	vnl_double_3x3 a;
	a.set_identity();
	
	std::cout << a << std::endl;
	
	vsl_b_ofstream output(Filename);
	vsl_b_write(output, a);
	
	output.close();
	
}

void ReadMatrix()
{
	vsl_b_ifstream input(Filename);
	vnl_double_3x3 a;
	if(!(!input))
	{
		vsl_b_read(input, a);
		std::cout << a << std::endl;
	}
	else
	{
		assert(0);
	}
	
	input.close();
}

#include <vsl/vsl_vector_io.txx>
#include <vnl/io/vnl_io_matrix_fixed.txx>
//VSL_VECTOR_IO_INSTANTIATE(vnl_matrix_fixed<double,3,3>);
VSL_VECTOR_IO_INSTANTIATE(vnl_double_3x3);

void WriteVectorOfFixedMatrix()
{
	unsigned int num = 10;
	std::vector<vnl_double_3x3> a(num);
	
	srand48((unsigned)time(0));
	for(unsigned int i = 0; i < num; i++)
		a[i] = RandomFixedMatrix();
	
	OutputVector(a);
	std::cout << std::endl << std::endl;
	
	vsl_b_ofstream output(Filename);
	vsl_b_write(output, a);
	output.close();
}

void ReadVectorOfFixedMatrix()
{
	
	vsl_b_ifstream input(Filename);
	std::vector<vnl_double_3x3> a;
	if(!(!input))
	{
		vsl_b_read(input, a);
		OutputVector(a);
	}
	else
	{
		assert(0);
	}
	
	input.close();
	
}

//VSL_VECTOR_IO_INSTANTIATE(vnl_matrix<double>);
void WriteVectorOfMatrix()
{
	unsigned int num = 10;
	std::vector<vnl_matrix<double> > a(num);
	
	srand48((unsigned)time(0));
	for(unsigned int i = 0; i < num; i++)
		a[i] = RandomMatrix();
	
	OutputVector(a);
	std::cout << std::endl << std::endl;
	
	vsl_b_ofstream output(Filename);
	vsl_b_write(output, a);
	output.close();
}

void ReadVectorOfMatrix()
{
	
	vsl_b_ifstream input(Filename);
	std::vector<vnl_matrix<double> > a;
	if(!(!input))
	{
		vsl_b_read(input, a);
		OutputVector(a);
	}
	else
	{
		assert(0);
	}
	
	input.close();
	
}


vnl_double_3x3 RandomFixedMatrix()
{
	vnl_double_3x3 a;
	a(0,0) = drand48();
	a(0,1) = drand48();
	a(0,2) = drand48();
	a(1,0) = drand48();
	a(1,1) = drand48();
	a(1,2) = drand48();
	a(2,0) = drand48();
	a(2,1) = drand48();
	a(2,2) = drand48();
	
	return a;
}

vnl_matrix<double> RandomMatrix()
{
	vnl_matrix<double> a(3,3);
	a(0,0) = drand48();
	a(0,1) = drand48();
	a(0,2) = drand48();
	a(1,0) = drand48();
	a(1,1) = drand48();
	a(1,2) = drand48();
	a(2,0) = drand48();
	a(2,1) = drand48();
	a(2,2) = drand48();
	
	return a;
}