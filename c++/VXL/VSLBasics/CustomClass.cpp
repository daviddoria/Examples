#include <iostream>
#include <vector>
#include <cstdio>

#include <vsl/vsl_binary_io.h>
#include <vsl/vsl_vector_io.h>

#include "TestClass.h"

int main(int argc, char **argv)
{
/*
	double a = 4.5;
	vsl_b_ofstream bfs_out("test.bin");
	vsl_b_write(bfs_out, a);
	bfs_out.close();
*/
	
	/*
	//compiles:
	TestClass T(4.5, "hello");
	vsl_b_ofstream bfs_out("test.bin");
	vsl_b_write(bfs_out, T);
	bfs_out.close();
	*/
	
	TestClass T(4.5, "hello");
	std::vector<TestClass> V;
	for(unsigned int i = 0; i < 10; i++)
	{
		V.push_back(T);
	}
	
	vsl_b_ofstream bfs_out("test.bin");
	vsl_b_write(bfs_out, V);
	bfs_out.close();
	
	return 0;
}

/*
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
*/