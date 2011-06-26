#ifndef TOOLS_H
#define TOOLS_H

#include <iostream>
#include <vector>


//MUST put the definition here
template <typename T>
void Output(std::vector<T> &V)
{
	for(unsigned int i = 0; i < V.size(); i++)
		std::cout << V[i] << std::endl;
		
}

template <typename T> 
void Display(std::vector<T> &V);

#endif

