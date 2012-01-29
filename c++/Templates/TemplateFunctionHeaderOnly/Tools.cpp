#include "Tools.h"
#include "Point.h"

#include <iostream>
#include <vector>

using namespace std;

template <typename T>
void Display(vector<T> &V)
{
	for(unsigned int i = 0; i < V.size(); i++)
		cout << V[i] << " ";
}

template void Display<unsigned int>(vector<unsigned int> &V);
template void Display<Point>(vector<Point> &V);