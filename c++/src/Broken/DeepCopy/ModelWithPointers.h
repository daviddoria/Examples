#ifndef MODELWITHPOITNERS_H
#define MODELWITHPOITNERS_H

#include <iostream>
#include <vector>

#include "Point.h"

using namespace std;

class ModelWithPoitners
{
	vector<Point*> Points;

	//default constructor
	ModelWithPoitners() {}

	~ModelWithPoitners()
	{
		for(unsigned int i = 0; i < Points.size(); i++)
		{
			delete Points[i];
		}

		cout << "Destructed!" << endl;
	}
	//copy constructor
	ModelWithPoitners(const ModelWithPoitners &M)
	{
		for(int i = 0; i < M.Points.size(); i++)
		{
			Point* P = new Point(M.Points[i]->x, M.Points[i]->y, M.Points[i]->z);
			Points.push_back(P);
		}
	}

	//assignment operator
	ModelWithPoitners& operator=(const ModelWithPoitners& M)
	{
		for(int i = 0; i < M.Points.size(); i++)
		{
			Point* P = new Point(M.Points[i]->x, M.Points[i]->y, M.Points[i]->z);
			Points.push_back(P);
		}
		
		return *this;
	}

};

#endif