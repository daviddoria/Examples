#ifndef POINT_H
#define POINT_H

#include <iostream>
using namespace std;

class Point
{
	private:
		friend class PointFriend;
		double x,y,z,a;
		friend void OutputX();
		friend ostream& operator<<(ostream& output,  const Point &P);
		
	public:
		Point(const double xin, const double yin, const double zin) : x(xin), y(yin), z(zin) {}
};

class PointFriend
{
	private:
		//friend class Point;
	public:
		PointFriend()
		{
			Point P(1.2, 2.3, 3.4);
			cout << P.x << endl;
		}
};

ostream& operator<<(std::ostream& output, const Point &P)
{
	output << "Point: " << P.x << " " << P.y << " " << P.z << endl;
	output << P.a << endl; //can't do this without friend declaration
	return output;
}

//can't do this without defining OutputX() as a friend
void OutputX()
{
	Point P(1.2, 2.3, 3.4);
	cout << P.x << endl;
}

#endif