#ifndef POINT_H
#define POINT_H

#include <iostream>

using namespace std;

class Vector;

class Point
{
	double X_, Y_, Z_;
	public:
		//////////// Accessors /////////////
		double getX() const {return X_;}
		double getY() const {return Y_;}
		double getZ() const {return Z_;}
		
		//////////// Mutators //////////////
		void setX(double X) {X_ = X;}
		void setY(double Y) {Y_ = Y;}
		void setZ(double Z) {Z_ = Z;}
		
		//////////// Constructors //////////
		Point(){}
		Point(const double x, const double y, const double z){X_ = x; Y_ = y; Z_ = z;}
		
		/////////// Operators //////////////
		Point operator+ (const Vector &V) const;
		Vector operator- (const Point &P) const;
		
		/////////// Functions /////////////
		Vector ToVector() const;
};

////////////// Non Member Functions //////////////
ostream& operator<< (ostream& output, const Point& p) ;

#endif