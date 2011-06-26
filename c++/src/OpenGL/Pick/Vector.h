#ifndef VECTOR_H
#define VECTOR_H

#include "Point.h"

#include <cmath>
#include <string>
#include <iostream>
#include <vector>
#include <stdlib.h>

using namespace std;

class Vector
{
  double X_, Y_, Z_;

public:
	////////////Constructors//////////
	
	Vector()  : X_ (0), Y_ (0), Z_ (0){} //default
	
	Vector(const Point &P1, const Point &P2) : X_ (P2.getX() - P1.getX()), Y_ (P2.getY() - P1.getY()), Z_ (P2.getZ() - P1.getZ()){} //between two points
	
	Vector(const double x, const double y, const double z) : X_ (x), Y_ (y), Z_ (z) {}//in a direction
  
  //////////// Accessors /////////////
  double getX() const {return X_;}
  double getY() const {return Y_;}
  double getZ() const {return Z_;}

  //////////// Mutators //////////////
  void setX(double X) {X_ = X;}
  void setY(double Y) {Y_ = Y;}
  void setZ(double Z) {Z_ = Z;}

  //////////// Functions ////////////
  double Magnitude() const { return sqrt(X_*X_ + Y_*Y_ + Z_*Z_);}
  void Normalize() { *this /= Magnitude(); }
  Vector Normalized() {return *this / Magnitude();}

  Vector Cross(const Vector &V2) const;
		  
  Point PointAlong(const Point &P0, double Distance) const{return P0 + *this * Distance;}
  
  Vector operator*(const double d) const { return Vector(X_ * d, Y_ * d, Z_ * d); }
  
  void operator/= (const double d)
  {
	  X_ = X_ / d;
	  Y_ = Y_ / d;
	  Z_ = Z_ / d;
  }
  
  Vector operator/ (const double d)
  {
	return Vector(X_ / d, Y_ / d, Z_ / d);
  }
  
};


//////////// External Operators //////////////
ostream& operator<< (ostream &output, const Vector &v);

#endif
