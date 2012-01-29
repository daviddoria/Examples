#include <iostream>

class Point
{
private:
  double a;

public:
  double x,y,z;

  Point(){}
  Point(const double xin, const double yin, const double zin) : x(xin), y(yin), z(zin) {a = 5.0;}

  Point operator+(const Point &P) const;

  //Point& operator=(const Point &rhs);// const;
  Point operator=(const Point &rhs);

  double operator[](int index)
  {
    if(index == 0)
    {
      return x;
    }
    else if(index == 1)
    {
      return y;
    }
    else if(index == 2)
    {
      return z;
    }
  }
  
  double operator()(int index)
  {
    return 2.0f;
  }
  double operator()()
  {
    return 3.0f;
  }
  
  bool operator==(const Point& other) const;
};

bool Point::operator==(const Point& other) const 
{
  if(this->x == other.x && this->y == other.y && this->z == other.z)
    {
    return true;
    }
  return false;
}

Point Point::operator+(const Point &P) const
{
  return Point(x+P.x, y+P.y, z+P.z);
}

/*
Point& Point::operator=(const Point &rhs)// const
{
  x = rhs.x;
  y = rhs.y;
  z = rhs.z;
  return this;
}
*/

Point Point::operator=(const Point &rhs)// const
{
  x = rhs.x;
  y = rhs.y;
  z = rhs.z;
  return *this;
}

bool operator<(const Point &P1, const Point &P2)
{
  if(P1.x < P2.x)
  {
    return true;
  }
  else if (P2.x < P1.x)
  {
    return false;
  }

  if (P1.y < P2.y)
  {
    return true;
  }
  else if (P2.y < P1.y)
  {
    return false;
  }

  if (P1.z < P2.z)
  {
    return true;
  }
  else if (P2.z < P1.z)
  {
    return false;
  }
}


int main(int argc, char* argv[])
{
  Point P(0.0, 1.1, 2.0);
  //std::cout << P << std::endl;

  Point P2(1.0, 2.0, 3.0);
  //std::cout << P + P2 << std::endl;

  std::cout << P[0] << " " << P[1] << std::endl;

  Point* myPoint = new Point(1,2,3);
  //std::cout << myPoint[0] << std::endl;
  
  Point a;
  std::cout << a(1) << a();
  return 0;
}
