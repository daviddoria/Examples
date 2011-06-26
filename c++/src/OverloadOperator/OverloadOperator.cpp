#include <iostream>

class Point
{
private:
  double a;

public:
  double x,y,z;

  Point(const double xin, const double yin, const double zin) : x(xin), y(yin), z(zin) {a = 5.0;}

  Point operator+(const Point &P) const;

  //Point& operator=(const Point &rhs);// const;
  Point operator=(const Point &rhs);

  friend std::ostream& operator<<(std::ostream& output,  const Point &P);

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
};

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


std::ostream& operator<<(std::ostream& output, const Point &P)
{
  output << "Point: " << P.x << " " << P.y << " " << P.z << std::endl;
  output << P.a << std::endl;
  return output;
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
  std::cout << P << std::endl;

  Point P2(1.0, 2.0, 3.0);
  std::cout << P + P2 << std::endl;

  std::cout << P[0] << " " << P[1] << std::endl;

  Point* myPoint = new Point(1,2,3);
  std::cout << myPoint[0] << std::endl;
  return 0;
}
