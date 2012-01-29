#include <iostream>
#include <vector>

class Point
{
private:
  double x0,y0,z0;
  double x1,y1,z1;
  double x2,y2,z2;
		
public:
  Point();

  double operator[](int index)
  {
    if(index == 0)
      return x1;
    else if(index == 1)
      return y1;
    else if(index == 2)
      return z1;
  }
  
  double operator[][](int rindex, int cindex)
  {
    if(index == 0)
      return x1;
    else if(index == 1)
      return y1;
    else if(index == 2)
      return z1;
  }
  
};

Point::Point()
{
  x0 = 0.1; y0 = 1.1; z0 = 2.1;
  x1 = 3.1; y1 = 4.1; z1 = 5.1;
  x2 = 6.1; y2 = 7.1; z2 = 8.1;
}

int main(int argc, char* argv[])
{

  Point P;
	
  std::cout << P[0] << " " << P[1] << std::endl;
  
  return 0;
}
