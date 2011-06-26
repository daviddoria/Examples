#include<iostream>
#include<cstdlib>

int main()
{
  double a[3];
  a[0] = drand48();
  a[1] = drand48();
  a[2] = drand48();
  
  std::cout << "a: " << a[0] << " " << a[1] << " " << (int)a[2];
  
  unsigned char b[3];
  b[0] = static_cast<unsigned char>(255. * a[0]);
  b[1] = static_cast<unsigned char>(255. * a[1]);
  b[2] = static_cast<unsigned char>(255. * a[2]);
  
    std::cout << "Converted " << (int)a[0] << " " << (int)a[1] << " " << (int)a[2]
	      << " to " << (int)b[0] << " " << (int)b[1] << " " << (int)b[2] << std::endl;
  return 0;
}