// virtual members
#include <iostream>
using namespace std;

class CPolygon {
  protected:
    int width, height;
  public:
    void set_values (int a, int b)
      { width=a; height=b; }
    virtual int area () = 0;
      //{ return (0); }
  };

class CRectangle: public CPolygon {
  public:
    int area ()
      { return (width * height); }
  };

class CTriangle: public CPolygon {
  public:
    int area ()
      { return (width * height / 2); }
      int triFunc(){return 5;}
  };

int main () 
{
  CPolygon* ppoly1 = new CRectangle;
  CPolygon* ppoly2 = new CTriangle;
  
  ppoly1->set_values (4,5);
  ppoly2->set_values (4,5);

  cout << ppoly1->area() << endl;
  cout << ppoly2->area() << endl;

  std::cout << dynamic_cast<CTriangle*>(ppoly2)->triFunc() << std::endl; //works (and should work)
  std::cout << dynamic_cast<CTriangle*>(ppoly1)->triFunc() << std::endl; //works (but seems like it shouldn't)
  return 0;
}