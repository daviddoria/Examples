#include <iostream>
#include <cstdlib> // for exit()


class Shape
{
public:
  virtual void OutputNumberOfSides() = 0;
};

class Square: public Shape
{
public:
  void OutputNumberOfSides()
  {
    std::cout << "4" << std::endl;
  }
};

struct Triangle : public Shape
{
public:
  void OutputNumberOfSides()
  {
    std::cout << "3" << std::endl;
  }
};

class ShapeFactory
{
public:
  static Shape* Create(const std::string& name)
  {
    if(name.compare("square") == 0)
      {
      return new Square;
      }
    else if (name.compare("triangle") == 0)
      {
      return new Triangle;
      }
    else
      {
      std::cerr << "Shape name not recognized!" << std::endl;
      exit(-1);
      }
  }
};


int main(int argc, char *argv[])
{
  Shape* shape1 = ShapeFactory::Create("square");
  shape1->OutputNumberOfSides();
  
  Shape* shape2 = ShapeFactory::Create("triangle");
  shape2->OutputNumberOfSides();
  
  return 0;
}
