#include <iostream>
#include <cstdlib> // for exit()

// Note that the output is 0 even though the object is a Square - it still calls the Shape static function.

class Shape
{
public:

  static int GetNumberOfSides()
  {
    return 0;
  }
};

class Square: public Shape
{
public:
  static int GetNumberOfSides()
  {
    return 4;
  }
};

struct Triangle : public Shape
{
public:
  static int GetNumberOfSides()
  {
    return 3;
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
  std::cout << shape1->GetNumberOfSides() << std::endl;
  return 0;
}
