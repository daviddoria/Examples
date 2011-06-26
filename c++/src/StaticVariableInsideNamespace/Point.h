#ifndef POINT_H
#define POINT_H


namespace David
{
  class Point
  {
  public:
    typedef char Value;
    Point();
    static const Value X = 1;
    virtual void MyFunction() = 0;
  };

  class Subpoint : public Point
  {
  public:
    Subpoint();
    virtual void MyFunction();
  };

  class Subpoint2 : public Subpoint
  {
  public:
    Subpoint2();
    virtual void MyFunction();
  };
}

#endif