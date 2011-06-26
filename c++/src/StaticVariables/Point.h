#ifndef POINT_H
#define POINT_H

class Point
{

  public:
    Point();
    static const double X = 1.0;
};

Point::Point()
{
  std::cout << Point::X << std::endl;
}
//double Point::X = 1.0;

#endif