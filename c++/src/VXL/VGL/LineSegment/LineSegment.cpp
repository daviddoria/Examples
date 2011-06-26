#include <iostream>

#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_line_segment_3d.h>
#include <vgl/vgl_intersection.h>

void Test3D();
void Test2D();

void TestIntersection3D();
void TestIntersection2D();


bool vgl_intersection(vgl_line_segment_2d<double> lineSegment0, vgl_line_segment_2d<double> lineSegment1, vgl_point_2d<double> &intersectionPoint)
{
  vgl_point_3d<double> lineSegment03DP0(lineSegment0.point1().x(), lineSegment0.point1().y(), 0);
  vgl_point_3d<double> lineSegment03DP1(lineSegment0.point2().x(), lineSegment0.point2().y(), 0);
  vgl_line_segment_3d<double> lineSegment03D(lineSegment03DP0, lineSegment03DP1);


  vgl_point_3d<double> lineSegment13DP0(lineSegment1.point1().x(), lineSegment1.point1().y(), 0);
  vgl_point_3d<double> lineSegment13DP1(lineSegment1.point2().x(), lineSegment1.point2().y(), 0);
  vgl_line_segment_3d<double> lineSegment13D(lineSegment13DP0, lineSegment13DP1);

  vgl_point_3d<double> intersectionPoint3D;
  bool isIntersect = vgl_intersection(lineSegment03D, lineSegment13D, intersectionPoint3D);

  intersectionPoint.set(intersectionPoint3D.x(), intersectionPoint3D.y());

  return isIntersect;
}


int main()
{	
  Test3D();
  Test2D();

  TestIntersection3D();
  TestIntersection2D();

	return 0;
}

void Test3D()
{
  // Create a line segment
  vgl_point_3d<double> LineSegmentP0(-10,0,0);
  vgl_point_3d<double> LineSegmentP1(10,0,0);
  vgl_line_segment_3d<double> LineSegment(LineSegmentP0, LineSegmentP1);

  std::cout << LineSegment.point1() << std::endl;
  std::cout << LineSegment.point1().x() << std::endl;
  
}

void Test2D()
{
  // Create another line segment
  vgl_point_2d<double> LineSegmentP0(0,-10);
  vgl_point_2d<double> LineSegmentP1(0,10);
  vgl_line_segment_2d<double> LineSegment(LineSegmentP0, LineSegmentP1);


}

void TestIntersection3D()
{
  // Create a line segment
  vgl_point_3d<double> LineSegment0P0(-10,0,0);
  vgl_point_3d<double> LineSegment0P1(10,0,0);
  vgl_line_segment_3d<double> LineSegment0(LineSegment0P0, LineSegment0P1);

  // Create another line segment
  vgl_point_3d<double> LineSegment1P0(0,-10,0);
  vgl_point_3d<double> LineSegment1P1(0,10,0);
  vgl_line_segment_3d<double> LineSegment1(LineSegment1P0, LineSegment1P1);


  vgl_point_3d<double> intersectionPoint;
  bool isIntersect = vgl_intersection(LineSegment0, LineSegment1, intersectionPoint);

  std::cout << "Intersect? " << isIntersect << std::endl;
  if(isIntersect)
    {
    std::cout << intersectionPoint << std::endl;
    }
  
}

void TestIntersection2D()
{
  // Create a line segment
  vgl_point_2d<double> LineSegment0P0(-10,0);
  vgl_point_2d<double> LineSegment0P1(10,0);
  vgl_line_segment_2d<double> LineSegment0(LineSegment0P0, LineSegment0P1);

  // Create another line segment
  vgl_point_2d<double> LineSegment1P0(0,-10);
  vgl_point_2d<double> LineSegment1P1(0,10);
  vgl_line_segment_2d<double> LineSegment1(LineSegment1P0, LineSegment1P1);

  vgl_point_2d<double> intersectionPoint;
  bool isIntersect = vgl_intersection(LineSegment0, LineSegment1, intersectionPoint);

  std::cout << "Intersect? " << isIntersect << std::endl;
  if(isIntersect)
    {
    std::cout << intersectionPoint << std::endl;
    }
}
