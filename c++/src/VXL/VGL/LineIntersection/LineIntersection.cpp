#include <iostream>

#include <vgl/vgl_line_3d_2_points.h>
#include <vgl/vgl_intersection.h>

int main()
{	
  // Create a line segment
  vgl_point_3d<double> Line0P0(-10,0,0);
  vgl_point_3d<double> Line0P1(10,0,0);
  vgl_line_3d_2_points<double> Line0(Line0P0, Line0P1);

  // Create another line segment
  vgl_point_3d<double> Line1P0(0,-10,0);
  vgl_point_3d<double> Line1P1(0,10,0);
  vgl_line_3d_2_points<double> Line1(Line1P0, Line1P1);

  // Find the intersection, if there is one
  vgl_point_3d<double> intersectionPoint = vgl_intersection(Line0, Line1);
  
  return 0;
}
