#include <iostream>

#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_box_3d.h>
#include <vgl/vgl_vector_3d.h>
#include <vgl/vgl_triangle_3d.h>
#include <vgl/vgl_plane_3d.h>
#include <vgl/vgl_line_3d_2_points.h>
#include <vgl/vgl_intersection.h>
#include <vgl/vgl_distance.h>
#include <vgl/vgl_closest_point.h>

void PointTest(void);
void VectorTest(void);
void TrianglePointInsideTest(void);
void TriangleIntersectLineGreen(void);
void TriangleIntersectLineRed(void);
void DistancePointToLine(void);
void ClosestPointOnLine(void);
void PlaneIntersectLine(void);

int main()
{	
  PointTest();
  //VectorTest();
  //TrianglePointInsideTest();
  //TriangleIntersectLineGreen();
  //TriangleIntersectLineRed();
  //PlaneIntersectLine();
  //DistancePointToLine();
  //ClosestPointOnLine();
  return 0;
}

void PointTest(void)
{
  std::cout << std::endl << "PointTest()" << std::endl
                  << "---------------" << std::endl;

  vgl_point_3d<double> a(1.0, 2.0, 3.0);
  std::cout << a << std::endl;

  a.set(4.0, a.y(), a.z());
  std::cout << "a: " << a << std::endl;

  std::cout << "a.x: " << a.x() << std::endl;

  a += vgl_vector_3d<double> (0.0, 0.0, 5.0);
  std::cout << "a: " << a << std::endl;

	
}

void VectorTest()
{
	std::cout << std::endl << "VectorTest()" << std::endl
			<< "---------------" << std::endl;

	vgl_vector_3d<double> a(1.0, 2.0, 3.0);
	std::cout << a << std::endl;

	normalize(a);
	std::cout << a << std::endl;
	
}

void TrianglePointInsideTest()
{
	std::cout << std::endl << "TrianglePointInsideTest()" << std::endl
			<< "----------------" << std::endl;
	//define the triangle with 3 points
	vgl_point_3d<double> p0(0.0,0.0,0.0);
	vgl_point_3d<double> p1(1.0,0.0,0.0);
	vgl_point_3d<double> p2(0.0,1.0,0.0);
	
	//create 2 points to test if they are inside the triangle
	vgl_point_3d<double> Test1(.5,.5,0.0);
	vgl_point_3d<double> Test2(2.0,2.0,0.0);	

	//perform the tests
	bool bInside1 = vgl_triangle_3d_test_inside(Test1,p0,p1,p2);
	bool bInside2 = vgl_triangle_3d_test_inside(Test2,p0,p1,p2);
	
	//output the results
	std::cout << "Inside1 = " << bInside1 << std::endl;
	std::cout << "Inside2 = " << bInside2 << std::endl;

}

void TriangleIntersectLineGreen()
{
	std::cout << std::endl << "TriangleIntersectLineGreen()" << std::endl
			<< "-------------------------" << std::endl;
	
	//create a triangle in the XY plane
	vgl_point_3d<double> TriP0(0.0,0.0,0.0);
	vgl_point_3d<double> TriP1(1.0,0.0,0.0);
	vgl_point_3d<double> TriP2(0.0,1.0,0.0);
	
	//create a line orthogonal to the XY plane
	vgl_point_3d<double> LineP0(.2,.2,1.0);
	vgl_point_3d<double> LineP1(.2,.2,-1.0);
	//vgl_line_3d_2_points<double> Line(LineP0, LineP1);
        vgl_line_segment_3d<double> Line(LineP0, LineP1);

	//find the intersection, if there is one
	vgl_point_3d<double> IntPoint;
	vgl_triangle_3d_intersection_t Intersect = vgl_triangle_3d_line_intersection(Line, TriP0, TriP1, TriP2, IntPoint);

	if(Intersect)
	{
		std::cout << "Intersects!" << std::endl;
		std::cout << "Intersection = " << IntPoint << std::endl;
	}
	else
	{
		std::cout << "No Intersection!" << std::endl;
	}
	
}

void TriangleIntersectLineRed()
{
	//triangle is in the XY plane
	
	std::cout << std::endl << "TriangleIntersectLineRed()" << std::endl
			<< "-------------------------" << std::endl;
	vgl_point_3d<double> LineP0(5.0,5.0,1.0);
	vgl_point_3d<double> LineP1(5.0,5.0,-1.0);

	vgl_line_segment_3d<double> Line(LineP0, LineP1);

	vgl_point_3d<double> TriP0(0.0,0.0,0.0);
	vgl_point_3d<double> TriP1(1.0,0.0,0.0);
	vgl_point_3d<double> TriP2(0.0,1.0,0.0);

	vgl_point_3d<double> IntPoint;
	vgl_triangle_3d_intersection_t Intersect = vgl_triangle_3d_line_intersection(Line, TriP0, TriP1, TriP2, IntPoint);

	if(Intersect)
	{
		std::cout << "Intersects!" << std::endl;
		std::cout << "Intersection = " << IntPoint << std::endl;
	}
	else
	{
		std::cout << "No Intersection!" << std::endl;
	}
	
}

void PlaneIntersectLine()
{
	std::cout << std::endl << "PlaneIntersectLine()" << std::endl
			<< "-------------------------" << std::endl;
	//create a line (the x axis)
	vgl_point_3d<double> LineP0(0.0, 0.0, 0.0);
	vgl_point_3d<double> LineP1(1.0, 0.0, 0.0);
	vgl_line_3d_2_points<double> Line(LineP0, LineP1);

	//create a plane (parallel to the YZ plane at x=1)
	vgl_point_3d<double> PlaneP0(1.0,0.0,0.0);
	vgl_point_3d<double> PlaneP1(1.0,0.0,1.0);
	vgl_point_3d<double> PlaneP2(1.0,1.0,0.0);
	vgl_plane_3d<double> Plane(PlaneP0, PlaneP1, PlaneP2);

	//intersect the line with the plane
	vgl_point_3d<double> Intersection = vgl_intersection(Line, Plane);

	std::cout << "Intersection: " << Intersection << std::endl;

}

void DistancePointToLine()
{
	//define a line
	vgl_point_3d<double> LineP0(0.0,0.0,0.0);
	vgl_point_3d<double> LineP1(1.0,1.0,1.0);
	vgl_line_3d_2_points<double> Line(LineP0, LineP1);

	//define a test point
	vgl_point_3d<double> Point(1.0,1.0,1.0);

	//get the distance from the point to the line
	double D = vgl_distance(Line, Point);
	
	std::cout << "Distance: " << D << std::endl;
}


void ClosestPointOnLine()
{
	//define a line
	vgl_point_3d<double> LineP0(0.0,0.0,0.0);
	vgl_point_3d<double> LineP1(1.0,1.0,1.0);
	vgl_line_3d_2_points<double> Line(LineP0, LineP1);

	//define a test point
	vgl_point_3d<double> Point(1.0,1.0,1.0);

	vgl_point_3d<double> ClosestPoint = vgl_closest_point(Line, Point);

	std::cout << "Closest Point: " << ClosestPoint << std::endl;
}

vgl_box_3d<double> BoundingBox(const std::vector<vgl_point_3d<double> > &Points)
{
	vgl_box_3d<double> BB;
	std::vector<vgl_point_3d<double>  > P;
	for(unsigned int i = 0; i < Points.size(); i++)
		P.push_back(Points[i]);
		
	vgl_box_3d_bounds(P.begin(), P.end(), BB);

	return BB;
}