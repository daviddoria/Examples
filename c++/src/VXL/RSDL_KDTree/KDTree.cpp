#include <iostream>
// rpl is RPI Public Library
// rsdl is RPI _ _ Library

#include <vnl/vnl_vector.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_matrix.h>
#include <rsdl/rsdl_kd_tree.h>
#include <rsdl/rsdl_point.h>

#include <vgl/vgl_point_3d.h>

//http://public.kitware.com/vxl/doc/release/contrib/rpl/rsdl/html/classrsdl__kd__tree.html
using namespace std;


int main()
{
	rsdl_point P0(vnl_double_3(1,2,3));
	rsdl_point P1(vnl_double_3(4,5,6));
	rsdl_point P2(vnl_double_3(6,7,8));
	rsdl_point P3(vnl_double_3(9,10,11));
	
	
	int n = 4;
	vcl_vector<rsdl_point> Points(n);
	Points[0] = rsdl_point(P0);
	Points[1] = rsdl_point(P1);
	Points[2] = rsdl_point(P2);
	Points[3] = rsdl_point(P3);
	
	rsdl_kd_tree Tree(Points);
	
	rsdl_point TestPoint(vnl_double_3(4.1,5.2,6.1));
	//rsdl_point TestPoint(vnl_double_3(1.1, 2.1, 3.1));
	//rsdl_point TestPoint(vnl_double_3(1.1, 2.1, 3.1));
	
	vector<rsdl_point> ClosestPoints;
	vector<int> Indices;
	Tree.n_nearest(TestPoint, 2, ClosestPoints, Indices);
		
	for(int i = 0; i < ClosestPoints.size(); i++)
	{
		cout << "Closest: " << ClosestPoints[i] << endl;
		cout << "Indices: " << Indices[i] << endl;
	}
	
	rsdl_point Pclosest = ClosestPoints[0];
	
	vgl_point_3d<double> CP(Pclosest.cartesian(0), Pclosest.cartesian(1), Pclosest.cartesian(2));
	cout << "CP: " << CP << endl;
	
	cout << "Radius Test --------------------" << endl;
	vcl_vector<rsdl_point> RadiusPoints;
	vcl_vector<int> RadiusIndices;
	Tree.points_in_radius(TestPoint, 1.0, RadiusPoints, RadiusIndices );
	
	cout << "Closest: " << RadiusPoints[0] << endl;
	cout << "Indices: " << RadiusIndices[0] << endl;
	
	
	return 0;
}