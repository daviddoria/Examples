#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>

#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>

#include <utility> // defines std::pair
#include <list>
#include <fstream>
#include <iterator>

#include "VTK2CGAL.h"

// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;

// Point with normal vector stored in a std::pair.
typedef std::pair<Point, Vector> PointVectorPair;

int main(int argc, char *argv[])
{
	if(argc != 3)
	{
		std::cout << "Required parameters: InputFilename OutputFilename" << std::endl;
		exit(-1);
	}
  
	std::string InputFilename = argv[1];
	std::string OutputFilename = argv[2];
	
    // Reads a .xyz point set file in points[].
	std::list<PointVectorPair> points;
			
	ReadVTP(InputFilename, std::back_inserter(points));
	
    // Estimates normals direction.
    // Note: pca_estimate_normals() requires an iterator over points
    // as well as property maps to access each point's position and normal.
	const int nb_neighbors = 18; // K-nearest neighbors = 3 rings
	CGAL::pca_estimate_normals(points.begin(), points.end(),
										CGAL::First_of_pair_property_map<PointVectorPair>(),
												CGAL::Second_of_pair_property_map<PointVectorPair>(),
														nb_neighbors);

    // Orients normals.
    // Note: mst_orient_normals() requires an iterator over points
    // as well as property maps to access each point's position and normal.
	std::list<PointVectorPair>::iterator unoriented_points_begin =
			CGAL::mst_orient_normals(points.begin(), points.end(),
											 CGAL::First_of_pair_property_map<PointVectorPair>(),
													 CGAL::Second_of_pair_property_map<PointVectorPair>(),
															 nb_neighbors);

	std::cout << points.size() << std::endl;
    // Optional: delete points with an unoriented normal
    // if you plan to call a reconstruction algorithm that expects oriented normals.
	points.erase(unoriented_points_begin, points.end());
	
	std::cout << points.size() << std::endl;
	
    // Optional: after erase(), use Scott Meyer's "swap trick" to trim excess capacity
	std::list<PointVectorPair>(points).swap(points);

	vtkPoints* Points = vtkPoints::New();
	vtkDoubleArray* Normals = vtkDoubleArray::New();
	Normals->SetNumberOfComponents(3);
	
	for(std::list<PointVectorPair>::iterator it1 = points.begin(); it1 != points.end(); it1++)
	{
		PointVectorPair P = *it1;
		
		//point
		Points->InsertNextPoint(P.first.x(), P.first.y(), P.first.z());
		
		//normal
		double N[3] = {P.second.x(), P.second.y(), P.second.z()};
		Normals->InsertNextTupleValue(N);
	}
	
	std::cout << "num points: " << Points->GetNumberOfPoints() << std::endl;
	std::cout << "num normals: " << Normals->GetNumberOfTuples() << std::endl;
	
	vtkPolyData* Polydata = vtkPolyData::New();
	Polydata->SetPoints(Points);
	Polydata->GetPointData()->SetNormals(Normals);
	
	//write
	vtkXMLPolyDataWriter* Writer = vtkXMLPolyDataWriter::New();
	Writer->SetFileName(OutputFilename.c_str());
	Writer->SetInput(Polydata);
	Writer->Write();
	
	std::cout << points.size() << std::endl;
	
	//cleanup
	Points->Delete();
	Normals->Delete();
	Polydata->Delete();
	Writer->Delete();
	return EXIT_SUCCESS;
}


