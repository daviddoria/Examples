#ifndef CGAL_READ_XYZ_POINTS_H
#define CGAL_READ_XYZ_POINTS_H

#include <CGAL/property_map.h>
#include <CGAL/value_type_traits.h>
#include <CGAL/point_set_processing_assertions.h>

#include <boost/version.hpp>
#if BOOST_VERSION >= 104000
#include <boost/property_map/property_map.hpp>
#else
#include <boost/property_map.hpp>
#endif

#include <iostream>
#include <sstream>
#include <string>

#include <vtkXMLPolyDataReader.h>
#include <vtkPolyData.h>

bool ReadVTP(std::string &InputFilename, std::back_insert_iterator<std::vector<std::pair<Kernel::Point_3, Kernel::Vector_3> > > output)
{
	typedef std::back_insert_iterator<std::vector<std::pair<Kernel::Point_3, Kernel::Vector_3> > > OutputIterator;
  // value_type_traits is a workaround as back_insert_iterator's value_type is void
	typedef typename value_type_traits<OutputIterator>::type Enriched_point;

	typedef typename Kernel::Point_3 Point;
	typedef typename Kernel::Vector_3 Vector;
	PointPMap point_pmap = make_dereference_property_map(output);
	Kernel = Kernel();
	
	vtkXMLPolyDataReader* Reader = vtkXMLPolyDataReader::New();
	Reader->SetFileName(InputFilename.c_str());
	Reader->Update();
	
	vtkPolyData* Polydata = Reader->GetOutput();
			
	for(unsigned int i = 0; i < Polydata->GetNumberOfPoints(); i++)
	{
		double p[3];
		Polydata->GetPoint(i, p);
		Point point(p[0],p[1],p[2]);
		Enriched_point pwn;
		put(point_pmap,  &pwn, point);  // point_pmap[&pwn] = point
		Vector normal = CGAL::NULL_VECTOR;
		put(normal_pmap, &pwn, normal);
		*output++ = pwn;
	}


	return true;
}
