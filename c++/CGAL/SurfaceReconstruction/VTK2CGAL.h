#ifndef VTK2CGAL_H
#define VTK2CGAL_H

#include <iterator>
#include <string>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>

bool ReadVTP(std::string &InputFilename, std::back_insert_iterator<std::vector<std::pair<Kernel::Point_3, Kernel::Vector_3> > > output);

#endif

