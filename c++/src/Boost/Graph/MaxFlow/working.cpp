#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kolmogorov_max_flow.hpp>

using namespace boost;

  typedef adjacency_list_traits < vecS, vecS, directedS > Traits;
  typedef adjacency_list < vecS, vecS, directedS,
  property < vertex_name_t, std::string,
  property < vertex_index_t, long,
  property < vertex_color_t, boost::default_color_type,
  property < vertex_distance_t, long,
  property < vertex_predecessor_t, Traits::edge_descriptor > > > > >,
    
  property < edge_capacity_t, double,
  property < edge_residual_capacity_t, double,
  property < edge_reverse_t, Traits::edge_descriptor > > > > Graph;
  

  int main(int,char*[])
{
	Graph g; //a graph with 0 vertices

	property_map < Graph, edge_reverse_t >::type rev = get(edge_reverse, g);
	
	//add a source and sink node, and store them in s and t, respectively
	Traits::vertex_descriptor s = add_vertex(g);
	Traits::vertex_descriptor t = add_vertex(g);
	
  //add an edge between node s and node t with weight (capacity) 2.3
	Traits::edge_descriptor e1 = add_edge(s, t, g).first;
	Traits::edge_descriptor e2 = add_edge(t, s, g).first;
	put(edge_capacity, g, e1, 4.3);
	put(edge_capacity, g, e2, 5.1);
	
	//adding these two lines makes the segmentation fault stop!
	rev[e1] = e2;
	rev[e2] = e1;
	
 //find min cut
	double flow = kolmogorov_max_flow(g, s, t); // a list of sources will be returned in s, and a list of sinks will be returned in t
	std::cout << "Max flow is: " << flow << std::endl;

	return 0;
}
