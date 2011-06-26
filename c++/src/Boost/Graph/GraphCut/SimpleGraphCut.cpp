#include <iostream>                  // for std::cout
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kolmogorov_max_flow.hpp>

using namespace boost;

typedef adjacency_list_traits<vecS, vecS, bidirectionalS> Traits;

typedef property<edge_reverse_t, Traits::edge_descriptor> EdgeReverseProperty;
typedef property<edge_residual_capacity_t, long, EdgeReverseProperty> EdgeResidualCapacityProperty;
typedef property<edge_capacity_t, double, EdgeResidualCapacityProperty> EdgeCapacityProperty;

typedef adjacency_list<vecS, vecS, bidirectionalS, EdgeCapacityProperty> Graph; //simple

int main(int,char*[])
{
   // declare a graph object
	Graph g(2); //a graph with 2 vertices
	
	//add an edge between node 0 and node 1 with weight (capacity) 2.3
	EdgeCapacityProperty e = 2.3;
	add_edge(0, 1, e, g); 
	
	//find min cut

	Traits::vertex_descriptor s, t; 
	//double flow = kolmogorov_max_flow(g, s, t); // a list of sources will be returned in s, and a list of sinks will be returned in t
	EdgeCapacityProperty flow = kolmogorov_max_flow(g, s, t); // a list of sources will be returned in s, and a list of sinks will be returned in t
	std::cout << "Max flow is: " << flow << std::endl;

	return 0;
}
