#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kolmogorov_max_flow.hpp>
#include <boost/graph/push_relabel_max_flow.hpp>
#include <boost/graph/edmunds_karp_max_flow.hpp>


using namespace boost;

typedef int EdgeWeightType;

typedef adjacency_list_traits < vecS, vecS, directedS > Traits;
typedef adjacency_list < vecS, vecS, directedS,
property < vertex_name_t, std::string,
property < vertex_index_t, long,
property < vertex_color_t, boost::default_color_type,
property < vertex_distance_t, long,
property < vertex_predecessor_t, Traits::edge_descriptor > > > > >,
  
property < edge_capacity_t, EdgeWeightType,
property < edge_residual_capacity_t, EdgeWeightType,
property < edge_reverse_t, Traits::edge_descriptor > > > > Graph;

Traits::edge_descriptor AddEdge(Traits::vertex_descriptor &v1, Traits::vertex_descriptor &v2, property_map < Graph, edge_reverse_t >::type &rev, const double capacity, Graph &g);
		  
int main(int, char*[])
{
  Graph g; //a graph with 0 vertices

  property_map < Graph, edge_reverse_t >::type rev = get(edge_reverse, g);
  
  //add a source and sink node, and store them in s and t, respectively
  Traits::vertex_descriptor v0 = add_vertex(g);
  Traits::vertex_descriptor v1 = add_vertex(g);
  Traits::vertex_descriptor v2 = add_vertex(g);
  Traits::vertex_descriptor v3 = add_vertex(g);
	  
  /*
  AddEdge(v0, v1, rev, 6.0, g);
  AddEdge(v0, v2, rev, 5.0, g);
  AddEdge(v1, v2, rev, 8.0, g);
  AddEdge(v2, v3, rev, 7.0, g);
  */
  AddEdge(v0, v1, rev, 6, g);
  AddEdge(v0, v2, rev, 5, g);
  AddEdge(v1, v3, rev, 8, g);
  AddEdge(v2, v3, rev, 7, g);
  
  //find min cut
  EdgeWeightType flow = kolmogorov_max_flow(g, v0, v3); // a list of sources will be returned in s, and a list of sinks will be returned in t
  //EdgeWeightType flow = push_relabel_max_flow(g, v0, v3); // a list of sources will be returned in s, and a list of sinks will be returned in t
  //EdgeWeightType flow = edmunds_karp_max_flow(g, v0, v3); // a list of sources will be returned in s, and a list of sinks will be returned in t
  
  std::cout << "Max flow is: " << flow << std::endl;

  property_map<Graph, edge_capacity_t>::type 
		  capacity = get(edge_capacity, g);
  property_map<Graph, edge_residual_capacity_t>::type 
		  residual_capacity = get(edge_residual_capacity, g);

  
  graph_traits<Graph>::vertex_iterator u_iter, u_end;
  graph_traits<Graph>::out_edge_iterator ei, e_end;
  for (tie(u_iter, u_end) = vertices(g); u_iter != u_end; ++u_iter)
	  for (tie(ei, e_end) = out_edges(*u_iter, g); ei != e_end; ++ei)
		  if (capacity[*ei] > 0)
			  std::cout << "Source: " << *u_iter << " destination: " << target(*ei, g) << " capacity: "  << capacity[*ei] << "residual cap: " << residual_capacity[*ei] << " used capacity: "
					  << (capacity[*ei] - residual_capacity[*ei]) << std::endl;

 return 0;
}

Traits::edge_descriptor AddEdge(Traits::vertex_descriptor &v1, Traits::vertex_descriptor &v2, property_map < Graph, edge_reverse_t >::type &rev, const double capacity, Graph &g)
{
  Traits::edge_descriptor e1 = add_edge(v1, v2, g).first;
  Traits::edge_descriptor e2 = add_edge(v2, v1, g).first;
  put(edge_capacity, g, e1, capacity);
  put(edge_capacity, g, e2, capacity);
  
  rev[e1] = e2;
  rev[e2] = e1;
}

  /*
  //find min cut
  kolmogorov_max_flow
		  (Graph& g,
		    CapacityEdgeMap cap,
    ResidualCapacityEdgeMap res_cap,
    ReverseEdgeMap rev,
    ColorMap color,
    IndexMap idx,
    typename graph_traits<Graph>::vertex_descriptor src,
    typename graph_traits<Graph>::vertex_descriptor sink)
  {
  */