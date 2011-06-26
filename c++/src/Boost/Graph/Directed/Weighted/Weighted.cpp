#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/directed_graph.hpp>

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;

typedef boost::directed_graph<boost::no_property, EdgeWeightProperty> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g;
  
  // Add vertices to the graph
  Graph::vertex_descriptor v0 = g.add_vertex();
  Graph::vertex_descriptor v1 = g.add_vertex();
  Graph::vertex_descriptor v2 = g.add_vertex();

  // Create weighted edges
  EdgeWeightProperty weight0 = 5;
  boost::add_edge(v0,v1,weight0,g);
  
  EdgeWeightProperty weight1 = 6;
  boost::add_edge(v1,v2,weight1,g);

  // Retrieve the weight of an edge (method 1, works)
  {
  typedef boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap;
  EdgeWeightMap edgeWeightMap = get(boost::edge_weight, g);

  std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(v0, v1, g);
  Graph::edge_descriptor edge = edgePair.first;

  std::cout << "Edge (" << v0 << ", " << v1 << ") has weight " << edgeWeightMap[edge] << std::endl;
  }

  // method 2
  {
  std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(v0, v1, g);
  Graph::edge_descriptor edge = edgePair.first;
  std::cout << std::cout << "Edge (" << v0 << ", " << v1 << ") has weight " << boost::get( boost::edge_weight, g, edge ) << std::endl;
  }

  return 0;
}
