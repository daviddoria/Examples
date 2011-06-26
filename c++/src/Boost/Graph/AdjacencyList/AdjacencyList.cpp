// adjacency_list is the superclass of both directed_graph and undirected_graph.
// You can use it directly if you need to specify properties more specifically than usual.

#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;

/*
// The parameters of adjacency_list are:

adjacency_list<OutEdgeList, VertexList, Directed,
               VertexProperties, EdgeProperties,
               GraphProperties, EdgeList>
*/
// The 'S' stands for "Selector", meaning you are choosing/selecting/specifying the internal storage container.
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty> Graph;

int main(int,char*[])
{
  // Create a graph
  {
  Graph g(3);
  
  // Add weighted edges. Can use this syntax:
  EdgeWeightProperty weight0(5);
  boost::add_edge(0, 1, weight0, g);
  }
  
  // Create a graph
  Graph g;

  Graph::vertex_descriptor v0 = boost::add_vertex(g);
  Graph::vertex_descriptor v1 = boost::add_vertex(g);
  
  // Add weighted edges
  // Can use this syntax:
  EdgeWeightProperty weight0(5);
  // Or this syntax:
  EdgeWeightProperty weight1 = 3;
  
  // Add an edge.
  // Can use this syntax:
  boost::add_edge(v0, v1, weight0, g);
  // Or this syntax:  
  boost::add_edge(1, 2, weight1, g);

  // Prepare to iterate over edges
  boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);

  typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
  std::pair<edge_iter, edge_iter> edgePair;
  for(edgePair = boost::edges(g); edgePair.first != edgePair.second; ++edgePair.first)
  {
    // Output both vertices of the edge
    std::cout << "Edge: " << *(edgePair.first) << std::endl;
    
    // Output the vertices of the edge individually
    std::cout << "Edge: " << boost::source(*(edgePair.first), g) << " " << boost::target(*(edgePair.first), g) << std::endl;
    
    // Output the weight of the current edge
    std::cout << "Weight: " << EdgeWeightMap[*edgePair.first] << std::endl;
  }

  return 0;
}
