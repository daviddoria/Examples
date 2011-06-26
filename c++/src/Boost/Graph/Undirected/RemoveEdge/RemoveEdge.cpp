#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>

typedef boost::undirected_graph<boost::no_property> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g;
  
  // Add vertices
  Graph::vertex_descriptor v0 = g.add_vertex();
  Graph::vertex_descriptor v1 = g.add_vertex();
  Graph::vertex_descriptor v2 = g.add_vertex();

  // Add edges
  std::pair<Graph::edge_descriptor, bool> e0 = g.add_edge(v0, v1);
  std::pair<Graph::edge_descriptor, bool> e1 = g.add_edge(v1, v2);

  std::cout << "There are " << g.num_edges() << " edges." << std::endl;

  g.remove_edge(e0.first);

  std::cout << "There are " << g.num_edges() << " edges." << std::endl;
  
  return 0;
}
