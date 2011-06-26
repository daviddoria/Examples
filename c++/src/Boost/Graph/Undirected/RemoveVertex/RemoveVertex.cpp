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

  std::cout << "There are " << g.num_vertices() << " vertices." << std::endl;

  g.remove_vertex(v0);

  std::cout << "There are " << g.num_vertices() << " vertices." << std::endl;
  
  return 0;
}
