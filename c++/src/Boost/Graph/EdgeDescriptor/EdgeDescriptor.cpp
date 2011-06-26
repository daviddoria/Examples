#include <iostream>
#include <boost/graph/adjacency_list.hpp>

// Define the type of the graph - this specifies a bundled property for vertices
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g;
  Graph::vertex_descriptor v0 = boost::add_vertex(g);
  Graph::vertex_descriptor v1 = boost::add_vertex(g);

  std::pair<Graph::edge_descriptor, bool> e1 = boost::add_edge(v0, v1, g);
  
  return 0;
}
