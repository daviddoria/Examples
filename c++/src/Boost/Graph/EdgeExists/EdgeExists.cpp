#include <iostream>
#include <boost/graph/adjacency_list.hpp>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g;
  Graph::vertex_descriptor v0 = boost::add_vertex(g);
  Graph::vertex_descriptor v1 = boost::add_vertex(g);

  // Check if an edge between v0 and v1 exists. It should not at this point.
  std::cout << "Edge exists?" << boost::edge(v0, v1, g).second << std::endl; // false

  // Add an edge between v0 and v1.
  std::pair<Graph::edge_descriptor, bool> e0 = boost::add_edge(v0, v1, g);

  // Check again if an edge between v0 and v1 exists. It should now.
  std::cout << "Edge exists?" << boost::edge(v0, v1, g).second << std::endl; // true

  // A demonstration of the full return type of edge(). At this point, retrievedEdge.first
  // should be exactly equal to e0
  std::pair<Graph::edge_descriptor, bool> retrievedEdge = boost::edge(v0, v1, g);

  return 0;
}
