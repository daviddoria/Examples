#include <iostream>
#include <string>

#include <boost/graph/undirected_graph.hpp>

typedef boost::property<boost::vertex_name_t, std::string> VertexProperty;
typedef boost::undirected_graph<VertexProperty> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g;

  Graph::vertex_descriptor v0 = g.add_vertex();
  std::cout << "There are " << boost::num_vertices(g) << " vertices." << std::endl;
  return 0;
}
