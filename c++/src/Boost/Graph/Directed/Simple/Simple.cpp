#include <boost/graph/directed_graph.hpp>

typedef boost::directed_graph<boost::no_property> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g;
  
  // Add vertices to the graph
  Graph::vertex_descriptor v0 = g.add_vertex();
  Graph::vertex_descriptor v1 = g.add_vertex();
  Graph::vertex_descriptor v2 = g.add_vertex();
  
  // Add edges to the graph
  boost::add_edge(v0,v1,g);
  boost::add_edge(v1,v2,g);

  return 0;
}
