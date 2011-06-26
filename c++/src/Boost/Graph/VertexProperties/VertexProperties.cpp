#include <iostream>
#include <boost/graph/adjacency_list.hpp>

struct VertexProperty
{
  int Id;
};

// Define the type of the graph - this specifies a bundled property for vertices
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperty> Graph;

int main(int,char*[])
{
  std::cout << "Method 1:" << std::endl;
  
  // Create a graph object
  Graph g;
  Graph::vertex_descriptor v0 = boost::add_vertex(g);
  Graph::vertex_descriptor v1 = boost::add_vertex(g);

  g[v0].Id = 21;
  g[v1].Id = 34;

  typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
  std::pair<vertex_iter, vertex_iter> vertexPair;
  for (vertexPair = vertices(g); vertexPair.first != vertexPair.second; ++vertexPair.first)
    {
    std::cout << g[*vertexPair.first].Id <<  std::endl;
    }

  std::cout << "Method 2:" << std::endl;
  
  {
  // Create a graph object
  Graph g(2);
  
  // Assign "21" as the Id of the 0th vertex, and "34" as the Id of the 1st vertex
  typedef boost::graph_traits<Graph>::vertex_iterator VItr;
  VItr vitr, vend;
  boost::tie( vitr, vend) = boost::vertices(g);
  g[*vitr].Id = 21;
  vitr++;
  g[*vitr].Id = 34;

  boost::tie( vitr, vend) = boost::vertices(g);
  for ( ; vitr != vend ; ++vitr )
    std::cout << g[*vitr].Id << "\n";
  }

  
  return 0;
}
