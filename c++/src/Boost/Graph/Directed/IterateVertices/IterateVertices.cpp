#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/directed_graph.hpp>

typedef boost::directed_graph<boost::no_property> Graph;

int main(int,char*[])
{
  // Construct a graph
  Graph g;
  
  // Add vertices to the graph
  Graph::vertex_descriptor v0 = g.add_vertex();
  Graph::vertex_descriptor v1 = g.add_vertex();
  Graph::vertex_descriptor v2 = g.add_vertex();
  
  boost::add_edge(v0, v1, g);
  boost::add_edge(v1, v2, g);

  // Get the vertices
  typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;

  // "Ask a question" of a graph. The IndexMap is the solution set.
  IndexMap index = get(boost::vertex_index, g);

  std::cout << "vertices = ";
  typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
  std::pair<vertex_iter, vertex_iter> vertexPair;
  // The call to vertices(g) sets vp to (0, numberOfElements)
  // The loop, therefore, starts at 0, and increments the 'first' of the vertexPair
  // until it equals the 'second', which again is the numberOfElements.
  for (vertexPair = boost::vertices(g); vertexPair.first != vertexPair.second; ++vertexPair.first)
  {
    // You can see what is going on with this:
    //std::cout << "first: " << index[*vertexPair.first] <<  " second: " << index[*vp.second] << std::endl; // the vertex iterator

    // Or just output the vertex ids
    std::cout << index[*vertexPair.first] <<  " ";
  }
  std::cout << std::endl;

  return 0;
}
