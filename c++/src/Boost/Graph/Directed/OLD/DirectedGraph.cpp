#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS> Graph; // only have access to out_edges

int main(int,char*[])
{
  // Create a graph object
  Graph g(3);

  boost::add_edge(0,1,g);
  boost::add_edge(1,2,g);

  std::cout << "Out edges: " << std::endl;
  // Get a list of outgoing edges from vertex 1
  typedef boost::graph_traits < Graph >::out_edge_iterator out_edge_iterator;
  std::pair<out_edge_iterator, out_edge_iterator> outEdges =
    boost::out_edges(1, g);

  for(; outEdges.first != outEdges.second; ++outEdges.first)
    {
    std::cout << *outEdges.first << " ";
    }

  std::cout << std::endl;

  return 0;
}
