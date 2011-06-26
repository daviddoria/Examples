#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

//typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS> Graph; //can't use this, in_edges are not defined in this concept - it only stores out_edges for space efficiency
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g(3);

  boost::add_edge(0,1,g);
  boost::add_edge(1,2,g);

  // Get a list of incoming edges to vertex 1
  typedef boost::graph_traits < Graph >::in_edge_iterator in_edge_iterator;

  std::pair<in_edge_iterator, in_edge_iterator> inEdges = boost::in_edges(1, g);

  std::cout << "In edges: " << std::endl;
  for(; inEdges.first != inEdges.second; ++inEdges.first)
    {
    //std::cout << index[*inEdges.first] << " ";
    std::cout << *inEdges.first << " ";
    }

  std::cout << std::endl << "Out edges: " << std::endl;
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
