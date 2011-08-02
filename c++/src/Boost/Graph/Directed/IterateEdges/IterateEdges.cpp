#include <iostream>
#include <boost/graph/adjacency_list.hpp>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g;

  Graph::vertex_descriptor v0 = boost::add_vertex(g);
  Graph::vertex_descriptor v1 = boost::add_vertex(g);

  std::pair<Graph::edge_descriptor, bool> e0 = boost::add_edge(v0, v1, g);
    
  std::pair<Graph::edge_iterator, Graph::edge_iterator> edgeIteratorRange = boost::edges(g);
  for(Graph::edge_iterator edgeIterator = edgeIteratorRange.first; edgeIterator != edgeIteratorRange.second; ++edgeIterator)
   {
     std::cout << *edgeIterator << std::endl;
   }
  return 0;
}
