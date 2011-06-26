#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_iterator.hpp>

typedef boost::property<boost::vertex_property_tag, double> VertexProperty;

/*
adjacency_list<OutEdgeList, VertexList, Directed,
               VertexProperties, EdgeProperties,
               GraphProperties, EdgeList>
*/
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperty> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g(3);

  // Create two edges
  boost::add_edge(0,1,g);
  boost::add_edge(1,2,g);

  typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
  IndexMap index = get(boost::vertex_index, g);

  typedef boost::graph_traits < Graph >::adjacency_iterator adjacency_iterator;

  std::pair<adjacency_iterator, adjacency_iterator> neighbors =
    boost::adjacent_vertices(1, g);

  for(; neighbors.first != neighbors.second; ++neighbors.first)
    {
    std::cout << index[*neighbors.first] << " ";
    }

  return 0;
}
