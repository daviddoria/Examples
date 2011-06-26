#include <iostream>
#include <boost/graph/graph_traits.hpp>
//#include <boost/graph/directed_graph.hpp>
#include <boost/graph/adjacency_list.hpp>

//typedef boost::directed_graph<boost::no_property> Graph;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g;
  /*
  Graph::vertex_descriptor v0 = g.add_vertex();
  Graph::vertex_descriptor v1 = g.add_vertex();
  Graph::vertex_descriptor v2 = g.add_vertex();
  */

  Graph::vertex_descriptor v0 = boost::add_vertex(g);
  std::cout << v0;

  Graph::vertex_descriptor v1 = boost::add_vertex(g);
  std::cout << v1;
/*
  boost::add_edge(v0,v1,g);
  boost::add_edge(v1,v2,g);

  // Get a list of incoming edges to vertex 1
  typedef boost::graph_traits < Graph >::in_edge_iterator in_edge_iterator;

  std::pair<in_edge_iterator, in_edge_iterator> inEdges = boost::in_edges(v1, g);

  std::cout << "In edges: " << std::endl;
  for(; inEdges.first != inEdges.second; ++inEdges.first)
    {
    std::cout << *(inEdges.first) << " ";  // This outputs here is the (vertex_descriptor,vertex_descriptor). If you had properties
    }

  // Get a list of outgoing edges from vertex 1
  typedef boost::graph_traits < Graph >::out_edge_iterator out_edge_iterator;
  std::pair<out_edge_iterator, out_edge_iterator> outEdges =
    boost::out_edges(v1, g);

  std::cout << std::endl << "Out edges: " << std::endl;
  for(; outEdges.first != outEdges.second; ++outEdges.first)
    {
    std::cout << *outEdges.first << " ";
    }
*/

  std::cout << std::endl;
  /*
  // Get a list of all edges from vertex 1
  // This does not work
  typedef boost::graph_traits < Graph >::edge_iterator edge_iterator;
  std::pair<edge_iterator, edge_iterator> allEdges =
    boost::edges(v1, g);

  std::cout << std::endl << "All edges: " << std::endl;
  for(; allEdges.first != allEdges.second; ++allEdges.first)
    {
    std::cout << *allEdges.first << " ";
    }

  std::cout << std::endl;*/
  
/*
  // Method 2:
  //"Ask a question" of a graph. The IndexMap is the solution set.
  typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
  IndexMap index = get(boost::vertex_index, g);
  std::cout << "Edges from index map:";
  typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
  std::pair<edge_iter, edge_iter> edgePair;
  for(edgePair = boost::edges(g); edgePair.first != edgePair.second; ++edgePair.first)
  {
      std::cout << "(" << index[boost::source(*edgePair.first, g)]
              << "," << index[boost::target(*edgePair.first, g)] << ") ";
  }
  std::cout << std::endl;
*/
  return 0;
}
