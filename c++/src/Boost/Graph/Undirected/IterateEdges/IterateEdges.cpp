#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;

/*
// The parameters of undirected_graph are:

undirected_graph<VertexProp, EdgeProp, GraphProp>
*/

typedef boost::undirected_graph<boost::no_property, EdgeWeightProperty> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g;
  
  // Add vertices
  Graph::vertex_descriptor v0 = g.add_vertex();
  Graph::vertex_descriptor v1 = g.add_vertex();
  Graph::vertex_descriptor v2 = g.add_vertex();

  // Add weighted edges
  EdgeWeightProperty weight0 = 5;
  g.add_edge(v0, v1, weight0);

  EdgeWeightProperty weight1 = 3;
  g.add_edge(v1, v2, weight1);

  boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);

  typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
  std::pair<edge_iter, edge_iter> edgePair;
  for(edgePair = boost::edges(g); edgePair.first != edgePair.second; ++edgePair.first)
  {
      std::cout << EdgeWeightMap[*edgePair.first] << " ";
  }

/*
  boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);

  typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
  std::pair<edge_iter, edge_iter> edgePair;
  for(edgePair = boost::edges(g); edgePair.first != edgePair.second; ++edgePair.first)
  {
      std::cout << EdgeWeightMap[*edgePair.first] << " ";
  }
  */
  return 0;
}
