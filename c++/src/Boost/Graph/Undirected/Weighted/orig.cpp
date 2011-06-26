#include <iostream>                  // for std::cout
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

// define the type of the edge weight property (we want the weights to be doubles)
typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;

// Define the type of the graph
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, EdgeWeightProperty> Graph; //simple

int main(int,char*[])
{
  // Declare a graph object
  Graph g(2); //a graph with 2 vertices

  // Add an edge between node 0 and node 1 with weight 1.2
  EdgeWeightProperty e = 1.2;
  std::cout << e << std::endl;
  add_edge(0, 1, e, g);

  /*
  //get the 0th edge weight
  boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);

  typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
  std::pair<edge_iter, edge_iter> edgePair;
  for(edgePair = edges(g); edgePair.first != edgePair.second; ++edgePair.first)
  {
      std::cout << EdgeWeightMap[*edgePair.first] << " ";
              //<< "," << EdgeWeightMap[target(*edgePair.first, g)];// << ") weight: " << EdgeWeightMap[*edgePair.first];
  }
*/
  return 0;
}
