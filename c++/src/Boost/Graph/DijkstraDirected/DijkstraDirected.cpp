#include <boost/config.hpp>
#include <iostream>
#include <fstream>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>

int main(int, char *[])
{
  typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
  
  typedef boost::adjacency_list < boost::listS, boost::vecS, boost::directedS,
    boost::no_property, EdgeWeightProperty > Graph;
  
  typedef boost::graph_traits < Graph >::vertex_descriptor vertex_descriptor;
  typedef boost::graph_traits < Graph >::edge_descriptor edge_descriptor;
  typedef std::pair<int, int> Edge;

  // Create a graph
  Graph g;
  
  Graph::vertex_descriptor v0 = boost::add_vertex(g);
  Graph::vertex_descriptor v1 = boost::add_vertex(g);
  Graph::vertex_descriptor v2 = boost::add_vertex(g);

  // Add weighted edges
  EdgeWeightProperty weight0(5);
  boost::add_edge(v0, v1, weight0, g);

  EdgeWeightProperty weight1 = 3;
  boost::add_edge(v1, v2, weight1, g);
  
  EdgeWeightProperty weight2 = 2;
  boost::add_edge(v2, v0, weight2, g);
  
  // At this point the graph is
  /*   v0
       .
   5  / \ 2
     /___\
    v1 3 v2
  */

  // Create things for Dijkstra
  std::vector<vertex_descriptor> parents(boost::num_vertices(g)); // To store parents
  std::vector<int> distances(boost::num_vertices(g)); // To store distances

  // Compute shortest paths from v0 to all vertices, and store the output in parents and distances
  boost::dijkstra_shortest_paths(g, v0, boost::predecessor_map(&parents[0]).distance_map(&distances[0]));

  // Output results
  std::cout << "distances and parents:" << std::endl;
  boost::graph_traits < Graph >::vertex_iterator vertexIterator, vend;
  for (boost::tie(vertexIterator, vend) = boost::vertices(g); vertexIterator != vend; ++vertexIterator) 
  {
    std::cout << "distance(" << *vertexIterator << ") = " << distances[*vertexIterator] << ", ";
    std::cout << "parent(" << *vertexIterator << ") = " << parents[*vertexIterator] << std::endl;
  }
  std::cout << std::endl;
  
  /*
  The output is:
  distance(0) = 0, parent(0) = 0
  distance(1) = 5, parent(1) = 0
  distance(2) = 8, parent(2) = 1
  
  which means:
  the distance from v0 to v0 is 0
  the distance from v0 to v1 is 5
  the distance from v0 to v2 is 8
  */
  return EXIT_SUCCESS;
}
