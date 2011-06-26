#include <boost/config.hpp>
#include <iostream>
#include <fstream>

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;

typedef boost::adjacency_list < boost::listS, boost::vecS, boost::directedS,
  boost::no_property, EdgeWeightProperty > Graph;

typedef Graph::vertex_descriptor vertex_descriptor;

std::vector<unsigned int> GetShortestPath(Graph& g, vertex_descriptor start, vertex_descriptor end);
std::vector<unsigned int> ReverseVector(std::vector<unsigned int> &v);

int main(int, char *[])
{
  // Create a graph
  Graph g;
  
  vertex_descriptor v0 = boost::add_vertex(g);
  vertex_descriptor v1 = boost::add_vertex(g);
  vertex_descriptor v2 = boost::add_vertex(g);
  vertex_descriptor v3 = boost::add_vertex(g);

  // Add weighted edges
  EdgeWeightProperty weight0(5);
  boost::add_edge(v0, v1, weight0, g);

  EdgeWeightProperty weight1(3);
  boost::add_edge(v1, v3, weight1, g);
  
  EdgeWeightProperty weight2(2);
  boost::add_edge(v0, v2, weight2, g);
  
  EdgeWeightProperty weight3(4);
  boost::add_edge(v2, v3, weight3, g);
  
  // At this point the graph is
  /*   v0
       .
      / \ 2
     /   \
    /     . v2
  5/      \
  /        \ 4
 /          \
v1---------- v3
      3
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
  distance(2) = 2, parent(2) = 0
  distance(3) = 6, parent(3) = 2
  
  which means:
  the distance from v0 to v0 is 0 and it is reached directly
  the distance from v0 to v1 is 5 and it is reached directly
  the distance from v0 to v2 is 2 and it is reached directly
  the distance from v0 to v3 is 6 and it is reached via v2
  */
  
  std::vector<unsigned int> shortestPath = GetShortestPath(g, v0, v3);
  
  for(unsigned int i = 0; i < shortestPath.size(); ++i)
  {
    std::cout << shortestPath[i] << " ";
  }
  std::cout << std::endl;
  
  return EXIT_SUCCESS;
}

std::vector<unsigned int> GetShortestPath(Graph& g, Graph::vertex_descriptor start, Graph::vertex_descriptor end)
{
  // Create things for Dijkstra
  std::vector<vertex_descriptor> parents(boost::num_vertices(g)); // To store parents
  std::vector<int> distances(boost::num_vertices(g)); // To store distances

  // Compute shortest paths from 'start' to all vertices, and store the output in parents and distances
  boost::dijkstra_shortest_paths(g, start, boost::predecessor_map(&parents[0]).distance_map(&distances[0]));

  // Create a vector in which to store the path
  std::vector<unsigned int> shortestPath;
  
  // Start at the end and work back to the beginning (aka Backtracking algorithm)
  vertex_descriptor currentVertex = end;
  
  std::cout << "Starting at " << currentVertex << " and looking for " << start << std::endl;
  
  while(parents[currentVertex] != start)
  {
    std::cout << "currentVertex: " << currentVertex << std::endl;
    std::cout << "current parent: " << parents[currentVertex] << std::endl;
    shortestPath.push_back(currentVertex);
    currentVertex = parents[currentVertex];
  }
  
  // The next to last vertex will not be added (one after 'start'), so add it manually
  shortestPath.push_back(currentVertex);
  
  // Add the 'start' vertex to the path
  shortestPath.push_back(start);
  
  return ReverseVector(shortestPath);
}

std::vector<unsigned int> ReverseVector(std::vector<unsigned int> &v)
{
  std::vector<unsigned int> result;
  
  for(int i = v.size() - 1; i >= 0; --i) // this is not unsigned because I'm not sure about the behavior of the comparison when it gets near zero
  {
    result.push_back(v[i]);
  }
  
  return result;
}