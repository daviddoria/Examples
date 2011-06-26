#include <boost/config.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/properties.hpp>

#include <boost/property_map/property_map.hpp>

#include <iostream>
#include <utility>
#include <vector>

typedef int Weight;
typedef boost::property<boost::edge_weight_t, Weight> WeightProperty;
typedef boost::property<boost::vertex_name_t, std::string> NameProperty;

typedef boost::adjacency_list < boost::listS, boost::vecS, boost::directedS,
  NameProperty, WeightProperty > Graph;

typedef boost::graph_traits < Graph >::vertex_descriptor Vertex;

typedef boost::property_map < Graph, boost::vertex_index_t >::type IndexMap;
typedef boost::property_map < Graph, boost::vertex_name_t >::type NameMap;

typedef boost::iterator_property_map < Vertex*, IndexMap, Vertex, Vertex& > PredecessorMap;
typedef boost::iterator_property_map < Weight*, IndexMap, Weight, Weight& > DistanceMap;

struct PathType
{
  Vertex Source;
  Vertex Desintation;
  float Distance;
};

int main(int, char *[])
{


  // Create a graph
  Graph g;

  // Add named vertices
  Vertex v0 = boost::add_vertex(std::string("v0"), g);
  Vertex v1 = boost::add_vertex(std::string("v1"), g);
  Vertex v2 = boost::add_vertex(std::string("v2"), g);
  Vertex v3 = boost::add_vertex(std::string("v3"), g);

  // Add weighted edges
  Weight weight0 = 5;
  Weight weight1 = 3;
  Weight weight2 = 2;
  Weight weight3 = 4;

  boost::add_edge(v0, v1, weight0, g);
  boost::add_edge(v1, v3, weight1, g);
  boost::add_edge(v0, v2, weight2, g);
  boost::add_edge(v2, v3, weight3, g);

  // At this point the graph is
  /*    v0
         .
        / \ 2
       /   \
      /     . v2
    5/       \
    /         \ 4
   /           \
  v1----------- v3
      3
  */

  // Create things for Dijkstra
  std::vector<Vertex> predecessors(boost::num_vertices(g)); // To store parents
  std::vector<Weight> distances(boost::num_vertices(g)); // To store distances

  IndexMap indexMap = boost::get(boost::vertex_index, g);
  PredecessorMap predecessorMap(&predecessors[0], indexMap);
  DistanceMap distanceMap(&distances[0], indexMap);

  // Compute shortest paths from v0 to all vertices, and store the output in predecessors and distances
  boost::dijkstra_shortest_paths(g, v0, boost::predecessor_map(predecessorMap).distance_map(distanceMap));

  // Output results
  std::cout << "distances and parents:" << std::endl;
  NameMap nameMap = boost::get(boost::vertex_name, g);

  BGL_FORALL_VERTICES(v, g, Graph)
  {
    std::cout << "distance(" << nameMap[v0] << ", " << nameMap[v] << ") = " << distanceMap[v] << ", ";
    std::cout << "predecessor(" << nameMap[v] << ") = " << nameMap[predecessorMap[v]] << std::endl;
  }

  // Extract a shortest path
  std::cout << std::endl;

  std::vector<PathType> path;

  Vertex v = v3; // We want to start at the destination and work our way back to the source
  for(Vertex u = predecessorMap[v]; // Start by setting 'u' to the destintaion node's predecessor
      u != v; // Keep tracking the path until we get to the source
      v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
  {
    PathType step;
    step.Source = u;
    step.Desintation = v;
    step.Distance = distanceMap[v];
    path.push_back( step );
  }

  // Write shortest path
  std::cout << "Shortest path from v0 to v3:" << std::endl;
  float totalDistance = 0;
  for(std::vector<PathType>::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
  {
    std::cout << nameMap[pathIterator->Source] << " -> " << nameMap[pathIterator->Desintation]
              << " = " << pathIterator->Distance << std::endl;
    totalDistance += pathIterator->Distance;
  }

  std::cout << std::endl;

  std::cout << "Total distance: " << totalDistance << std::endl;

  return EXIT_SUCCESS;
}

/*
 * Output:
distances and parents:
distance(v0, v0) = 0, predecessor(v0) = v0
distance(v0, v1) = 5, predecessor(v1) = v0
distance(v0, v2) = 2, predecessor(v2) = v0
distance(v0, v3) = 6, predecessor(v3) = v2

Shortest path from v0 to v3:
v0 -> v2 = 2
v2 -> v3 = 6

Total distance: 8

*/
