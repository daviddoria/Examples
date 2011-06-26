/*
 * This example demonstrates 3 ways to construct a graph.
 */

// STL
#include <iostream>                  // for std::cout

// Boost
#include <boost/graph/adjacency_list.hpp> // for customizable graphs
#include <boost/graph/directed_graph.hpp> // A subclass to provide reasonable arguments to adjacency_list for a typical directed graph
#include <boost/graph/undirected_graph.hpp>// A subclass to provide reasonable arguments to adjacency_list for a typical undirected graph

void AdjacencyList();
void UndirectedGraph();
void DirectedGraph();

int main(int,char*[])
{
  AdjacencyList();
  UndirectedGraph();
  DirectedGraph();

  return 0;
}

void AdjacencyList()
{
  /* Method 1: The most generic
  * The generic class for a graph in Boost is adjacency_list.
  * Up to 7 template parameters can be given, for example:
  * typedef boost::adjacency_list<     boost::listS,             // out-edges stored in a std::list
  *                       boost::vecS,             // vertex set stored here
  *                       boost::undirectedS,    // bidirectional graph.
  *                       boost::no_property,              // vertex properties
  *                       EdgeWeightProperty,       // edge properties
  *                       boost::no_property,       // graph properties
  *                       boost::listS              // edge storage
  *                       > graph_t;
  *
  * The 'S' at the end of the choices (vecS, etc.) stands for 'S'elector.
  */

  {
  // Construct a graph with the vertices container as a vector
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS> Graph;
  Graph g(3); // Create a graph with 3 vertices.

  // The graph behaves as a new user would expect if the vertex container type is vector. That is, vertices can be indexed with an unsigned int.
  boost::add_edge(0, 1, g);
  boost::add_edge(1, 2, g);
  }

  {
  // Construct a graph with the vertices container as a set
  typedef boost::adjacency_list<boost::vecS, boost::setS, boost::bidirectionalS> Graph;

  // Since the vertex container type is not a vector, the vertices can NOT be indexed with an unsigned int. I.e. the following will not work:
  //Graph g(3); // 3 vertices
  //boost::add_edge(0, 1, g);
  //boost::add_edge(1, 2, g);

  // Instead, you must add vertices individually so that you get a handle to them (a way to reference them, Boost calls this a "vertex_descriptor"),
  // and then add the edges by referencing these descriptors. Note this is a very generic method, so it would work just as well with a vecS vertex container.

  Graph g; // Create a graph.
  Graph::vertex_descriptor v0 = boost::add_vertex(g);
  Graph::vertex_descriptor v1 = boost::add_vertex(g);
  Graph::vertex_descriptor v2 = boost::add_vertex(g);
  
  boost::add_edge(v0, v1, g);
  boost::add_edge(v1, v2, g);

  }
}

void UndirectedGraph()
{
  // undirected_graph is a subclass of adjacency_list which gives you object oriented access to functions like add_vertex and add_edge, which makes the code easier to understand. However, it hard codes many of the template parameters, so it is much less flexible.

  typedef boost::undirected_graph<> Graph;
  Graph g;
  Graph::vertex_descriptor v0 = g.add_vertex();
  Graph::vertex_descriptor v1 = g.add_vertex();

  g.add_edge(v0, v1);
}

void DirectedGraph()
{
  // directed_graph is a subclass of adjacency_list which gives you object oriented access to functions like add_vertex and add_edge, which makes the code easier to understand. However, it hard codes many of the template parameters, so it is much less flexible.
  
  typedef boost::directed_graph<> Graph;
  Graph g;
  Graph::vertex_descriptor v0 = g.add_vertex();
  Graph::vertex_descriptor v1 = g.add_vertex();

  g.add_edge(v0, v1);
}
