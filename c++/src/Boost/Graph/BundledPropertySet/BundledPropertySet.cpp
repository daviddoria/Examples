#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp> // for print_vertices

// Create a struct to hold several properties
struct MyProperty
{
  int MyIntProperty;
};

// Define the type of the graph - this specifies a bundled property for vertices
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, MyProperty, MyProperty> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g;
  
  // Add vertices to the graph
  Graph::vertex_descriptor v0 = boost::add_vertex(g);
  Graph::vertex_descriptor v1 = boost::add_vertex(g);
  Graph::vertex_descriptor v2 = boost::add_vertex(g);

  // Create weighted edges
  std::pair<Graph::edge_descriptor, bool> e01 = boost::add_edge(v0,v1,g);
  boost::add_edge(v1,v2,g);

  
  //boost::put(boost::get(&MyProperty::MyIntProperty, g), e01.first, 5);
  
  //boost::property_map<Graph, int MyProperty::*>::type MyIntPropertyMap = boost::get(&MyProperty::MyIntProperty, g);
  //boost::put(MyIntPropertyMap, 0, 5);
  //boost::put(MyIntPropertyMap, e01.first, 5);
  //std::cout << MyIntPropertyMap[0];
  
  
  g[e01.first].MyIntProperty = 3;
  std::cout << g[e01.first].MyIntProperty;
  
  return 0;
}
