#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp> // for print_vertices

// Create a struct to hold several properties
struct MyProperty
{
  int MyIntProperty;
  std::string MyStringProperty;
};

// Define the type of the graph - this specifies a bundled property for vertices
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, MyProperty> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g(2);

  boost::property_map<Graph, int MyProperty::*>::type MyIntPropertyMap = boost::get(&MyProperty::MyIntProperty, g);
  boost::put(MyIntPropertyMap, 0, 5);
  boost::put(MyIntPropertyMap, 1, 10);

  boost::property_map<Graph, std::string MyProperty::*>::type MyStringPropertyMap = boost::get(&MyProperty::MyStringProperty, g);
  boost::put(MyStringPropertyMap, 0, "TestName0");
  boost::put(MyStringPropertyMap, 1, "TestName1");

  typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
  IndexMap index = get(boost::vertex_index, g);

  typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
  std::pair<vertex_iter, vertex_iter> vertexPair;
  for (vertexPair = vertices(g); vertexPair.first != vertexPair.second; ++vertexPair.first)
    {
    std::cout << index[*vertexPair.first] <<  " : " << MyIntPropertyMap[*vertexPair.first] << " : " << MyStringPropertyMap[*vertexPair.first] <<  std::endl;
    }

  std::cout << "Vertices\n";
  boost::print_vertices(g, MyIntPropertyMap);

  return 0;
}
