// http://www.boost.org/doc/libs/1_37_0/libs/graph/doc/read_graphviz.html
#include <iostream>
#include <string>
#include <fstream>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/property_maps/null_property_map.hpp>
#include <boost/property_map/property_map.hpp>

//typedef boost::property < boost::vertex_name_t, std::string> VertexProperty;
//typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS, VertexProperty> Graph;
typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS> Graph;
  
void OutputEdges(const Graph& g);

int main(int argc, char*argv[])
{
  if(argc < 2)
  {
    std::cerr << "Required: filename.dot" << std::endl;
    return -1;
  }

  Graph graph;
  boost::dynamic_properties dp;

  //boost::property_map<Graph, boost::vertex_name_t>::type name =
    //get(boost::vertex_name, graph);
  //dp.property("node_id",name);
  
  //dp.property("node_id", boost::make_null_property<Graph::vertex_descriptor, std::string>());
  
  //dp.property("node_id", boost::dummy_property_map<Graph::vertex_descriptor, std::string>()); // dummy_property_map is not a template
  //dp.property("node_id", boost::dummy_property_map()); // "error: invalid use of void expression"
  //dp.property("node_id", boost::dummy_property_map);
  //dp.property("node_id", boost::static_property_map<Graph::vertex_descriptor>()); // error: no matching function for call to ‘boost::static_property_map<unsigned int>::static_property_map()’
  
  //dp.property("node_id", boost::ref_property_map<Graph::vertex_descriptor, std::string>()); // error: no matching function for call to ‘boost::ref_property_map<unsigned int, std::basic_string<char> >::ref_property_map()’
  //dp.property("node_id", boost::ref_property_map<Graph::vertex_descriptor, std::string>);

  std::string s;
  dp.property("node_id", boost::static_property_map<std::string>(s));
  
  std::string filename = argv[1];
  std::ifstream fin(filename.c_str());

  bool status = boost::read_graphviz(fin,graph,dp,"node_id");

  std::cout << "There are " << boost::num_vertices(graph) << " vertices." << std::endl;
  
  OutputEdges(graph);
  
  return 0;
}



void OutputEdges(const Graph& g)
{
  std::pair<Graph::edge_iterator, Graph::edge_iterator> edgeIteratorRange = boost::edges(g);
  for(Graph::edge_iterator edgeIterator = edgeIteratorRange.first; edgeIterator != edgeIteratorRange.second; ++edgeIterator)
    {
      std::cout << "Edge exists between " << boost::target(*edgeIterator, g) << " and " 
                <<  boost::source(*edgeIterator, g) << std::endl;
    }  
}
