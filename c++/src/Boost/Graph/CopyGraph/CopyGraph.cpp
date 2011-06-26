// http://www.boost.org/doc/libs/1_37_0/libs/graph/doc/read_graphviz.html
#include <iostream>
#include <string>
#include <fstream>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

int main(int argc, char*argv[])
{
  if(argc < 2)
  {
    std::cerr << "Required: filename.dot" << std::endl;
    return -1;
  }

  typedef boost::property < boost::vertex_name_t, std::string> VertexProperty;
  typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::directedS, VertexProperty> graph_t;
  
  //typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::directedS, boost::null_property_map> graph_t;
  graph_t graph;
  boost::dynamic_properties dp;

  boost::property_map<graph_t, boost::vertex_name_t>::type name =
    get(boost::vertex_name, graph);
  dp.property("node_id",name);

  /*
  // Sample graph as an std::istream;
  std::istringstream
    gvgraph("digraph { graph [name=\"graphname\"]  a  c e [mass = 6.66] }");

  bool status = boost::read_graphviz(gvgraph,graph,dp,"node_id");
  */

  std::string filename = argv[1];
  std::ifstream fin(filename.c_str());

  bool status = boost::read_graphviz(fin,graph,dp,"node_id");

  std::cout << "There are " << boost::num_vertices(graph) << " vertices." << std::endl;
  
  return 0;
}
