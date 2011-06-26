// http://www.boost.org/doc/libs/1_38_0/libs/graph/doc/write-graphviz.html

#include <iostream>
#include <string>
#include <fstream>

#include <boost/graph/undirected_graph.hpp>
#include <boost/graph/graphviz.hpp>

typedef boost::undirected_graph<boost::no_property> Graph;

// We would like to write edges as
// 1--2 [style=invis];

template <class Name>
class label_writer
{
public:
  label_writer(Name _name) : name(_name) {}
  template <class VertexOrEdge>
  void operator()(std::ostream& out, const VertexOrEdge& v) const
  {
    out << "[style=invis]"; 
  }
private:
  Name name;
};

int main(int argc, char*argv[])
{
  if(argc < 2)
  {
    std::cerr << "Required: filename.dot" << std::endl;
    return -1;
  }
  std::string filename = argv[1];
  std::ofstream fout(filename.c_str());

  Graph g;
  Graph::vertex_descriptor v0 = g.add_vertex();
  Graph::vertex_descriptor v1 = g.add_vertex();
  Graph::vertex_descriptor v2 = g.add_vertex();
  boost::add_edge(v0,v1,g);
  boost::add_edge(v1,v2,g);
  
  //boost::write_graphviz(fout, g, boost::make_label_writer(label_writer));
  //boost::write_graphviz(fout, g, label_writer);
  
  label_writer<int> mywriter(0);
  boost::write_graphviz(fout, g, mywriter);
  

  // To visualize interactively, use kgraphviewer
  
  // To save the visualization, install libgraphviz-perl (on Ubuntu 11.4) and then use:
  // neato -Tpdf output.dot > output.pdf
  // -Tpdf means "create a pdf". Even so, you have to redirect the output to the name of the pdf file you would like to create.
  return 0;
}
