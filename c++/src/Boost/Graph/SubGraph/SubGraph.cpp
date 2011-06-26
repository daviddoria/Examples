#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/filtered_graph.hpp>

namespace boost {
  enum vertex_component_t { vertex_component = 111 };
  BOOST_INSTALL_PROPERTY(vertex, component);
}

template <typename ComponentMap>
struct vertexComponent {

  vertexComponent() {}

  vertexComponent(ComponentMap component, int f_component) : m_component(component), m_f_component(f_component) {}

  template <typename Vertex>
  bool operator()(const Vertex& v) const {
    return (get(m_component, v) == m_f_component);
  }

  ComponentMap m_component;
  int m_f_component;
};

int main(int argc, char ** argv) {

  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                         boost::property<boost::vertex_component_t, int> > Graph;

  typedef boost::property_map<Graph, boost::vertex_component_t>::type ComponentMap;
  typedef boost::filtered_graph<Graph,
                         boost::keep_all,
                         vertexComponent<ComponentMap> > FilteredGraph;
  
  boost::graph_traits < Graph >::vertex_iterator vi, vi_end;
  boost::graph_traits < Graph >::out_edge_iterator oei, oei_end;
  boost::graph_traits < FilteredGraph >::vertex_iterator fvi, fvi_end;
  boost::graph_traits < FilteredGraph >::out_edge_iterator foei, foei_end;

  Graph * g;
  FilteredGraph * fg;

  enum { A, B, C, D, E, F, G, H, I, N };
  const char* name = "ABCDEFGHI";
  g = new Graph(N);
  add_edge(A, B, *g);
  add_edge(C, D, *g);
  add_edge(D, E, *g);
  add_edge(E, C, *g);
  add_edge(F, A, *g);
  add_edge(F, B, *g);
  add_edge(G, H, *g);
  add_edge(G, I, *g);
  add_edge(H, I, *g);

  std::cout<<std::endl<<"Graph out-edges:"<<std::endl;
  for (tie(vi, vi_end) = vertices(*g); vi != vi_end; ++vi) 
  {
    std::cout<<name[*vi]<<" outedges - ";
    for(tie(oei, oei_end)=out_edges(*vi, *g); oei != oei_end; ++oei) 
    {
      std::cout<<name[source(*oei, *g)]<<"-->"<<name[target(*oei, *g)]<<"  ";
    }
    std::cout<<std::endl;
  }
  std::cout<<std::endl;

  int numComponents = connected_components(*g, get(boost::vertex_component, *g));
					   
  std::cout<<"Graph has "<<numComponents<<" components."<<std::endl<<std::endl;

  for(int i=0; i<numComponents; i++) 
  {
    boost::keep_all efilter;
    vertexComponent<ComponentMap> vfilter(get(boost::vertex_component, *g), i);
    fg = new FilteredGraph(*g, efilter, vfilter);

    std::cout<<"Filtered graph (component "<<i<<") out-edges:"<<std::endl;
    for (tie(fvi, fvi_end) = vertices(*fg); fvi != fvi_end; ++fvi) 
    {    
      std::cout<<name[*fvi]<<" outedges - ";    
      for(tie(foei, foei_end)=out_edges(*fvi, *fg); foei != foei_end; ++foei) 
      {
	std::cout<<name[source(*foei, *fg)]<<"-->"<<name[target(*foei, *fg)]<<"  ";
      }
      std::cout<<std::endl;
    }
    std::cout<<std::endl;

    delete fg;
    fg = NULL;
  }

  delete g;
  g = NULL;

  return EXIT_SUCCESS;
}
