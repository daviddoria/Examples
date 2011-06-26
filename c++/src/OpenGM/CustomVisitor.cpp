#include <opengm/explicitfactor.hxx>
#include <opengm/graphicalmodel.hxx>
#include <opengm/adder.hxx>
#include <opengm/inference/beliefpropagation.hxx>
#include <opengm/maxdistance.hxx>

typedef double Energy;
typedef opengm::DiscreteSpace Space;
typedef opengm::ExplicitFactor<Energy> Factor;
typedef opengm::GraphicalModel<Factor, opengm::Adder> GraphicalModel;

double unary(int node, int label);
double binary(int node1, int node2, int label1, int label2);

template<class BP>
class BeliefPropagationProtocolVisitor {
public:
  typedef BP bp_type;
  typedef typename bp_type::value_type value_type;

  BeliefPropagationProtocolVisitor() : iteration(0){}

  void operator()(const bp_type& bp)
  {
    std::vector<size_t> state;
    bp.arg(state);

    value_type value = bp.graphicalModel().evaluate(state);

    value_type distance = bp.convergence();

    std::cout << "iteration " << iteration
        << ": energy=" <<  value
        << ", distance=" << distance
        << std::endl;
    iteration++;
  }

private:
  unsigned int iteration;
  std::vector<value_type> values_;
  std::vector<value_type> distances_;

};

int main()
{
  unsigned int numberOfNodes = 3;
  unsigned int numberOfLabels = 2;

  // Create the graphical model
  std::vector<size_t> nodes(numberOfNodes, numberOfLabels);
  Space space(nodes.begin(), nodes.end());
  GraphicalModel gm;

  // Create all unary terms
  for(unsigned int node = 0; node < numberOfNodes; node++)
    {
    std::vector<size_t> unaryIndices(1, node);
    Factor unaryFactor(space, unaryIndices.begin(), unaryIndices.end());
    for(unsigned int label = 0; label < numberOfLabels; label++)
      {
      unaryFactor(label) = unary(node, label);
      //std::cout << "The cost of assigning node " << node << " = " << label << " is " << unaryFactor(label) << std::endl;
      }
    gm.addFactor(unaryFactor);
    }

  // Create all binary terms - a 'complete' graph (every node is connected to every other node)

  for(unsigned int nodeB = 0; nodeB < numberOfNodes; nodeB++)
    {
    for(unsigned int nodeA = 0; nodeA < nodeB; nodeA++)
      {
      if(nodeA == nodeB)
        {
        continue;
        }
      std::vector<size_t> binaryIndices(2);
      binaryIndices[0] = nodeA;
      binaryIndices[1] = nodeB;
      Factor binaryFactor(space, binaryIndices.begin(), binaryIndices.end()); // Note: the nodes in a greater than unary term must appear in increasing order
      for(unsigned int labelA = 0; labelA < numberOfLabels; labelA++)
        {
        for(unsigned int labelB = 0; labelB < numberOfLabels; labelB++)
          {
          binaryFactor(labelA,labelB) = binary(nodeA, nodeB, labelA, labelB);
          //std::cout << "The cost of assigning node " << nodeA << " = " << labelA << " and node " << nodeB << " = " << labelB << " is " << binaryFactor(labelA, labelB) << std::endl;
          }
        }
      gm.addFactor(binaryFactor);
      }
    }

  ////////////// Optimize ///////////////
  typedef opengm::BeliefPropagation<GraphicalModel, opengm::Minimizer, opengm::MaxDistance> BP;
  std::cout << "setting up tree-reweighted belief propagation... " << std::endl;
  BP::Parameter para;
  para.maximumNumberOfSteps_ = 100;

  BP bp(gm, para);
  //trbp.infer();
  //BeliefPropagationProtocolVisitor<BP> visitor(clp.protocolStates);
  BeliefPropagationProtocolVisitor<BP> visitor;
  bp.infer(visitor);

  std::vector<size_t> result;
  bp.arg(result);

  for(unsigned int i = 0; i < result.size(); i++)
    {
    //std::cout << result[i] << " " ;
    }

  return 0;
}

double unary(int node, int label)
{
  if(label == 1)
    return 0.0;
  else
    return 1.0;
}

double binary(int node1, int node2, int label1, int label2)
{
  double difference = static_cast<double>(label2) - static_cast<double>(label1);
  return difference*difference;
}
