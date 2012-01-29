/*
 * This example constructs a complete graph on 3 nodes.
 * It then assigns the cost of each labeling (with 2 labels to choose from).
 * There is a unary cost function 'unary' and a binary cost function 'binary'.
 * The unary function assigns a cost of zero to assigning the label '1' and a
 * cost of 1 for any other labeling.
 * The binary function assigns a cost of the squared difference between the labels.
 * This encourages adjacent nodes to have similar labels. The resulting minimum labeling
 * is (1, 1, 1).
*/

#include <opengm/explicitfactor.hxx>
#include <opengm/graphicalmodel.hxx>
#include <opengm/adder.hxx>
#include <opengm/inference/treereweightedbeliefpropagation.hxx>
#include <opengm/maxdistance.hxx>

#include <string>
#include <sstream>

typedef double Energy;
typedef opengm::DiscreteSpace Space;
typedef opengm::ExplicitFactor<Energy> Factor;
typedef opengm::GraphicalModel<Factor, opengm::Adder> GraphicalModel;

double unary(int node, int label);
double binary(int node1, int node2, int label1, int label2);
unsigned int RandomInt(const int MAX);

int main(int argc, char*argv[])
{
  if(argc != 3)
    {
    std::cerr << "Required: numberOfNodes numberOfLabels" << std::endl;
    exit(-1);
    }
  std::string strNumberOfNodes = argv[1];
  std::string strNumberOfLabels = argv[2];

  std::stringstream ssNumberOfNodes, ssNumberOfLabels;
  ssNumberOfNodes << strNumberOfNodes;
  ssNumberOfLabels << strNumberOfLabels;

  unsigned int numberOfNodes, numberOfLabels;
  ssNumberOfNodes >> numberOfNodes;
  ssNumberOfLabels>> numberOfLabels;

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

  for(unsigned int node = 1; node < numberOfNodes; node++)
    {
    // Create a link between the current node and the next node
    std::vector<size_t> binaryIndices(2);
    binaryIndices[0] = node-1;
    binaryIndices[1] = node;
    Factor binaryFactor(space, binaryIndices.begin(), binaryIndices.end());
    for(unsigned int labelA = 0; labelA < numberOfLabels; labelA++)
      {
      for(unsigned int labelB = 0; labelB < numberOfLabels; labelB++)
        {
        binaryFactor(labelA,labelB) = binary(node-1, node, labelA, labelB);
        }
      }
    gm.addFactor(binaryFactor);

    // Create links to 3 other random nodes
    for(unsigned int i = 0; i < 3; i++)
      {
      std::vector<size_t> binaryIndicesRandom(2);
      unsigned int otherNode = RandomInt(node);
      binaryIndicesRandom[0] = otherNode;
      binaryIndicesRandom[1] = node;
      Factor binaryFactor(space, binaryIndicesRandom.begin(), binaryIndicesRandom.end());
      for(unsigned int labelA = 0; labelA < numberOfLabels; labelA++)
        {
        for(unsigned int labelB = 0; labelB < numberOfLabels; labelB++)
          {
          binaryFactor(labelA,labelB) = binary(otherNode, node, labelA, labelB);
          }
        }
      gm.addFactor(binaryFactor);
      }
    }


  ////////////// Optimize ///////////////
  typedef opengm::TreeReweightedBeliefPropagation<GraphicalModel, opengm::Minimizer, opengm::MaxDistance> TRBP;
  std::cout << "setting up tree-reweighted belief propagation... " << std::endl;
  TRBP::Parameter para;
  para.maximumNumberOfSteps_ = 100;

  TRBP trbp(gm, para);
  trbp.infer();

  std::vector<size_t> result;
  trbp.arg(result);

  for(unsigned int i = 0; i < result.size(); i++)
    {
    std::cout << result[i] << " " ;
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

unsigned int RandomInt(const int MAX)
{
  if(MAX == 0)
    return 0;

  //produce an int from 0 to MAX-1
  return rand() % MAX;
}