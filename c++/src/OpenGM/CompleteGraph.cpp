
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