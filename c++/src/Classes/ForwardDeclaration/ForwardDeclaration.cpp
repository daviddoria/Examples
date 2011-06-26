#include <iostream>
#include <vector>

class MessageVector;

class Node
{
  std::vector<MessageVector> messages;
};

class MessageVector
{
  Node* ToNode;
};

int main(int argc, char *argv[])
{
  Node a;
  
  return 0;
}
