#include <iostream>

class Item { ... };
class Visitor {
public:
        void Visit(Item &item) = 0;
};
class Collection : public baseCollection {
        ...
public:
        void Visit(Visitor &visitor) {
                for each item in the collection
                        visitor.Visit(item)
        }
};


int SomeClass::Sum(Collection &c) {
        class Accumulator : public Visitor {
        public:
                int sum = 0;
                Sum() : sum(0) {}
                void Visit(Item &item) {
                        sum += item.value();
                }
        } accumulator;
        c.Visit(accumulator);
        return accumulator.sum
}

int main(int argc, char *argv[])
{

  return 0;
}
