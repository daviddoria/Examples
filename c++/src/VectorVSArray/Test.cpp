#include <iostream>
#include <vul/vul_timer.h>
#include <vector>

using namespace std;

void FunctionWithDynamicArray();
void FunctionWithArray();
void FunctionWithVectorPushBack();
void FunctionWithDynamicVector();
void FunctionWithVectorSized();
void FunctionWithVectorAt();
void FunctionWithVectorSubscript();

#define ArraySize 2000000
int DynamicSize = 2000000;

int main(int argc, char* argv[])
{
  vul_timer timer;

  timer.mark();
  FunctionWithArray();
  cout << "FunctionWithArray time: " << timer.real() << " ms" << endl;

  timer.mark();
  FunctionWithDynamicArray();
  cout << "FunctionWithDynamicArray time: " << timer.real() << " ms" << endl;

  timer.mark();
  FunctionWithVectorPushBack();
  cout << "FunctionWithVectorPushBack time: " << timer.real() << " ms" << endl;

  timer.mark();
  FunctionWithVectorSized();
  cout << "FunctionWithVectorSized time: " << timer.real() << " ms" << endl;

  timer.mark();
  FunctionWithDynamicVector();
  cout << "FunctionWithDynamicVector time: " << timer.real() << " ms" << endl;

  timer.mark();
  FunctionWithVectorAt();
  cout << "FunctionWithVectorAt time: " << timer.real() << " ms" << endl;

  timer.mark();
  FunctionWithVectorSubscript();
  cout << "FunctionWithVectorSubscript time: " << timer.real() << " ms" << endl;

  return 0;
}

void FunctionWithDynamicArray()
{
  
  int test[DynamicSize];
  for(int i = 0; i < DynamicSize; i ++)
    {
      test[i] = i;
    }
  

  /*
  int a;
  cin >> a;

  int test[a];
  for(int i = 0; i <a; i ++)
    {
      test[i] = i;
    }
  */
}

void FunctionWithArray()
{
  int test[ArraySize];
  for(int i = 0; i < ArraySize; i ++)
    {
      test[i] = i;
    }
}

void FunctionWithDynamicVector()
{
  vector<int> test(DynamicSize);
  for(int i = 0; i < DynamicSize; i ++)
    {
      test[i] = i;
    }
}

void FunctionWithVectorSized()
{
  vector<int> test(ArraySize);
  for(int i = 0; i < ArraySize; i ++)
    {
      test[i] = i;
    }
}

void FunctionWithVectorPushBack()
{
  vector<int> test;
  for(int i = 0; i < ArraySize; i ++)
    {
      test.push_back(i);
    }
}

void FunctionWithVectorAt()
{
  vector<int> test;
  for(int i = 0; i < ArraySize; i ++)
    {
      test.push_back(i);
    }

  int temp;

  for(int i = 0; i < ArraySize; i ++)
    {
      temp = test.at(i);
    }

}

void FunctionWithVectorSubscript()
{
  vector<int> test;
  for(int i = 0; i < ArraySize; i ++)
    {
      test.push_back(i);
    }

  int temp;

  for(int i = 0; i < ArraySize; i ++)
    {
      temp = test[i];
    }


}
