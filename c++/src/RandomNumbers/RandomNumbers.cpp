#include <iostream>
#include <cstdlib> //drand48()
#include <cstdio>
#include <ctime>

using namespace std;

void RandomInts();
void RandomDoubles();
void RandomFloats();

int RandomInt(const int MAX);
double RandomDouble();
float RandomFloat();

int main()
{
  unsigned int t = time(NULL);
  cout << t << endl;
  srand(t); 
  
  //RandomFloats();
  RandomDoubles();
  //RandomInts();
  
  return 0;
}

void RandomInts()
{
  //srand((unsigned)time(0)); //if you dont do this, they are the same every time
  
  int random_integer; 
  int n = 10;
  int MAX = 5;
  
  for(int index = 0; index < n; index++)
  { 
    random_integer = RandomInt(MAX);
    cout << random_integer << endl; 
  } 
}

void RandomDoubles()
{
  //srand48((unsigned)time(0)); //if you dont do this, they are the same every time

  int n = 10;
  //produce n random doubles between 0 and 1

  for(int i = 0; i < n; i++)
  {
    double r = RandomDouble();
    cout << r << endl;
  }
}

void RandomFloats()
{
  //srand48((unsigned)time(0)); //if you dont do this, they are the same every time

  int n = 10;
  //produce n random doubles between 0 and 1

  for(int i = 0; i < n; i++)
  {
    float r = RandomFloat();
    cout << r << endl;
  }
}

double RandomDouble()
{
  //produce a random double between 0 and 1
  return drand48();
}

float RandomFloat()
{
  //produce a random float between 0 and 1
  return drand48();
}

int RandomInt(const int MAX)
{
  //produce an int from 0 to MAX-1
  return rand() % MAX; 
}