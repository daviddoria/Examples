#include <iostream>
#include <vector>
#include <cmath>
#include <assert.h>

using namespace std;

vector<double> AreaWeights(const double x, const double y);
void OutputVector(const vector<double> &V);
double VectorSum(const vector<double> &V);

int main(int argc, char* argv[])
{
  //center point
  vector<double> W1 = AreaWeights(.5, .5);
  OutputVector(W1);

  //point near lower left
  vector<double> W2 = AreaWeights(.1, .1);
  OutputVector(W2);

  //point near upper left
  vector<double> W3 = AreaWeights(.1, .8);
  OutputVector(W3);

  //point near upper right
  vector<double> W4 = AreaWeights(.8, .9);
  OutputVector(W4);
  
  //point near lower right
  vector<double> W5 = AreaWeights(.9, .1);
  OutputVector(W5);
  
  return 0;
}


vector<double> AreaWeights(const double x, const double y)
{
  //these are ordered in the clockwise direction, starting at the bottom left

  assert((x >= 0.0) && (x <= 1.0) && (y >= 0.0) && (y <= 1.0));
  vector<double> Weights(4);

  Weights[0] = (1.0 - x) * (1.0 - y); //the lower left point - this is really the upper right area (because if (x,y) is close to the lower left point, it should get the most weight)
  Weights[1] = (1.0 - x) * y;//the upper left point (lower right area)
  Weights[2] = x * y; //the upper right point (lower left area)
  Weights[3] = x * (1.0 - y); //the lower right point (upper left area)

  assert(fabs(VectorSum(Weights) - 1.0) < 1e-6); //the weights should sum to 1
  return Weights;
}

void OutputVector(const vector<double> &V)
{
  for(unsigned int i = 0; i < V.size(); i++)
  {
    cout << V[i] << " ";
  }

  cout << endl;
}

double VectorSum(const vector<double> &V)
{
  double sum = 0.0;
  for(unsigned int i = 0; i < V.size(); i++)
  {
    sum += V[i];
  }

  return sum;
}