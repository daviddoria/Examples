#ifndef HEIGHT_H
#define HEIGHT_H

class Height
{
  double H;

  public:
    Height() {}
    Height(const unsigned int value) {H = value;}
    double getH() const {return H;}
};

bool operator< (const Height &H1, const Height &H2)
{
  if(H1.getH() < H2.getH())
  {
    return true;
  }
  else
  {
    return false;
  }
}

#endif