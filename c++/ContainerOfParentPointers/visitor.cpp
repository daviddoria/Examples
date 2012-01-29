#include <iostream>

class Visitor
{
public:
  template <typename TImage>
  void Visit(TImage* image)
  {
    Output(image);
  }
};

class ImageBase
{
public:
  virtual void Visit(Visitor& v) = 0;
};


template <typename TPixel>
class Image : public ImageBase
{
public:
  TPixel GetPixel() const
  {
    TPixel a = 3; return a;
  }
  void Visit(Visitor& v)
  {
    v.Visit(this);
  }
};

template<typename TImage>
void Output(const TImage* image)
{
  std::cout << image->GetPixel();
}

int main(int, char *[])
{
  ImageBase* image = new Image<float>;
  Visitor v;
  image->Visit(v);

  return 0;
}

