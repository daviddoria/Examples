#include <iostream>

class ImageBase{};

template <typename TPixel>
class Image : public ImageBase
{
public:
  TPixel GetPixel() const {TPixel a; return a;}
};

template <typename TPixel>
class MyImage : public Image<TPixel>
{
public:
  void Output() {this->GetPixel();}
};

template<typename TImage>
void Output(const TImage* image)
{
  std::cout << image->GetPixel();
}

int main(int, char *[])
{
  // Works correctly
  {
  Image<float>* image = new Image<float>;
  image->GetPixel();
  Output(image);
  }

  {
  ImageBase* image = new Image<float>;
  //Output(image); // Can't do this - ImageBase does not have GetPixel defined.
  }

  {
  ImageBase* image = new MyImage<float>;
  //image->Output(); // Still, ImageBase does not have an Output function defined.
  }
  return 0;
}