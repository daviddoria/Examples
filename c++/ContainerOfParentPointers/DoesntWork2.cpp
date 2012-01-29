#include <iostream>
#include <vector>
#include <boost/variant.hpp>

class ImageBase { };

template <typename TPixel>
class Image : public ImageBase
{
public:
  TPixel GetPixel() const 
  {
    TPixel a = 3; return a;
  }
};

template<typename TImage>
void Output(const TImage* image)
{
  std::cout << image->GetPixel();
}

int main()
{
  std::vector<boost::variant< Image<int>*, Image<float>* > > images;

  images.push_back(new Image<float>);
  images.push_back(new Image<int>);
  Output(images[0]);
}
