#include <iostream>
#include <vector>

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

int main(int, char *[])
{
  std::vector<ImageBase*> images;
  ImageBase* image = new Image<float>;
  images.push_back(image);
  ImageBase* image2 = new Image<int>;
  images.push_back(image2);

  //Output(images[0]); // no member named GetPixel

  return 0;
}

