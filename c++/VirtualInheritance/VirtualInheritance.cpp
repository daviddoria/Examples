#include <vector>

class ImageBase 
{
  virtual void Test(){}
};

template <typename TPixel>
class Image : public ImageBase {};

struct CustomImageBase : public virtual ImageBase
{
  virtual void DoSomething() = 0;
};

template <typename TPixel>
struct CustomImage : public Image<TPixel>, public CustomImageBase
{
  void DoSomething(){}
};

int main(int, char *[])
{
  CustomImage<float>* floatImage = new CustomImage<float>;
  floatImage->DoSomething();
  CustomImage<int>* intImage = new CustomImage<int>;
  intImage->DoSomething();

  std::vector<CustomImageBase*> images;
  images.push_back(floatImage);
  images.push_back(intImage);
  for(unsigned int i = 0; i < images.size(); ++i)
  {
    images[i]->DoSomething();
  }
  return 0;
}
