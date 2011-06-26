#ifndef __itkImageFilterMultipleInputs_h
#define __itkImageFilterMultipleInputs_h

#include "itkImageToImageFilter.h"

namespace itk
{
template< class TImage>
class ImageFilterMultipleInputs : public ImageToImageFilter< TImage, TImage >
{
public:
  /** Standard class typedefs. */
  typedef ImageFilterMultipleInputs             Self;
  typedef ImageToImageFilter< TImage, TImage > Superclass;
  typedef SmartPointer< Self >        Pointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

  /** Run-time type information (and related methods). */
  itkTypeMacro(ImageFilterMultipleInputs, ImageToImageFilter);

  /** The image to be inpainted in regions where the mask is white.*/
  void SetInputImage(const TImage* image);

  /** The mask to be inpainted. White pixels will be inpainted, black pixels will be passed through to the output.*/
  void SetInputMask(const TImage* mask);
  
protected:
  ImageFilterMultipleInputs();
  ~ImageFilterMultipleInputs(){}

  /** Does the real work. */
  virtual void GenerateData();

private:
  ImageFilterMultipleInputs(const Self &); //purposely not implemented
  void operator=(const Self &);  //purposely not implemented

};
} //namespace ITK


#ifndef ITK_MANUAL_INSTANTIATION
#include "ImageFilterMultipleInputs.txx"
#endif


#endif // __itkImageFilterMultipleInputs_h