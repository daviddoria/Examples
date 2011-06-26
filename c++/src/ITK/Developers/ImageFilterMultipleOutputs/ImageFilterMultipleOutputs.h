#ifndef __itkImageFilterMultipleOutputs_h
#define __itkImageFilterMultipleOutputs_h

#include "itkImageToImageFilter.h"

namespace itk
{
template< class TImage>
class ImageFilterMultipleOutputs : public ImageToImageFilter< TImage, TImage >
{
public:
  /** Standard class typedefs. */
  typedef ImageFilterMultipleOutputs             Self;
  typedef ImageToImageFilter< TImage, TImage > Superclass;
  typedef SmartPointer< Self >        Pointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

  /** Run-time type information (and related methods). */
  itkTypeMacro(ImageFilterMultipleOutputs, ImageToImageFilter);

  TImage* GetOutput1();
  TImage* GetOutput2();
  
protected:
  ImageFilterMultipleOutputs();
  ~ImageFilterMultipleOutputs(){}

  /** Does the real work. */
  virtual void GenerateData();

  /**  Create the Output */
  DataObject::Pointer MakeOutput(unsigned int idx);

private:
  ImageFilterMultipleOutputs(const Self &); //purposely not implemented
  void operator=(const Self &);  //purposely not implemented

};
} //namespace ITK


#ifndef ITK_MANUAL_INSTANTIATION
#include "ImageFilterMultipleOutputs.txx"
#endif


#endif // __itkImageFilterMultipleOutputs_h