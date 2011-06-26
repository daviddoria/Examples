#include "itkSampleToHistogramFilter.h"
#include "itkListSample.h"
#include "itkHistogram.h"

typedef itk::Vector< unsigned char, 3 > MeasurementVectorType ;
typedef itk::Statistics::ListSample< MeasurementVectorType > SampleType ;

typedef itk::Statistics::Histogram< float,
        itk::Statistics::DenseFrequencyContainer2 > HistogramType;
  
void CreateSample(SampleType::Pointer image);

int main(int, char *[])
{
  SampleType::Pointer sample = SampleType::New();
  CreateSample(sample);

  typedef itk::Statistics::SampleToHistogramFilter<SampleType, HistogramType> SampleToHistogramFilterType;
  SampleToHistogramFilterType::Pointer sampleToHistogramFilter =
    SampleToHistogramFilterType::New();
  sampleToHistogramFilter->SetInput(sample);

  SampleToHistogramFilterType::HistogramSizeType histogramSize(3);
  histogramSize.Fill(10);
  sampleToHistogramFilter->SetHistogramSize(histogramSize);
  
  sampleToHistogramFilter->Update();

  const HistogramType* histogram = sampleToHistogramFilter->GetOutput();

  // Figure out which bin a measurement falls in (GetIndex) then figure out the frequency of that bin (GetFrequency)
  HistogramType::MeasurementVectorType mv(3);
  mv[0] = sample->GetMeasurementVector(0)[0];
  mv[1] = sample->GetMeasurementVector(0)[1];
  mv[2] = sample->GetMeasurementVector(0)[2];
  std::cout << "Frequency = " << histogram->GetFrequency(histogram->GetIndex(mv)) << std::endl;

  mv[0] = sample->GetMeasurementVector(1)[0];
  mv[1] = sample->GetMeasurementVector(1)[1];
  mv[2] = sample->GetMeasurementVector(1)[2];
  std::cout << "Frequency = " << histogram->GetFrequency(histogram->GetIndex(mv)) << std::endl;

  mv[0] = sample->GetMeasurementVector(2)[0];
  mv[1] = sample->GetMeasurementVector(2)[1];
  mv[2] = sample->GetMeasurementVector(2)[2];
  std::cout << "Frequency = " << histogram->GetFrequency(histogram->GetIndex(mv)) << std::endl;
  
  return EXIT_SUCCESS;
}

void CreateSample(SampleType::Pointer sample)
{
  MeasurementVectorType mv;
  mv[0] = 1;
  mv[1] = 2;
  mv[2] = 4;
  sample->PushBack(mv);

  mv[0] = 1;
  mv[1] = 2;
  mv[2] = 4;
  sample->PushBack(mv);

  mv[0] = 2;
  mv[1] = 8;
  mv[2] = 6;
  sample->PushBack(mv);
}