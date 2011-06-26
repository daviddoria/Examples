#include "itkHistogram.h"

int main()
{
  typedef float MeasurementType ;
  typedef itk::Statistics::Histogram< MeasurementType,
          itk::Statistics::DenseFrequencyContainer2 > HistogramType;
  HistogramType::Pointer histogram = HistogramType::New() ;

  HistogramType::SizeType size ;
  size.Fill(3) ;
  HistogramType::MeasurementVectorType lowerBound ;
  HistogramType::MeasurementVectorType upperBound ;
  lowerBound = 1.1 ;
  upperBound = 7.1 ;

  histogram->Initialize(size, lowerBound, upperBound ) ;

  histogram->SetFrequency(0UL, 0.0) ;
  histogram->SetFrequency(1UL, 2.0) ;
  histogram->SetFrequency(2UL, 3.0) ;
  histogram->SetFrequency(3UL, 2.0) ;
  histogram->SetFrequency(4UL, 0.5) ;
  histogram->SetFrequency(5UL, 1.0) ;
  histogram->SetFrequency(6UL, 5.0) ;
  histogram->SetFrequency(7UL, 2.5) ;
  histogram->SetFrequency(8UL, 0.0) ;

  HistogramType::IndexType index ;
  index[0] = 0 ;
  index[1] = 2 ;
  std::cout << "Frequency of the bin at index  " << index
            << " is " << histogram->GetFrequency(index)
            << ", and the bin's instance identifier is "
            << histogram->GetInstanceIdentifier(index) << std::endl ;

  HistogramType::MeasurementVectorType mv ;
  mv[0] = 4.1 ;
  mv[1] = 5.6 ;
  index.Fill(1) ;

  std::cout << "Measurement vector at the center bin is "
            << histogram->GetMeasurementVector(index) << std::endl ;

  HistogramType::IndexType resultingIndex;
  histogram->GetIndex(mv,resultingIndex);
  std::cout << "Index of the measurement vector " << mv
            << " is " << resultingIndex << std::endl ;

  std::cout << "Instance identifier of index " << index
            << " is " << histogram->GetInstanceIdentifier(index)
            << std::endl ;

  index.Fill(100) ;
  if ( histogram->IsIndexOutOfBounds(index) )
    {
    std::cout << "Index " << index << "is out of bounds." << std::endl ;
    }

  std::cout << "Number of bins = " << histogram->Size()
            << " Total frequency = " << histogram->GetTotalFrequency()
            << " Dimension sizes = " << histogram->GetSize() << std::endl ;

  std::cout << "50th percentile along the first dimension = "
            << histogram->Quantile(0, 0.5) << std::endl ;

  return 0 ;
}