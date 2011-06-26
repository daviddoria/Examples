#include "itkListSample.h"
#include "itkVector.h"

int main()
{
  typedef itk::Vector< float, 3 > MeasurementVectorType ;
  typedef itk::Statistics::ListSample< MeasurementVectorType > SampleType ;
  SampleType::Pointer sample = SampleType::New() ;

  MeasurementVectorType mv ;
  mv[0] = 1.0 ;
  mv[1] = 2.0 ;
  mv[2] = 4.0 ;

  sample->PushBack(mv) ;

  sample->Resize(3) ;

  mv[0] = 2.0 ;
  mv[1] = 4.0 ;
  mv[2] = 5.0 ;
  sample->SetMeasurementVector(1, mv) ;

  mv[0] = 3.0 ;
  mv[1] = 8.0 ;
  mv[2] = 6.0 ;
  sample->SetMeasurementVector(2, mv) ;

  for ( unsigned long i = 0 ; i < sample->Size() ; ++i )
    {
    std::cout << "id = " << i
              << "\t measurement vector = "
              << sample->GetMeasurementVector(i)
              << "\t frequency = "
              << sample->GetFrequency(i)
              << std::endl ;
    }

  SampleType::Iterator iter = sample->Begin() ;

  while( iter != sample->End() )
    {
    std::cout << "id = " << iter.GetInstanceIdentifier()
              << "\t measurement vector = "
              << iter.GetMeasurementVector()
              << "\t frequency = "
              << iter.GetFrequency()
              << std::endl ;
    ++iter ;
    }

  std::cout << "Size = " << sample->Size() << std::endl ;
  std::cout << "Total frequency = "
            << sample->GetTotalFrequency() << std::endl ;

  return EXIT_SUCCESS;
}