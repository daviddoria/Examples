#include "itkListSample.h"
#include "itkMembershipSample.h"
#include "itkVector.h"

int main()
{
  typedef itk::Vector< float, 3 > MeasurementVectorType;
  typedef itk::Statistics::ListSample< MeasurementVectorType > SampleType;
  SampleType::Pointer sample = SampleType::New();
  MeasurementVectorType mv;

  mv[0] = 1.0;
  mv[1] = 2.0;
  mv[2] = 4.0;
  sample->PushBack(mv);

  mv[0] = 2.0;
  mv[1] = 4.0;
  mv[2] = 5.0;
  sample->PushBack(mv);

  mv[0] = 3.0;
  mv[1] = 8.0;
  mv[2] = 6.0;
  sample->PushBack(mv);
  typedef itk::Statistics::MembershipSample< SampleType >
    MembershipSampleType;

  MembershipSampleType::Pointer membershipSample =
    MembershipSampleType::New();

  membershipSample->SetSample(sample);
  membershipSample->SetNumberOfClasses(2);

  membershipSample->AddInstance(0U, 0UL );
  membershipSample->AddInstance(0U, 1UL );
  membershipSample->AddInstance(1U, 2UL );

  MembershipSampleType::ConstIterator iter = membershipSample->Begin();
  while ( iter != membershipSample->End() )
    {
    std::cout << "instance identifier = " << iter.GetInstanceIdentifier()
              << "\t measurement vector = "
              << iter.GetMeasurementVector()
              << "\t frequency = "
              << iter.GetFrequency()
              << "\t class label = "
              << iter.GetClassLabel()
              << std::endl;
    ++iter;
    }


  MembershipSampleType::ClassSampleType::ConstPointer classSample =
                                  membershipSample->GetClassSample( 0 );

  MembershipSampleType::ClassSampleType::ConstIterator c_iter =
                                                    classSample->Begin();

  while ( c_iter != classSample->End() )
    {
    std::cout << "instance identifier = " << c_iter.GetInstanceIdentifier()
              << "\t measurement vector = "
              << c_iter.GetMeasurementVector()
              << "\t frequency = "
              << c_iter.GetFrequency() << std::endl;
    ++c_iter;
    }

  return EXIT_SUCCESS;
}