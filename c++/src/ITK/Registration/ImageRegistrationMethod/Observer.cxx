//    INPUTS: {BrainProtonDensitySliceBorder20.png}
//    INPUTS: {BrainProtonDensitySliceShifted13x17y.png}
//    OUTPUTS: {ImageRegistration1Output.png}
//    OUTPUTS: {ImageRegistration1DifferenceAfter.png}
//    OUTPUTS: {ImageRegistration1DifferenceBefore.png}


class CommandIterationUpdate : public itk::Command
{
  public:
    typedef  CommandIterationUpdate   Self;
    typedef  itk::Command             Superclass;
    typedef itk::SmartPointer<Self>  Pointer;
    itkNewMacro( Self );

  protected:
    CommandIterationUpdate() {};

  public:

    typedef itk::RegularStepGradientDescentOptimizer     OptimizerType;
    typedef const OptimizerType                         *OptimizerPointer;

    void Execute(itk::Object *caller, const itk::EventObject & event)
    {
      Execute( (const itk::Object *)caller, event);
    }

    void Execute(const itk::Object * object, const itk::EventObject & event)
    {
      OptimizerPointer optimizer =
          dynamic_cast< OptimizerPointer >( object );

      if( ! itk::IterationEvent().CheckEvent( &event ) )
      {
        return;
      }

      std::cout << optimizer->GetCurrentIteration() << " = " << optimizer->GetValue() << " : " << optimizer->GetCurrentPosition() << std::endl;
    }

};