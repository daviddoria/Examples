{
  // Why use this instead of GradientMagnitudeImageFilter ?
typedef itk::VectorGradientAnisotropicDiffusionImageFilter<VectorImageType,
							    VectorImageType>  DiffusionFilterType;
  DiffusionFilterType::Pointer diffusion = DiffusionFilterType::New();
  diffusion->SetNumberOfIterations( 10 );
  diffusion->SetConductanceParameter( 2.0 );
  diffusion->SetTimeStep(0.125);
  diffusion->SetInput(reader->GetOutput());
  diffusion->Update();

  typedef itk::VectorMagnitudeImageFilter<VectorImageType, ScalarImageType> VectorMagnitudeFilterType; 
  VectorMagnitudeFilterType::Pointer vectorMagnitudeFilter = VectorMagnitudeFilterType::New();
  vectorMagnitudeFilter->SetInput(diffusion->GetOutput());
  vectorMagnitudeFilter->GetOutput();
}