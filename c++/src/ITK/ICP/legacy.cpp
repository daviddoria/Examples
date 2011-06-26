  // Scale the translation components of the Transform in the Optimizer
     OptimizerType::ScalesType scales( transform->GetNumberOfParameters() );

     const double translationScale = 1000.0;   // dynamic range of translations
     const double rotationScale    =    1.0;   // dynamic range of rotations

     scales[0] = 1.0 / rotationScale;
     scales[1] = 1.0 / rotationScale;
     scales[2] = 1.0 / rotationScale;
     scales[3] = 1.0 / translationScale; 
     scales[4] = 1.0 / translationScale; 
     scales[5] = 1.0 / translationScale;
  


     optimizer->SetScales( scales );