

    // Turn interpolation of the image off so it's easier to see
    // pixel boundaries for this example.
  //ImageViewer->GetImageActor()->InterpolateOff();

    // Connect our picker with our interactor, set the picking tolerance to zero.
  //Picker->SetTolerance(0.0);

  //ImageViewer->GetRenderWindow()->SetSize(640, 480);
    
  // Add a mouse move obvserver which we will have tell us the pixel value
  // under the mouse whenever it moves.
  /*
  VtkObserverMouseMove* observeMouseMove = VtkObserverMouseMove::New(ImageViewer, RenderWindowInteractor, Picker);
  RenderWindowInteractor->AddObserver(vtkCommand::MouseMoveEvent, observeMouseMove);
  */
  
  
  /*
  VtkObserverMouseClick* observeMouseClick = VtkObserverMouseClick::New(ImageViewer, RenderWindowInteractor, Picker);
  //RenderWindowInteractor->AddObserver(vtkCommand::MouseClickEvent, observeMouseClick);
  RenderWindowInteractor->AddObserver(vtkCommand::LeftButtonPressEvent, observeMouseClick);
  */