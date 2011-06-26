  vtkSmartPointer<vtkPoints> transformedSourcePoints =
    vtkSmartPointer<vtkPoints>::New();
  landmarkTransform->TransformPoints(sourcePoints, transformedSourcePoints);

  for(unsigned int i = 0; i < 3; i++)
    {
    double origpoint[3];
    sourcePoints->GetPoint(i, origpoint);
    std::cout << "Original point: (" << origpoint[0] << ", "
              << origpoint[1] << ", " << origpoint[2] << ")" << std::endl;

    double transpoint[3];
    transformedSourcePoints->GetPoint(i, transpoint);
    std::cout << "Transformed point: (" << transpoint[0] << ", "
              << transpoint[1] << ", " << transpoint[2] << ")" << std::endl;
    }