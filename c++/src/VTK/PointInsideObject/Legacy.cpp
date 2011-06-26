
  double pcoords[3];
  
  
  vtkIdType cellId;
    
  int subId;
  
  //should be inside
  cellId = Sphere->FindCell(TestInside, NULL, 0, 0.0, subId, pcoords, NULL);
  if (cellId>=0)
{
  cout << "In cell " << cellId << endl;
  std::cout << "inside" << std::endl;
}
  else
{
  std::cout << "outside" << std::endl;
}
      
  //should be outside
  cellId = Sphere->FindCell(TestOutside, NULL, 0, 0.0, subId, pcoords, NULL);
  if (cellId>=0)
{
  cout << "In cell " << cellId << endl;
  std::cout << "inside" << std::endl;
}
  else
{
  std::cout << "outside" << std::endl;
}