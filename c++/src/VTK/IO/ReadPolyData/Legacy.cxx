  //ensure a filename was specified
  vtkstd::string inputFilename;
      
  cout << "VTK_DATA_ROOT is " << VTK_DATA_ROOT << endl;
#ifdef VTK_DATA_ROOT
  inputFilename = vtkstd::string(VTK_DATA_ROOT) + "/Data/political.vtp";
  cout << "Using file " << inputFilename << " from VTKData." << endl;
#endif
    
  if (inputFilename.empty())
{
  if(argc == 2)
  {
      //get the filename from the command line
    inputFilename = argv[1];
    cout << "Using file " << inputFilename << " from command line." << endl;
  }
  if(argc != 2)
  {
    cout << "Required arguments: InputFilename" << endl;
    return EXIT_FAILURE;
  }
}
  