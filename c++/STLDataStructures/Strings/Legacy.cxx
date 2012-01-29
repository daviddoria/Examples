
  if(argc != 2)
{
  std::cout << "Required parameters: InputFilename" << std::endl;
  exit(-1);
}

  std::string InputFilename = argv[1];
    
  std::ifstream in(InputFilename.c_str());
      
  if (!in) 
{
  std::cout << "File " << InputFilename << " not found" << std::endl;
  exit(-1);
}
  
  std::string line;
  while(getline(in, line))
{
  std::stringstream ss;
  ss << line;
  std::cout << line << " length: " << line.size();
  if(line.size() > 0)
  {
    std::cout << " first char: " << line[0];
    if(line[0] == '#')
      std::cout << " comment!" << std::endl;
    else
      std::cout << std::endl;
  }
  else
    std::cout << std::endl;
    
    //std::string str = ss.str();

}
