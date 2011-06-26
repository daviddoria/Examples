#ifndef __MyReader_h
#define __MyReader_h

#include "vtkPolyDataAlgorithm.h"

class MyReader : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(MyReader,vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static MyReader *New();

  // Description:
  // Specify file name of the .abc file.
  vtkSetStringMacro(FileName);
  vtkGetStringMacro(FileName);
  
protected:
  MyReader();
  ~MyReader(){}

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  MyReader(const MyReader&);  // Not implemented.
  void operator=(const MyReader&);  // Not implemented.

  char* FileName;
};

#endif
