#ifndef __vtkTestReader_h
#define __vtkTestReader_h

#include "vtkXMLPolyDataReader.h"

class vtkTestReader : public vtkXMLPolyDataReader
{
public:
  vtkTypeRevisionMacro(vtkTestReader,vtkXMLPolyDataReader);
  static vtkTestReader *New();
  vtkTestReader();
  
protected:
  
  int FillOutputPortInformation( int port, vtkInformation* info );
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  vtkTestReader(const vtkTestReader&);  // Not implemented.
  void operator=(const vtkTestReader&);  // Not implemented.
};

#endif
