#ifndef __vtkTestFilter_h
#define __vtkTestFilter_h

//#include "vtkGraphAlgorithm.h"
#include "vtkAlgorithm.h"

/*
class vtkTestFilter : public vtkGraphAlgorithm 
{
public:
  vtkTypeRevisionMacro(vtkTestFilter,vtkGraphAlgorithm);
*/

class vtkTestFilter : public vtkAlgorithm
{
public:
  vtkTypeRevisionMacro(vtkTestFilter,vtkAlgorithm);

  static vtkTestFilter *New();

  //vtkSetStringMacro(OutputType);
  void SetOutputType(std::string outputType){this->OutputType = outputType;}
  
protected:
  vtkTestFilter();
  ~vtkTestFilter(){}
  int FillInputPortInformation(int port, vtkInformation *info);

  int FillOutputPortInformation( int port, vtkInformation* info );
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);
  //int ProcessRequest(vtkInformation* request,
    //                                        vtkInformationVector** inInfoVec,
      //                                      vtkInformationVector* outInfoVec);
  
  std::string OutputType;
  
private:
  vtkTestFilter(const vtkTestFilter&);  // Not implemented.
  void operator=(const vtkTestFilter&);  // Not implemented.

};

#endif
