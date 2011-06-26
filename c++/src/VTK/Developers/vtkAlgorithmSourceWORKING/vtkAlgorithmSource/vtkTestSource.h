#ifndef __vtkTestSource_h
#define __vtkTestSource_h

#include "vtkTestAlgorithm.h"

class vtkTestSource : public vtkTestAlgorithm
{
  public:
    static vtkTestSource* New();
    vtkTypeRevisionMacro(vtkTestSource,vtkTestAlgorithm);
    virtual void PrintSelf( ostream& os, vtkIndent indent );

  protected:
    vtkTestSource();
    virtual ~vtkTestSource();

    virtual int FillInputPortInformation( int port, vtkInformation* info );

    virtual int RequestData(
                            vtkInformation* request,
                            vtkInformationVector** inputVector,
                            vtkInformationVector* outputVector );

   
  private:
    vtkTestSource( const vtkTestSource& ); // Not implemented.
    void operator = ( const vtkTestSource& ); // Not implemented.
};

#endif