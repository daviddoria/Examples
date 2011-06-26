#ifndef __vtkTestSource_h
#define __vtkTestSource_h

#include "vtkAlgorithm.h"

class vtkDataSet;
class vtkTest;

class vtkTestSource : public vtkAlgorithm
{
  public:
    static vtkTestSource *New();
    vtkTypeRevisionMacro(vtkTestSource,vtkAlgorithm);
    void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Get the output data object for a port on this algorithm.
    vtkTest* GetOutput();
    vtkTest* GetOutput(int);
    virtual void SetOutput(vtkDataObject* d);

  // Description:
  // see vtkAlgorithm for details
    virtual int ProcessRequest(vtkInformation*,
                               vtkInformationVector**,
                               vtkInformationVector*);

  protected:
    vtkTestSource();
    ~vtkTestSource();

  // Description:
  // This is called by the superclass.
  // This is the method you should override.
    virtual int RequestDataObject(
                                  vtkInformation* request,
                                  vtkInformationVector** inputVector,
                                  vtkInformationVector* outputVector );

  // convenience method
    virtual int RequestInformation(
                                   vtkInformation* request,
                                   vtkInformationVector** inputVector,
                                   vtkInformationVector* outputVector );

  // Description:
  // This is called by the superclass.
  // This is the method you should override.
    virtual int RequestData(
                            vtkInformation* request,
                            vtkInformationVector** inputVector,
                            vtkInformationVector* outputVector );

  // Description:
  // This is called by the superclass.
  // This is the method you should override.
    virtual int RequestUpdateExtent(
                                    vtkInformation*,
                                    vtkInformationVector**,
                                    vtkInformationVector* );

    virtual int FillOutputPortInformation( int port, vtkInformation* info );

  private:
    vtkTestSource( const vtkTestSource& ); // Not implemented.
    void operator = ( const vtkTestSource& );  // Not implemented.
};

#endif