#ifndef __vtkTestFilter_h
#define __vtkTestFilter_h

#include "vtkAlgorithm.h"

class vtkDataSet;
class vtkTest;

class vtkTestFilter : public vtkAlgorithm
{
  public:
    static vtkTestFilter *New();
    vtkTypeRevisionMacro(vtkTestFilter,vtkAlgorithm);

  protected:
    vtkTestFilter();
    ~vtkTestFilter(){}

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
    virtual int FillInputPortInformation( int port, vtkInformation* info );
    
  private:
    vtkTestFilter( const vtkTestFilter& ); // Not implemented.
    void operator = ( const vtkTestFilter& );  // Not implemented.
};

#endif