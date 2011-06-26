#ifndef __vtkAlgorithmFilter_h
#define __vtkAlgorithmFilter_h

#include "vtkAlgorithm.h"

class vtkDataSet;
class vtkTest;

class vtkAlgorithmFilter : public vtkAlgorithm
{
  public:
    static vtkAlgorithmFilter *New();
    vtkTypeRevisionMacro(vtkAlgorithmFilter,vtkAlgorithm);

    virtual int ProcessRequest(vtkInformation*,
                               vtkInformationVector**,
                               vtkInformationVector*);

  protected:
    vtkAlgorithmFilter();
    ~vtkAlgorithmFilter(){}

    virtual int RequestDataObject(
                                  vtkInformation* request,
                                  vtkInformationVector** inputVector,
                                  vtkInformationVector* outputVector );

    virtual int RequestInformation(
                                   vtkInformation* request,
                                   vtkInformationVector** inputVector,
                                   vtkInformationVector* outputVector );

    virtual int RequestData(
                            vtkInformation* request,
                            vtkInformationVector** inputVector,
                            vtkInformationVector* outputVector );

    virtual int RequestUpdateExtent(
                                    vtkInformation*,
                                    vtkInformationVector**,
                                    vtkInformationVector* );

    virtual int FillOutputPortInformation( int port, vtkInformation* info );
    virtual int FillInputPortInformation( int port, vtkInformation* info );
    
  private:
    vtkAlgorithmFilter( const vtkAlgorithmFilter& ); // Not implemented.
    void operator = ( const vtkAlgorithmFilter& );  // Not implemented.
};

#endif