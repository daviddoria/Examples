#ifndef __vtkTestAlgorithm_h
#define __vtkTestAlgorithm_h

#include "vtkAlgorithm.h"

class vtkDataSet;
class vtkTest;

class VTK_RENDERING_EXPORT vtkTestAlgorithm : public vtkAlgorithm
{
  public:
    static vtkTestAlgorithm *New();
    vtkTypeRevisionMacro(vtkTestAlgorithm,vtkAlgorithm);
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

  // this method is not recommended for use, but lots of old style filters use it
    vtkDataObject* GetInput();
    vtkDataObject* GetInput(int port);
    vtkTest* GetLabelHierarchyInput(int port);

  // Description:
  // Set an input of this algorithm. You should not override these
  // methods because they are not the only way to connect a pipeline.
  // Note that these methods support old-style pipeline connections.
  // When writing new code you should use the more general
  // vtkAlgorithm::SetInputConnection().  These methods transform the
  // input index to the input port index, not an index of a connection
  // within a single port.
    void SetInput( vtkDataObject* );
    void SetInput( int, vtkDataObject* );

  // Description:
  // Add an input of this algorithm.  Note that these methods support
  // old-style pipeline connections.  When writing new code you should
  // use the more general vtkAlgorithm::AddInputConnection().  See
  // SetInput() for details.
    void AddInput( vtkDataObject* );
    void AddInput( int, vtkDataObject* );

  protected:
    vtkTestAlgorithm();
    ~vtkTestAlgorithm();

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

  // see algorithm for more info
    virtual int FillOutputPortInformation( int port, vtkInformation* info );
    virtual int FillInputPortInformation( int port, vtkInformation* info );

  private:
    vtkTestAlgorithm( const vtkTestAlgorithm& ); // Not implemented.
    void operator = ( const vtkTestAlgorithm& );  // Not implemented.
};

#endif