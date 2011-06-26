#ifndef __vtkTestSource_h
#define __vtkTestSource_h

#include "vtkAlgorithm.h"
//#include "vtkImageData.h" // makes things a bit easier
#include "vtkTest.h"

class vtkTestSource : public vtkAlgorithm
{
  public:
    static vtkTestSource *New();
    vtkTypeRevisionMacro(vtkTestSource,vtkAlgorithm);
    void PrintSelf(ostream& os, vtkIndent indent);

    vtkTest* GetOutput();

    virtual int ProcessRequest(vtkInformation*,
                               vtkInformationVector**,
                               vtkInformationVector*);

  protected:
    vtkTestSource();
    ~vtkTestSource();

    void RequestData(vtkInformation* request,
                     vtkInformationVector** inputVector,
                     vtkInformationVector* outputVector);
    virtual void RequestInformation (vtkInformation*,
                                     vtkInformationVector**,
                                     vtkInformationVector*);

      // see algorithm for more info
    virtual int FillOutputPortInformation(int port, vtkInformation* info);

  private:
    vtkTestSource(const vtkTestSource&);  // Not implemented.
    void operator=(const vtkTestSource&);  // Not implemented.
};

#endif

