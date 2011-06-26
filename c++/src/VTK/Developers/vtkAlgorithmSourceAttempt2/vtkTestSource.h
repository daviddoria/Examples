#ifndef __vtkTestSource_h
#define __vtkTestSource_h

#include "vtkAlgorithm.h"

class vtkDataArraySelection;
class vtkDataSet;
class vtkDataSetAttributes;
class vtkXMLDataElement;
class vtkXMLDataParser;
class vtkInformationVector;
class vtkInformation;

class vtkTestSource : public vtkAlgorithm
{
  public:
    vtkTypeRevisionMacro(vtkTestSource, vtkAlgorithm);
    void PrintSelf(ostream& os, vtkIndent indent);
    
    static vtkTestSource *New();
	
  // Description:
  // Get the output as a vtkDataSet pointer.
    vtkDataSet* GetOutputAsDataSet();
    vtkDataSet* GetOutput();
    vtkDataSet* GetOutputAsDataSet(int index);
  
  // For the specified port, copy the information this reader sets up in
  // SetupOutputInformation to outInfo
    virtual void CopyOutputInformation(vtkInformation *vtkNotUsed(outInfo),
                                       int vtkNotUsed(port)) {}

  protected:
    vtkTestSource();
    ~vtkTestSource();
    
    //int FillInputPortInformation( int port, vtkInformation* info );
    int FillOutputPortInformation( int port, vtkInformation* info );
  // Setup the output's information.
    virtual void SetupOutputInformation(vtkInformation *vtkNotUsed(outInfo)) {}
  
  // Setup the output's information for the update extent
    virtual void SetupUpdateExtentInformation
        (vtkInformation *vtkNotUsed(outInfo)) {}

  // Setup the output's data with allocation.
    virtual void SetupOutputData();
  
    // Create a vtkAbstractArray from its cooresponding XML representation.
  // Does not allocate.
    vtkAbstractArray* CreateArray(vtkXMLDataElement* da);

    virtual int RequestData(vtkInformation *request,
                            vtkInformationVector **inputVector,
                            vtkInformationVector *outputVector);
    
    virtual int RequestDataObject(vtkInformation *vtkNotUsed(request),
                                  vtkInformationVector **vtkNotUsed(inputVector),
                                      vtkInformationVector *vtkNotUsed(outputVector))
    { return 1; }
    
    vtkInformation* GetCurrentOutputInformation();
  
  private:
  
    vtkDataObject* CurrentOutput;
    vtkInformation* CurrentOutputInformation;
  
  private:
    vtkTestSource(const vtkTestSource&);  // Not implemented.
    void operator=(const vtkTestSource&);  // Not implemented.
};

#endif