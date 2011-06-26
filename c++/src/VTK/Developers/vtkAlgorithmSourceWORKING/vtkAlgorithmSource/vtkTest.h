#ifndef __vtkTest_h
#define __vtkTest_h

//#include "vtkPointSet.h"
#include "vtkDataObject.h"

class vtkAbstractArray;
class vtkCamera;
class vtkCoincidentPoints;
class vtkDataArray;
class vtkIntArray;
class vtkPoints;
class vtkPolyData;
class vtkRenderer;
class vtkTextProperty;

class vtkTest : public vtkDataObject// : public vtkPointSet
{
  public:
    static vtkTest* New();
    vtkTypeRevisionMacro(vtkTest,vtkDataObject);
    void PrintSelf( ostream& os, vtkIndent indent );

    vtkGetMacro(Value, double);
    
  protected:
    vtkTest();
    ~vtkTest();

  private:
    vtkTest( const vtkTest& ); // Not implemented.
    void operator = ( const vtkTest& ); // Not implemented.
    
    double Value;
};

#endif 