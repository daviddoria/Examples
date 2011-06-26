#include "vtkTest.h"

#include "vtkCamera.h"
#include "vtkCellType.h"
#include "vtkCoordinate.h"
#include "vtkDataArray.h"
#include "vtkExtractSelectedFrustum.h"
#include "vtkIdTypeArray.h"
#include "vtkIdList.h"
#include "vtkIntArray.h"
//#include "vtkLabelHierarchyIterator.h"
//#include "vtkLabelHierarchyPrivate.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkCoincidentPoints.h"
#include "vtkPlanes.h"
#include "vtkPoints.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkPythagoreanQuadruples.h"
#include "vtkRenderer.h"
#include "vtkSmartPointer.h"
#include "vtkTextProperty.h"

#include <octree/octree>
#include <vtkstd/deque>
#include <vtkstd/set>
#include <vtkstd/vector>
#include <vtkstd/map>

#include <stdlib.h>

vtkStandardNewMacro(vtkTest);
vtkCxxRevisionMacro(vtkTest,"$Revision: 1.46 $");

vtkTest::vtkTest()
{
  this->Value = 4.5;
}

vtkTest::~vtkTest()
{

}

void vtkTest::PrintSelf( ostream& os, vtkIndent indent )
{
  this->Superclass::PrintSelf( os, indent );
}
