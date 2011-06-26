/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkBinaryRemoveRegions.cxx,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkBinaryRemoveRegions.h"

#include "vtkCellData.h"
#include "vtkDataArray.h"
#include "vtkImageData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkStreamingDemandDrivenPipeline.h"

vtkCxxRevisionMacro(vtkBinaryRemoveRegions, "$Revision: 1.00 $");
vtkStandardNewMacro(vtkBinaryRemoveRegions);

//-----------------------------------------------------------------------------
// Construct an instance of vtkBinaryRemoveRegions fitler.
vtkBinaryRemoveRegions::vtkBinaryRemoveRegions()
{
  this->MinimumSize = 3;
  this->MaximumSize = 32786;
}

//-----------------------------------------------------------------------------
vtkBinaryRemoveRegions::~vtkBinaryRemoveRegions()
{
}

//-----------------------------------------------------------------------------
void vtkBinaryRemoveRegions::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Minimum Size: " << this->MinimumSize << endl;
  os << indent << "Maximum Size: " << this->MaximumSize << endl;
}

//-----------------------------------------------------------------------------
// This method contains the second switch statement that calls the correct
// templated function for the mask types.
template <class T>
void vtkBinaryRemoveRegionsExecute(vtkBinaryRemoveRegions *self,
                             vtkImageData *inData, T *inPtr, 
                             vtkImageData *outData, T *outPtr,
                             int outExt[6], int id,
                             vtkDataArray *inArray)
{
  vtkIdType inIncX, inIncY, inIncZ;
  vtkIdType outIncX, outIncY, outIncZ;

  int *inExt = inData->GetExtent();
  int numComp;

  if (!inArray)
    {
    return;
    }
  
  // Get information to march through data
  inData->GetIncrements(inIncX, inIncY, inIncZ); 
  outData->GetContinuousIncrements(outExt, outIncX, outIncY, outIncZ);
  numComp = inArray->GetNumberOfComponents();
  
  /// was in vtkMedian, it didn't have any influence here...
  inPtr = static_cast<T *>(
    inArray->GetVoidPointer( inExt[0] * inIncX + 
                             inExt[2] * inIncY +
                             inExt[4] * inIncZ) );          

  long x, y, z;
  x = y = z = 0;
  // loop through the image data
  for (int outZ = outExt[4]; outZ <= outExt[5]; ++outZ)
    {
    for (int outY = outExt[2]; 
         !self->AbortExecute && outY <= outExt[3]; ++outY)
      {
      for (int outX = outExt[0]; outX <= outExt[1]; ++outX)
        {
        //for each scalar component
        for (int outC = 0; outC < numComp; outC++)
          {    
          *outPtr = *inPtr;
          inPtr++;
          outPtr++;
          }
        }
      outPtr += outIncY;
      inPtr += inIncY;
      }
    outPtr += outIncZ;
    inPtr += inIncZ;
    }
}

//-----------------------------------------------------------------------------
// This method contains the first switch statement that calls the correct
// templated function for the input and output region types.
void vtkBinaryRemoveRegions::ThreadedRequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *vtkNotUsed(outputVector),
  vtkImageData ***inData,
  vtkImageData **outData,
  int outExt[6], int id)
{
  /// my guess was to use the same Extents as the output
  //void *inPtr  = inData[0][0]->GetScalarPointerForExtent(outExt);
  /// outPtr as in vtkMedian
  void *outPtr = outData[0]->GetScalarPointerForExtent(outExt);

  vtkDataArray *inArray = this->GetInputArrayToProcess(0,inputVector);
  if (id == 0)
    {
    outData[0]->GetPointData()->GetScalars()->SetName(inArray->GetName());
    }

  ///this was in vtkMedian, here it results in a Segmentation fault
  void *inPtr = inArray->GetVoidPointer(0); 

  // this filter expects that input is the same type as output.
  if (inArray->GetDataType() != outData[0]->GetScalarType())
    {
    vtkErrorMacro(<< "Execute: input data type, " << inArray->GetDataType()
                << ", must match out ScalarType "
                  << outData[0]->GetScalarType());
    return;
    }
  
  switch (inArray->GetDataType())
    {
    vtkTemplateMacro(
      vtkBinaryRemoveRegionsExecute(this,inData[0][0],
                              static_cast<VTK_TT *>(inPtr), 
                              outData[0], static_cast<VTK_TT *>(outPtr),
                              outExt, id, inArray));
    default:
      vtkErrorMacro(<< "Execute: Unknown input ScalarType");
      return;
    }
}
