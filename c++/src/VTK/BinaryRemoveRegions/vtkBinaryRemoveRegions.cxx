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

int vtkBinaryRemoveRegions::RequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  
  vtkImageData *input = vtkImageData::SafeDownCast(
      inInfo->Get(vtkDataObject::DATA_OBJECT()));
 
  vtkImageData *output = vtkImageData::SafeDownCast(
      outInfo->Get(vtkDataObject::DATA_OBJECT()));
  
  vtkIdType inIncX, inIncY, inIncZ;
  vtkIdType outIncX, outIncY, outIncZ;

  int *inExt = input->GetExtent();
  int *outExt = input->GetExtent();
  int numComp;
  
  // Get information to march through data
  input->GetIncrements(inIncX, inIncY, inIncZ); 
  output->GetContinuousIncrements(outExt, outIncX, outIncY, outIncZ);
  numComp = inArray->GetNumberOfComponents();
  
  /// was in vtkMedian, it didn't have any influence here...
  /*inPtr = static_cast<T *>(
    inArray->GetVoidPointer( inExt[0] * inIncX + 
                             inExt[2] * inIncY +
                             inExt[4] * inIncZ) ); */             

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
