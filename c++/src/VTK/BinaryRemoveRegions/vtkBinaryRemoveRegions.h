/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkBinaryRemoveRegions.h,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkBinaryRemoveRegions - Threshold Filter
// .SECTION Description
// vtkBinaryRemoveRegions is just great.

#ifndef __vtkBinaryRemoveRegions_h
#define __vtkBinaryRemoveRegions_h

#include "vtkImageSpatialAlgorithm.h"

class VTK_IMAGING_EXPORT vtkBinaryRemoveRegions : public vtkImageSpatialAlgorithm
{
public:
  static vtkBinaryRemoveRegions *New();
  vtkTypeRevisionMacro(vtkBinaryRemoveRegions,vtkImageSpatialAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Set/Get the minimum regions size.
  vtkSetMacro(MinimumSize,long);
  vtkGetMacro(MinimumSize,long);

  // Description:
  // Set/Get the minimum regions size.
  vtkSetMacro(MaximumSize,long);
  vtkGetMacro(MaximumSize,long);

protected:
  vtkBinaryRemoveRegions();
  ~vtkBinaryRemoveRegions();

  long MinimumSize;
  long MaximumSize;

  int RequestData(vtkInformation *request,
                           vtkInformationVector **inputVector,
                           vtkInformationVector *outputVector
                           );

private:
  vtkBinaryRemoveRegions(const vtkBinaryRemoveRegions&);  // Not implemented.
  void operator=(const vtkBinaryRemoveRegions&);  // Not implemented.
};

#endif
