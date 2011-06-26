/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkTestSource.h,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkTestSource
// .SECTION Description
// vtkTestSource

#ifndef __vtkTestSource_h
#define __vtkTestSource_h

#include "vtkGraphAlgorithm.h"

class vtkTestSource : public vtkGraphAlgorithm 
{
public:
  vtkTypeRevisionMacro(vtkTestSource,vtkGraphAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkTestSource *New();
	
protected:
  vtkTestSource();
  ~vtkTestSource();
  
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);
  
  int RequestDataObject(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  vtkTestSource(const vtkTestSource&);  // Not implemented.
  void operator=(const vtkTestSource&);  // Not implemented.

};

#endif
