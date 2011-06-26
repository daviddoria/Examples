/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkTest.h,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkTest
// .SECTION Description
// vtkTest 

#ifndef __vtkTest_h
#define __vtkTest_h

#include "vtkObject.h"

class vtkInformation;
class vtkInformationVector;

class vtkTest : public vtkObject
{
public:
  vtkTypeRevisionMacro(vtkTest,vtkObject);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkTest *New();
  	
  vtkSetMacro(Value,double);
  vtkGetMacro(Value,double);
protected:
  vtkTest();
  ~vtkTest();
  
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  vtkTest(const vtkTest&);  // Not implemented.
  void operator=(const vtkTest&);  // Not implemented.
  
  double Value;
};

#endif
