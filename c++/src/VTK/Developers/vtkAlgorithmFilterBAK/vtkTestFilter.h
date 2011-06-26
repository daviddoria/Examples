/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkTestReader.h,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkTestReader
// .SECTION Description
// vtkTestReader

#ifndef __vtkTestReader_h
#define __vtkTestReader_h

#include "vtkAlgorithm.h"

class vtkTestReader : public vtkAlgorithm 
{
public:
  vtkTypeRevisionMacro(vtkTestReader,vtkAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkTestReader *New();
	
protected:
  vtkTestReader();
  ~vtkTestReader();
  
  int FillOutputPortInformation( int port, vtkInformation* info );
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  vtkTestReader(const vtkTestReader&);  // Not implemented.
  void operator=(const vtkTestReader&);  // Not implemented.

};

#endif
