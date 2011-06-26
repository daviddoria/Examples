/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkTest.cxx,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkTest.h"

#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"

vtkCxxRevisionMacro(vtkTest, "$Revision: 1.70 $");
vtkStandardNewMacro(vtkTest);

vtkTest::vtkTest()
{
  
}

vtkTest::~vtkTest()
{

}

int vtkTest::RequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **vtkNotUsed(inputVector),
  vtkInformationVector *outputVector)
{
  
  // get the info object
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  
  // get the ouptut
//  vtkPolyData *output = vtkPolyData::SafeDownCast(
//		  outInfo->Get(vtkDataObject::DATA_OBJECT()));
  
  //output->ShallowCopy(polydata);
  
  return 1;
}


//----------------------------------------------------------------------------
void vtkTest::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  os << indent << "Value: " << this->Value << "\n";	
}

