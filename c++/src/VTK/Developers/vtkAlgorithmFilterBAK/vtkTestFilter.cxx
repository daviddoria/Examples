/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkTestReader.cxx,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkTestReader.h"
#include "vtkTest.h"

#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"
#include "vtkDataObject.h"
#include "vtkSmartPointer.h"

vtkCxxRevisionMacro(vtkTestReader, "$Revision: 1.70 $");
vtkStandardNewMacro(vtkTestReader);

vtkTestReader::vtkTestReader()
{
  this->FileName = NULL;
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

vtkTestReader::~vtkTestReader()
{

}

int vtkTestReader::FillOutputPortInformation( int port, vtkInformation* info )
{
  if ( port == 0 )
  {
    //info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkTest" );
    info->Set(vtkObject::DATA_TYPE_NAME(), "vtkTest" );
    return 1;
  }

  return 0;
}

int vtkTestReader::RequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **vtkNotUsed(inputVector),
  vtkInformationVector *outputVector)
{
  
  // get the info object
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  
  // get the ouptut
   vtkTest *output = vtkTest::SafeDownCast(
                    outInfo->Get(vtkObject::DATA_OBJECT()));
	  //outInfo->Get(vtkDataObject::DATA_OBJECT()));
  
  vtkSmartPointer<vtkTest> test = vtkSmartPointer<vtkTest>::New();
  test->SetValue(5.1) ;
  
  output = test;
    
  return 1;
}


//----------------------------------------------------------------------------
void vtkTestReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  os << indent << "File Name: " 
      << (this->FileName ? this->FileName : "(none)") << "\n";	
}

