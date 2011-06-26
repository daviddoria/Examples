/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkTestSource.cxx,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkTestSource.h"

#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"
#include "vtkDataObject.h"
#include "vtkSmartPointer.h"
#include "vtkMutableUndirectedGraph.h"
#include "vtkUndirectedGraph.h"
#include "vtkGraph.h"

vtkCxxRevisionMacro(vtkTestSource, "$Revision: 1.70 $");
vtkStandardNewMacro(vtkTestSource);

vtkTestSource::vtkTestSource()
{
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

vtkTestSource::~vtkTestSource()
{

}

int vtkTestSource::RequestData(vtkInformation *vtkNotUsed(request),
                                             vtkInformationVector **inputVector,
                                             vtkInformationVector *outputVector)
{
  
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  
  vtkGraph *output = vtkGraph::SafeDownCast(
      outInfo->Get(vtkDataObject::DATA_OBJECT()));
  
  vtkSmartPointer<vtkMutableUndirectedGraph> NewGraph = vtkSmartPointer<vtkMutableUndirectedGraph>::New();
    
  //add 3 vertices
  NewGraph->AddVertex();
  NewGraph->AddVertex();
  NewGraph->AddVertex();
  
  output->ShallowCopy(NewGraph);
  
  return 1;
}

int vtkTestSource::RequestDataObject(
                                       vtkInformation*, 
                                       vtkInformationVector**, 
                                       vtkInformationVector* )
{

  vtkUndirectedGraph *output = 0;
  output = vtkUndirectedGraph::New();
  
  
  this->GetExecutive()->SetOutputData(0, output);
  output->Delete();

  return 1;
}


//----------------------------------------------------------------------------
void vtkTestSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}

