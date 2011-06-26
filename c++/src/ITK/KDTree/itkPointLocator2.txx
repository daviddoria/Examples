/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkPointLocator2.txx,v $
  Language:  C++
  Date:      $Date: 2006-03-19 04:36:59 $
  Version:   $Revision: 1.19 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkPointLocator2_txx
#define __itkPointLocator2_txx
#include "itkPointLocator2.h"

namespace itk
{

template <class TMesh>
PointLocator2<TMesh>
::PointLocator2()
{
  this->m_SampleAdaptor = SampleAdaptorType::New();
  this->m_KdTreeGenerator = TreeGeneratorType::New();
}

template <class TMesh>
PointLocator2<TMesh>
::~PointLocator2()
{
}

template <class TMesh>
void
PointLocator2<TMesh>
::Initialize()
{
  this->m_SampleAdaptor = SampleAdaptorType::New();
  this->m_KdTreeGenerator = TreeGeneratorType::New();

  // Lack of const-correctness in the PointSetAdaptor should be fixed.
  this->m_SampleAdaptor->SetPointSet( 
    const_cast< PointSetType * >( this->m_PointSet.GetPointer() ) );

  this->m_SampleAdaptor->SetMeasurementVectorSize( PointDimension );

  this->m_KdTreeGenerator->SetSample( this->m_SampleAdaptor );
  this->m_KdTreeGenerator->SetBucketSize( 16 );

  this->m_KdTreeGenerator->Update();

  this->m_Tree = this->m_KdTreeGenerator->GetOutput();
}


template <class TMesh>
void
PointLocator2<TMesh>
::Search(const PointType &query,
         unsigned int numberOfNeighborsRequested,
         InstanceIdentifierVectorType& result) const
{
  this->m_Tree->Search( query, numberOfNeighborsRequested, result );
}


template <class TMesh>
void
PointLocator2<TMesh>
::Search(const PointType &query,
         double radius,
         InstanceIdentifierVectorType& result) const
{
  this->m_Tree->Search( query, radius, result );
}


/**
 * Print out internals
 */
template <class TMesh>
void
PointLocator2<TMesh>
::PrintSelf(std::ostream& os, Indent indent) const
{
  Superclass::PrintSelf(os, indent);
}

} // end namespace itk

#endif
