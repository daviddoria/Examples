/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkPointLocator.h,v $
  Language:  C++
  Date:      $Date: 2003-09-10 14:29:23 $
  Version:   $Revision: 1.23 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkPointLocator2_h
#define __itkPointLocator2_h

#include "itkObject.h"
#include "itkKdTree.h"
#include "itkKdTreeGenerator.h"
#ifdef ITK_USE_REVIEW_STATISTICS
#include "itkPointSetToListSampleAdaptor.h"
#else
#include "itkPointSetToListAdaptor.h"
#endif

namespace itk
{

/** \class PointLocator2
 * \brief Accelerate geometric searches for points.
 *
 * This class accelerates the search for the closest point to a user-provided
 * point, by using constructing a Kd-Tree structure for the PointSet.
 *
 */
template < class TPointSet >
class ITK_EXPORT PointLocator2 : public Object
{
public:
  /** Standard class typedefs. */
  typedef PointLocator2               Self;
  typedef Object                      Superclass;
  typedef SmartPointer<Self>          Pointer;
  typedef SmartPointer<const Self>    ConstPointer;
    
  /** Method for creation through the object factory. */
  itkNewMacro(Self);

  /** Standard part of every itk Object. */
  itkTypeMacro(PointLocator2, Object);

  itkStaticConstMacro(PointDimension, unsigned int, TPointSet::PointDimension);

  /** Typedefs related to the PointSet type */
  typedef TPointSet                             PointSetType;
  typedef typename PointSetType::ConstPointer   PointSetConstPointer;
  typedef typename PointSetType::PointType      PointType;

  /** Type of the PointSet to List Adaptor. */
#ifdef ITK_USE_REVIEW_STATISTICS
  typedef itk::Statistics::PointSetToListSampleAdaptor< PointSetType >    SampleAdaptorType;
#else
  typedef itk::Statistics::PointSetToListAdaptor< PointSetType >    SampleAdaptorType;
#endif

  typedef typename SampleAdaptorType::Pointer                       SampleAdaptorPointer;

  /** Types fo the KdTreeGenerator */
  typedef itk::Statistics::KdTreeGenerator< SampleAdaptorType >     TreeGeneratorType;
  typedef typename TreeGeneratorType::Pointer                       TreeGeneratorPointer;
  typedef typename TreeGeneratorType::KdTreeType                    TreeType;
  typedef typename TreeType::ConstPointer                           TreeConstPointer;
  typedef typename TreeType::InstanceIdentifierVectorType           InstanceIdentifierVectorType;


  /** Connect the PointSet as input */
  itkSetConstObjectMacro( PointSet, PointSetType );
  itkGetConstObjectMacro( PointSet, PointSetType );

  /** Pre-Compute the KdTree structure that will later facilitate the search of
   * points */
  void Initialize();

  /** Searches the k-nearest neighbors */
  void Search(const PointType &query,
              unsigned int numberOfNeighborsRequested,
              InstanceIdentifierVectorType& result) const;

  /** Searches the neighbors fallen into a hypersphere */
  void Search(const PointType &query,
              double radius,
              InstanceIdentifierVectorType& result) const;

protected:
  PointLocator2();
  ~PointLocator2();
  virtual void PrintSelf(std::ostream& os, Indent indent) const;

private:
  PointLocator2(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  PointSetConstPointer     m_PointSet;
  SampleAdaptorPointer     m_SampleAdaptor;
  TreeGeneratorPointer     m_KdTreeGenerator;
  TreeConstPointer         m_Tree;
};

} // end namespace itk
  
#ifndef ITK_MANUAL_INSTANTIATION
#include "itkPointLocator2.txx"
#endif
  
#endif
