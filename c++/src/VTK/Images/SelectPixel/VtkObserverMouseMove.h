/*=========================================================================

  Program: DevSample_PickPixelValue

  Copyright (c) Mark Wyszomierski
  All rights reserved.
  See copyright.txt or http://www.devsample.org/copyright.txt for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE. See the above copyright notice for more information.

=========================================================================*/
#ifndef __VtkObserverMouseMove_h
#define __VtkObserverMouseMove_h

#include "vtkCommand.h"
#include "vtkImageViewer2.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkPointPicker.h"
#include "vtkTextMapper.h"
#include <string>
#include <sstream>

class VtkObserverMouseMove : public vtkCommand {

    public:
        VtkObserverMouseMove(vtkImageViewer2* pImageViewer,
                             vtkRenderWindowInteractor* pIren,
                             vtkPointPicker* pPicker);

        static VtkObserverMouseMove *New(vtkImageViewer2* pImageViewer,
                                         vtkRenderWindowInteractor* pIren,
                                         vtkPointPicker* pPicker) 
        {
            return new VtkObserverMouseMove(pImageViewer,
                                            pIren,
                                            pPicker);
        }

    protected:
        vtkImageViewer2* m_pImageViewer;
        vtkRenderWindowInteractor* m_pIren;
        vtkPointPicker* m_pPicker;

        std::string m_strDetails;

    private:
        virtual void Execute(vtkObject *caller, unsigned long, void*);
};
#endif