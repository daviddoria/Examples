#ifndef __VtkObserverMouseClick_h
#define __VtkObserverMouseClick_h

#include "vtkCommand.h"
#include "vtkImageViewer2.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkPointPicker.h"
#include "vtkTextMapper.h"
#include <string>
#include <sstream>

class VtkObserverMouseClick : public vtkCommand {

    public:
      VtkObserverMouseClick(vtkImageViewer2* pImageViewer,
                             vtkRenderWindowInteractor* pIren,
                             vtkPointPicker* pPicker);

      static VtkObserverMouseClick *New(vtkImageViewer2* pImageViewer,
                                         vtkRenderWindowInteractor* pIren,
                                         vtkPointPicker* pPicker) 
        {
          return new VtkObserverMouseClick(pImageViewer,
                                            pIren,
                                            pPicker);
        }
        
        ~VtkObserverMouseClick();

    protected:
        vtkImageViewer2* m_pImageViewer;
        vtkRenderWindowInteractor* m_pIren;
        vtkPointPicker* m_pPicker;

        std::string m_strDetails;

    private:
        virtual void Execute(vtkObject *caller, unsigned long, void*);
};
#endif