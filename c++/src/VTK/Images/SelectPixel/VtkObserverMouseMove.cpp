#include "VtkObserverMouseMove.h"
#include "vtkImageData.h"

#include <sstream>

#define FmtStr(str_out, strm) { std::ostringstream oss; oss << strm; str_out = oss.str(); }

VtkObserverMouseMove::VtkObserverMouseMove(vtkImageViewer2* pImageViewer,
                                           vtkRenderWindowInteractor* pIren,
                                           vtkPointPicker* pPicker)
{
    m_pImageViewer = pImageViewer;
    m_pIren = pIren;
    m_pPicker = pPicker;
}

void VtkObserverMouseMove::Execute(vtkObject *caller, unsigned long, void*) 
{
    // Do the pick. It will return a non-zero value if we intersected the image.
    if (!m_pPicker->Pick(m_pIren->GetEventPosition()[0], 
                         m_pIren->GetEventPosition()[1], 
                         0,  // always zero.
                         m_pImageViewer->GetRenderer())) 
    {
        std::cout << "Mouse is outside of image extent." << std::endl;
        m_pIren->Render();
        return;
    }

    // Get the mapped position of the mouse using the picker.
    double ptMapped[3];
    m_pPicker->GetMapperPosition(ptMapped);

    // Get a shortcut to the pixel data.
    vtkImageData* pImageData = m_pImageViewer->GetInput();

    // Get the pixel that was clicked
    std::stringstream ss;
    ss << "Pixel (" << ptMapped[0] << "," << ptMapped[1] << ")";
    m_strDetails = ss.str();

    // Output the pixel coordinates/value
    vtkstd::cout << m_strDetails << vtkstd::endl;
    m_pIren->Render();
    
}