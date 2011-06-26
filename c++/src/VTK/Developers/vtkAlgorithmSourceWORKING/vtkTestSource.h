#ifndef __vtkTestSource_h
#define __vtkTestSource_h

#include "vtkTestAlgorithm.h"

class vtkTextProperty;

class VTK_RENDERING_EXPORT vtkTestSource : public vtkTestAlgorithm
{
  public:
    static vtkTestSource* New();
    vtkTypeRevisionMacro(vtkTestSource,vtkTestAlgorithm);
    virtual void PrintSelf( ostream& os, vtkIndent indent );

  // Description:
  // Set/get the "ideal" number of labels to associate with each node in the output hierarchy.
    vtkSetMacro(TargetLabelCount,int);
    vtkGetMacro(TargetLabelCount,int);

  // Description:
  // Set/get the maximum tree depth in the output hierarchy.
    vtkSetMacro(MaximumDepth,int);
    vtkGetMacro(MaximumDepth,int);

  // Description:
  // Whether to use unicode strings.
    vtkSetMacro(UseUnicodeStrings,bool);
    vtkGetMacro(UseUnicodeStrings,bool);
    vtkBooleanMacro(UseUnicodeStrings,bool);

  // Description:
  // Set/get the label array name.
    virtual void SetLabelArrayName(const char* name);
    virtual const char* GetLabelArrayName();

  // Description:
  // Set/get the priority array name.
    virtual void SetSizeArrayName(const char* name);
    virtual const char* GetSizeArrayName();

  // Description:
  // Set/get the priority array name.
    virtual void SetPriorityArrayName(const char* name);
    virtual const char* GetPriorityArrayName();

  // Description:
  // Set/get the icon index array name.
    virtual void SetIconIndexArrayName(const char* name);
    virtual const char* GetIconIndexArrayName();

  // Description:
  // Set/get the text orientation array name.
    virtual void SetOrientationArrayName(const char* name);
    virtual const char* GetOrientationArrayName();

  // Description:
  // Set/get the maximum text width (in world coordinates) array name.
    virtual void SetBoundedSizeArrayName(const char* name);
    virtual const char* GetBoundedSizeArrayName();

  // Description:
  // Set/get the text property assigned to the hierarchy.
    virtual void SetTextProperty(vtkTextProperty* tprop);
    vtkGetObjectMacro(TextProperty, vtkTextProperty);

  protected:
    vtkTestSource();
    virtual ~vtkTestSource();

    virtual int FillInputPortInformation( int port, vtkInformation* info );

    virtual int RequestData(
                            vtkInformation* request,
                            vtkInformationVector** inputVector,
                            vtkInformationVector* outputVector );

    int TargetLabelCount;
    int MaximumDepth;
    bool UseUnicodeStrings;
    vtkTextProperty* TextProperty;

  private:
    vtkTestSource( const vtkTestSource& ); // Not implemented.
    void operator = ( const vtkTestSource& ); // Not implemented.
};

#endif // __vtkPointSetToLabelHierarchy_h