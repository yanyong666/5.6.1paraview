vtk_module(vtkPVVTKExtensionsCore
  GROUPS
    ParaViewCore
  DEPENDS
    vtkFiltersCore
    vtkFiltersSources
    vtkIOImage
    vtkParallelCore
    vtkPVCommon
    vtkCommonMisc
  PRIVATE_DEPENDS
    vtksys
    vtkjsoncpp
  TEST_LABELS
    PARAVIEW
  KIT
    vtkPVExtensions
)