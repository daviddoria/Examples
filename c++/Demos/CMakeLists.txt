cmake_minimum_required(VERSION 2.6)

PROJECT(VTKExamples)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(demo_vtk_3ds_importer demo_vtk_3ds_importer.cpp )
TARGET_LINK_LIBRARIES(demo_vtk_3ds_importer vtkHybrid)

ADD_EXECUTABLE(demo_vtk_Delaunay3D demo_vtk_Delaunay3D.cpp )
TARGET_LINK_LIBRARIES(demo_vtk_Delaunay3D  vtkHybrid)

ADD_EXECUTABLE(demo_vtk_PerlinTerrain demo_vtk_PerlinTerrain.cpp )
TARGET_LINK_LIBRARIES( demo_vtk_PerlinTerrain  vtkHybrid)

ADD_EXECUTABLE(demo_vtk_subsample_grid demo_vtk_subsample_grid.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_subsample_grid vtkHybrid)

ADD_EXECUTABLE(demo_vtk_arrays demo_vtk_arrays.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_arrays vtkHybrid)

ADD_EXECUTABLE(demo_vtk_delMesh demo_vtk_delMesh.cpp)
TARGET_LINK_LIBRARIES( demo_vtk_delMesh vtkHybrid)

ADD_EXECUTABLE(demo_vtk_plane_texture demo_vtk_plane_texture.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_plane_texture vtkHybrid)

ADD_EXECUTABLE(demo_vtk_wrap_comb demo_vtk_wrap_comb.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_wrap_comb vtkHybrid)

ADD_EXECUTABLE(demo_vtk_assembly demo_vtk_assembly.cpp  )
TARGET_LINK_LIBRARIES(demo_vtk_assembly   vtkHybrid)

ADD_EXECUTABLE(demo_vtk_filter_CADPart demo_vtk_filter_CADPart.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_filter_CADPart vtkHybrid)

ADD_EXECUTABLE(demo_vtk_point2cell demo_vtk_point2cell.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_point2cell vtkHybrid)

ADD_EXECUTABLE(demo_vtk_band_contour_terrain demo_vtk_band_contour_terrain.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_band_contour_terrain vtkHybrid)

ADD_EXECUTABLE(demo_vtk_gen_tex_coord demo_vtk_gen_tex_coord.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_gen_tex_coord vtkHybrid)

ADD_EXECUTABLE(demo_vtk_procrustesAlignment demo_vtk_procrustesAlignment.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_procrustesAlignment vtkHybrid)

ADD_EXECUTABLE(test test.cpp)
TARGET_LINK_LIBRARIES(test vtkHybrid)

ADD_EXECUTABLE(demo_vtk_CADPart demo_vtk_CADPart.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_CADPart vtkHybrid)

ADD_EXECUTABLE(demo_vtk_hello demo_vtk_hello.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_hello vtkHybrid)

ADD_EXECUTABLE(demo_vtk_rainbow demo_vtk_rainbow.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_rainbow vtkHybrid)

ADD_EXECUTABLE(vtk_minimal vtk_minimal.cpp)
TARGET_LINK_LIBRARIES(vtk_minimal vtkHybrid)          
                                  
ADD_EXECUTABLE(demo_vtk_color_iso_surf demo_vtk_color_iso_surf.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_color_iso_surf vtkHybrid)          

ADD_EXECUTABLE(demo_vtk_ice_cream demo_vtk_ice_cream.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_ice_cream vtkHybrid)          

ADD_EXECUTABLE(demo_vtk_render_large_image demo_vtk_render_large_image.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_render_large_image vtkHybrid)          

ADD_EXECUTABLE(demo_vtk_constrainedDelaunay demo_vtk_constrainedDelaunay.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_constrainedDelaunay vtkHybrid)          

ADD_EXECUTABLE(demo_vtk_image_wrap demo_vtk_image_wrap.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_image_wrap vtkHybrid)          

ADD_EXECUTABLE(demo_vtk_simple_raycast demo_vtk_simple_raycast.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_simple_raycast vtkHybrid vtkVolumeRendering)          

ADD_EXECUTABLE(demo_vtk_contour2D demo_vtk_contour2D.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_contour2D vtkHybrid)          

ADD_EXECUTABLE(demo_vtk_marching demo_vtk_marching.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_marching vtkHybrid)          

ADD_EXECUTABLE(demo_vtk_simple_texturemap2d demo_vtk_simple_texturemap2d.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_simple_texturemap2d vtkHybrid vtkVolumeRendering)          

ADD_EXECUTABLE(demo_vtk_cspline demo_vtk_cspline.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_cspline vtkHybrid)          

ADD_EXECUTABLE(demo_vtk_medical_vol16_2 demo_vtk_medical_vol16_2.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_medical_vol16_2 vtkHybrid)          

ADD_EXECUTABLE(demo_vtk_stl_reader demo_vtk_stl_reader.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_stl_reader vtkHybrid)          

ADD_EXECUTABLE(demo_vtk_cut_combustor demo_vtk_cut_combustor.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_cut_combustor vtkHybrid)          

ADD_EXECUTABLE(demo_vtk_medical_vol16_3 demo_vtk_medical_vol16_3.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_medical_vol16_3 vtkHybrid)   
  
#ADD_EXECUTABLE(demo_vtk_streamline demo_vtk_streamline.cpp)
#TARGET_LINK_LIBRARIES(demo_vtk_streamline vtkHybrid)   
#error: ‘class vtkStreamTracer’ has no member named ‘SetMaximumPropagationUnitToTimeUnit’
#error: ‘class vtkStreamTracer’ has no member named ‘SetInitialIntegrationStepUnitToCellLengthUnit’

ADD_EXECUTABLE(demo_vtk_deci_pro demo_vtk_deci_pro.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_deci_pro vtkHybrid)   

ADD_EXECUTABLE(demo_vtk_medical_vol16 demo_vtk_medical_vol16.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_medical_vol16 vtkHybrid)   

ADD_EXECUTABLE(demo_vtk_strips demo_vtk_strips.cpp)
TARGET_LINK_LIBRARIES(demo_vtk_strips vtkHybrid)   

