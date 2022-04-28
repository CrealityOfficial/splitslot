__cc_find(Trimesh2)
__cc_find(FMesh)
__cc_find(CMesh)
__cc_find(Clipper)
__assert_target(trimesh2)
__assert_target(fmesh)
__assert_target(cmesh)
__assert_target(clipper)

set(SRCS splitslot/interface.h
		 splitslot/split.h
		 splitslot/split.cpp
		 )
		
set(INCS)
set(LIBS trimesh2 fmesh mmesh cmesh clipper)
set(DEFS)

__add_platform_library(splitslot)

if(CC_INSTALL_SPLITSLOT)
        __install_directory_specif(${CMAKE_CURRENT_SOURCE_DIR}/ splitslot)
endif()
