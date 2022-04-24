__cc_find(Trimesh2)
__cc_find(FMesh)
__cc_find(CMesh)
__assert_target(trimesh2)
__assert_target(fmesh)
__assert_target(cmesh)

set(SRCS splitslot/interface.h
		 splitslot/split.h
		 splitslot/split.cpp
		 )
		
set(INCS)
set(LIBS trimesh2 fmesh mmesh cmesh)
set(DEFS)

__add_platform_library(splitslot)


