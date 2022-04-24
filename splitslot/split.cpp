#include "split.h"

#include "fmesh/generate/generator.h"
#include "fmesh/generate/specialpoly.h"
#include "fmesh/font/fontoutlinecenter.h"
#include "fmesh/contour/polytree.h"
#include "cmesh/poly/roof.h"
#include "mmesh/trimesh/polygonstack.h"
#include "mmesh/trimesh/trimeshutil.h"
#include "mmesh/create/createcylinder.h"
#include "mmesh/util/drill.h"
#include "trimesh2/Vec3Utils.h"

namespace splitslot
{
	bool splitSlot(trimesh::TriMesh* input, const SplitPlane& plane, const SplitSlotParam& param,
		trimesh::TriMesh** out1, trimesh::TriMesh** out2)
	{
		return true;
	}
}