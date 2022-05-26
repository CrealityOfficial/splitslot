#ifndef SPLITSLOT_SPLIT_1650772575656_H
#define SPLITSLOT_SPLIT_1650772575656_H
#include "splitslot/interface.h"
#include "trimesh2/Vec.h"
#include "trimesh2/Box.h"
#include <vector>

namespace trimesh
{
	class TriMesh;
}

namespace ccglobal
{
	class Tracer;
}

namespace splitslot
{
	struct SplitPlane
	{
		trimesh::vec3 normal;
		trimesh::vec3 position;
	};

	struct SplitSlotParam
	{
		bool  haveSlot;//是否生成卡扣
		float redius;
		float depth;
		float gap;//图案间隔长度
		float xyOffset;//xy内缩尺寸
		float zOffset;//z内缩尺寸
	};

	SPLITSLOT_API bool splitSlot(trimesh::TriMesh* input, const SplitPlane& plane, const SplitSlotParam& param,
		std::vector<trimesh::TriMesh*>& outMeshes, bool NeedMerge = true ,bool NeedRepair = true);

	SPLITSLOT_API bool splitSlotBox(trimesh::TriMesh* input, const trimesh::box3& box, const SplitSlotParam& param,
		std::vector<trimesh::TriMesh*>& outMeshes, ccglobal::Tracer* tracer = nullptr);

	SPLITSLOT_API bool splitPlaneAndBox(trimesh::TriMesh* input, const SplitPlane& plane, const trimesh::box3& box, const SplitSlotParam& param,
		std::vector<trimesh::TriMesh*>& outMeshes, ccglobal::Tracer* tracer = nullptr);
}

#endif // SPLITSLOT_SPLIT_1650772575656_H