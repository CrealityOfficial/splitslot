#ifndef SPLITSLOT_SPLIT_1650772575656_H
#define SPLITSLOT_SPLIT_1650772575656_H
#include "splitslot/interface.h"
#include "trimesh2/Vec.h"

namespace trimesh
{
	class TriMesh;
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
		bool  haveSlot;//�Ƿ����ɿ���
		float redius;
		float depth;
		float gap;//ͼ���������
		float xyOffset;//xy�����ߴ�
		float zOffset;//z�����ߴ�
	};

	SPLITSLOT_API bool splitSlot(trimesh::TriMesh* input, const SplitPlane& plane, const SplitSlotParam& param,
		trimesh::TriMesh** out1, trimesh::TriMesh** out2);
}

#endif // SPLITSLOT_SPLIT_1650772575656_H