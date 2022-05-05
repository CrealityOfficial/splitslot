#include "split.h"

#include "fmesh/generate/generator.h"
#include "fmesh/generate/specialpoly.h"
#include "fmesh/font/fontoutlinecenter.h"
#include "fmesh/contour/polytree.h"
#include "cmesh/poly/roof.h"
#include "mmesh/trimesh/polygonstack.h"
#include "mmesh/trimesh/trimeshutil.h"
#include "mmesh/create/createcylinder.h"
#include "mmesh/trimesh/split.h"
#include "mmesh/util/drill.h"
#include "trimesh2/Vec3Utils.h"
#include "trimesh2/quaternion.h"

namespace splitslot
{
	bool CyInMesh(ClipperLibXYZ::Path& _Path, ClipperLibXYZ::IntPoint apoint)
	{
		float Rx = 2.0;//半径
		ClipperLibXYZ::Path newPath;//圆轮廓
		for (unsigned int i = 0; i < 100; i++)//顺时针取点
		{
			float Angle = 2 * M_PIf * (i / 100.0);
			trimesh::vec3 p;
			p.x = (apoint.X + Rx * cosf(Angle) * 1000);
			p.y = (apoint.Y + Rx * sinf(Angle) * 1000);

			//
			if (_Path.size() < 1)
				return false;
			int crossings = 0;
			ClipperLibXYZ::IntPoint p0 = _Path[_Path.size() - 1];
			for (unsigned int n = 0; n < _Path.size(); n++)
			{
				ClipperLibXYZ::IntPoint p1 = _Path[n];
				if ((p0.Y >= p.at(1) && p1.Y < p.at(1)) || (p1.Y > p.at(1) && p0.Y <= p.at(1)))
				{
					int64_t x = p0.X + (p1.X - p0.X) * (p.at(1) - p0.Y) / (p1.Y - p0.X);
					if (x >= p.at(0))
						crossings++;
				}
				p0 = p1;
			}
			if (crossings % 2 == 0)//点在轮廓外部
			{
				return false;
			}
		}
		return true;
	}


	bool splitSlot(trimesh::TriMesh* input, const SplitPlane& plane, const SplitSlotParam& param,
		std::vector<trimesh::TriMesh*>& outMeshes)
	{
		size_t vertex_size = input->vertices.size();
		if (vertex_size == 0)
			return false;

		trimesh::vec3 pos = plane.position;

		std::vector<float> distances;
		distances.resize(vertex_size);

#define min_value 1e-4
		bool allPositive = true;
		bool allNegtive = true;
		for (int i = 0; i < vertex_size; ++i)
		{
			trimesh::vec3 d = input->vertices.at(i) - pos;
			distances.at(i) = plane.normal.dot(d);

			if (distances.at(i) < -min_value)
				allPositive = false;
			if (distances.at(i) > min_value)
				allNegtive = false;
		}

		if (allPositive || allNegtive)
			return false;

		std::vector<trimesh::TriMesh::Face> mesh1;
		std::vector<trimesh::TriMesh::Face> mesh2;
		std::vector<trimesh::TriMesh::Face> collideFaces;

		for (trimesh::TriMesh::Face& f : input->faces)
		{
			float l0 = distances.at(f.x);
			float l1 = distances.at(f.y);
			float l2 = distances.at(f.z);
			if (l0 > 0.0f && l1 > 0.0f && l2 > 0.0f)
			{
				mesh1.push_back(f);
			}
			else if (l0 < 0.0f && l1 < 0.0f && l2 < 0.0f)
			{
				mesh2.push_back(f);
			}
			else
				collideFaces.push_back(f);
		}

		trimesh::TriMesh* m1 = new trimesh::TriMesh();
		trimesh::TriMesh* m2 = new trimesh::TriMesh();
		m1->faces.swap(mesh1);
		m2->faces.swap(mesh2);

		auto fillmesh = [&input](trimesh::TriMesh* m) {
			for (trimesh::TriMesh::Face& f : m->faces)
			{
				m->vertices.push_back(input->vertices.at(f.x));
				m->vertices.push_back(input->vertices.at(f.y));
				m->vertices.push_back(input->vertices.at(f.z));
			}

			int index = 0;
			//remap
			for (trimesh::TriMesh::Face& f : m->faces)
			{
				f.x = index++;
				f.y = index++;
				f.z = index++;
			}
		};

		fillmesh(m1);
		fillmesh(m2);
		auto addmesh = [](trimesh::TriMesh* mesh, const trimesh::vec3& v1, const trimesh::vec3& v2, const trimesh::vec3& v3) {
			int index = (int)mesh->vertices.size();
			mesh->vertices.push_back(v1);
			mesh->vertices.push_back(v2);
			mesh->vertices.push_back(v3);

			mesh->faces.push_back(trimesh::TriMesh::Face(index, index + 1, index + 2));
		};

		std::vector<trimesh::vec3> lines;
		//process collide
		trimesh::box3 box = input->bbox;
		auto fcollid = [&box, &addmesh, &input, &distances, &m1, &m2, &lines](int t, int i1, int i2)
		{
			trimesh::vec3& tv = input->vertices.at(t);
			trimesh::vec3& v1 = input->vertices.at(i1);
			trimesh::vec3& v2 = input->vertices.at(i2);
			float dv = distances.at(t);
			float d1 = distances.at(i1);
			float d2 = distances.at(i2);

			if (d1 == 0.0f && d2 == 0.0f)
			{
				if (dv >= 0.0f)
				{
					addmesh(m1, tv, v1, v2);

					lines.push_back(v1);
					lines.push_back(v2);
				}
				else
				{
					addmesh(m2, tv, v1, v2);

					lines.push_back(v2);
					lines.push_back(v1);
				}
			}
			else if (d1 == 0.0f && dv * d2 >= 0.0f)
			{
				if (dv > 0.0f || dv == 0.0f && d2 > 0.0)
				{
					addmesh(m1, tv, v1, v2);
					if (dv == 0.0f)
					{
						lines.push_back(tv);
						lines.push_back(v1);
					}
				}
				else
				{
					addmesh(m2, tv, v1, v2);
					if (dv == 0.0f)
					{
						lines.push_back(v1);
						lines.push_back(tv);
					}
				}
			}
			else if (d2 == 0.0f && dv * d1 >= 0.0f)
			{
				if (dv > 0.0f || dv == 0.0f && d1 > 0.0f)
				{
					addmesh(m1, tv, v1, v2);
					if (dv == 0.0f)
					{
						lines.push_back(v2);
						lines.push_back(tv);
					}
				}
				else
				{
					addmesh(m2, tv, v1, v2);
					if (dv == 0.0f)
					{
						lines.push_back(tv);
						lines.push_back(v2);
					}
				}
			}
			else
			{
				trimesh::vec3 c1 = (dv / (dv - d1)) * v1 - (d1 / (dv - d1)) * tv;
				trimesh::vec3 c2 = (dv / (dv - d2)) * v2 - (d2 / (dv - d2)) * tv;

#ifdef _DEBUG
				if (c1.x < box.min.x || c1.x > box.max.x || c1.y < box.min.y || c1.y > box.max.y
					|| c2.x < box.min.x || c2.x > box.max.x || c2.y < box.min.y || c2.y > box.max.y)
				{
					printf("error");
				}
#endif

				if (dv > 0.0f)
				{
					addmesh(m1, tv, c1, c2);
					addmesh(m2, c2, c1, v2);
					addmesh(m2, c1, v1, v2);

					lines.push_back(c1);
					lines.push_back(c2);
				}
				else if (dv < 0.0f)
				{
					addmesh(m2, tv, c1, c2);
					addmesh(m1, c2, c1, v2);
					addmesh(m1, c1, v1, v2);

					lines.push_back(c2);
					lines.push_back(c1);
				}
				else
				{
					if (d1 > 0.0f)
					{
						addmesh(m1, tv, v1, v2);
					}
					else if (d1 < 0.0f)
					{
						addmesh(m2, tv, v1, v2);
					}
				}
			}
		};

		int faceNum = (int)input->faces.size();
		for (int i = 0; i < faceNum; ++i)
		{
			trimesh::TriMesh::Face& f = input->faces.at(i);
			float l0 = distances.at(f.x) * distances.at(f.y);
			float l1 = distances.at(f.y) * distances.at(f.z);
			float l2 = distances.at(f.x) * distances.at(f.z);
			if (distances.at(f.x) == 0.0f && distances.at(f.y) == 0.0f && distances.at(f.z) == 0.0f)
				continue;

			if (l0 >= 0.0f && (l1 <= 0.0f || l2 <= 0.0f))
			{
				fcollid(f.z, f.x, f.y);
			}
			else if (l0 < 0.0f)
			{
				if (l1 <= 0.0f)
				{
					fcollid(f.y, f.z, f.x);
				}
				else if (l2 <= 0.0f)
				{
					fcollid(f.x, f.y, f.z);
				}
			}

			if (i % 1000 == 0)
			{
				;
			}
		}

		//fill hole
		std::vector<std::vector<int>> polygons;
		std::vector<trimesh::vec3> points;
		mmesh::lines2polygon(lines, polygons, points);

		trimesh::TriMesh* destmeshes = new trimesh::TriMesh();
		if (param.haveSlot)
		{
			std::vector<trimesh::vec3> sectionPoints = points;
			//绕z和y旋转，使平面变平
			const trimesh::vec3 XYnormal(0.0f, 0.0f, 1.0f);
			trimesh::quaternion q = q.rotationTo(XYnormal, plane.normal);
			trimesh::fxform xf = fromQuaterian(q);
			trimesh::box3 box2;
			for (trimesh::point& apoint : sectionPoints)
			{
				box2 += apoint;
			}
			trimesh::vec3 pointCenter = box2.center();

			for (trimesh::point& apoint : sectionPoints)
			{
				apoint = xf * (apoint - pointCenter);
			}

			ClipperLibXYZ::Paths sourcePaths;
			for (std::vector<int>& apolygon : polygons)
			{
				sourcePaths.push_back(ClipperLibXYZ::Path());
				trimesh::box3 abox3;
				for (int index : apolygon)
				{
					sourcePaths[sourcePaths.size() - 1].push_back(ClipperLibXYZ::IntPoint(sectionPoints[index].x * 1000, sectionPoints[index].y * 1000, sectionPoints[index].z * 1000));
					abox3 += sectionPoints[index];
				}

				ClipperLibXYZ::PolyTree apolytree;
				fmesh::convertPaths2PolyTree(&sourcePaths, apolytree);
				ClipperLibXYZ::Path* skeletonPath = new ClipperLibXYZ::Path;
				cmesh::skeletonPoints(&apolytree, skeletonPath);
				ClipperLibXYZ::Paths skeletonPaths;
				fmesh::sortPath(skeletonPath, &skeletonPaths);
				ClipperLibXYZ::Paths destPaths;
				fmesh::generateLines(skeletonPaths, destPaths, param.redius, param.gap, true);
				if (destPaths.empty() || destPaths[0].empty())
				{
					//ClipperLibXYZ::IntPoint circleCenterPoint(abox3.center().x * 1000, abox3.center().y * 1000, abox3.center().z * 1000);
					if (1/*CyInMesh(sourcePaths[sourcePaths.size() - 1], circleCenterPoint)*/)//圆柱是否超出轮廓边界
					{
						trimesh::point tPoint(abox3.center().x, abox3.center().y, abox3.center().z);
						tPoint = trimesh::inv(xf) * tPoint + pointCenter;
						destmeshes->vertices.push_back(tPoint);
					}
				}
				else
				{
					for (ClipperLibXYZ::Path& apath : destPaths)
					{
						for (ClipperLibXYZ::IntPoint& apoint : apath)
						{
							if (1/*CyInMesh(sourcePaths[sourcePaths.size() - 1], apoint)*/)//圆柱是否超出轮廓边界
							{
								trimesh::point tPoint(apoint.X / 1000.0, apoint.Y / 1000.0, apoint.Z / 1000.0);
								tPoint = trimesh::inv(xf) * tPoint + pointCenter;
								destmeshes->vertices.push_back(tPoint);
							}
						}
					}
				}
			}
		}

		std::vector<trimesh::TriMesh::Face> faces;
		std::vector<trimesh::dvec2> polygons2;
		polygons2.reserve(points.size());

		trimesh::vec3 zn = trimesh::vec3(0.0f, 0.0f, 1.0f);
		trimesh::vec3 axis = plane.normal TRICROSS zn;
		float angle = trimesh::vv_angle(axis, zn);
		trimesh::xform r = trimesh::xform::rot((double)angle, axis);
		for (size_t i = 0; i < points.size(); ++i)
		{
			trimesh::vec3 v = points.at(i);
			trimesh::dvec3 dv = trimesh::dvec3(v.x, v.y, v.z);
			trimesh::dvec3 p = r * dv;
			polygons2.push_back(trimesh::dvec2(p.x, p.y));
		}

		mmesh::PolygonStack pstack;
		pstack.generates(polygons, polygons2, faces, 0);

		bool fillHole = true;
		if (fillHole)
		{
			int start1 = (int)m1->vertices.size();
			int start2 = (int)m2->vertices.size();

			m1->vertices.insert(m1->vertices.end(), points.begin(), points.end());
			m2->vertices.insert(m2->vertices.end(), points.begin(), points.end());
			for (trimesh::TriMesh::Face& fs : faces)
			{
				trimesh::TriMesh::Face f1 = fs;
				f1 += trimesh::ivec3(start1, start1, start1);
				int t = f1[2];
				f1[2] = f1[1];
				f1[1] = t;
				m1->faces.push_back(f1);

				trimesh::TriMesh::Face f2 = fs;
				f2 += trimesh::ivec3(start2, start2, start2);
				m2->faces.push_back(f2);
			}
		}

		std::vector<mmesh::DrillParam> drillParams;
		std::vector<trimesh::TriMesh*> inMeshes;
		inMeshes.push_back(m2);
		for (trimesh::point& apoint : destmeshes->vertices)
		{
			trimesh::TriMesh* cMesh0 = new trimesh::TriMesh();
			cMesh0 = mmesh::createSoupCylinder(50, param.redius - param.xyOffset, param.depth - param.zOffset, apoint, plane.normal);
			mmesh::dumplicateMesh(cMesh0);
			inMeshes.push_back(cMesh0);

			trimesh::TriMesh* cMesh = new trimesh::TriMesh();
			cMesh = mmesh::createSoupCylinder(50, param.redius, param.depth, apoint, plane.normal);
			mmesh::dumplicateMesh(cMesh);
			
			mmesh::DrillParam adrillParam;
			adrillParam.cylinder_depth = param.depth;
			adrillParam.cylinder_Dir = plane.normal;
			adrillParam.cylinder_startPos = apoint;
			adrillParam.cylinder_radius = param.redius;
			adrillParam.cylinder_resolution = 50;
			drillParams.push_back(adrillParam);

			//trimesh::TriMesh* outTemp = mmesh::drill(m1, cMesh, nullptr, nullptr);
			//if (outTemp)
			//{
			//	m1 = outTemp;
			//}
		}

		trimesh::TriMesh* outTemp = mmesh::drillCylinder(m1, drillParams, nullptr, nullptr);
		if (outTemp)
		{
			m1 = outTemp;
		}

		mmesh::mergeTriMesh(m2, inMeshes);

		outMeshes.push_back(m1);
		outMeshes.push_back(m2);
		return true;
	}
}