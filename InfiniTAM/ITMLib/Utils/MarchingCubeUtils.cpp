//ccjn modified it


#include "MarchingCubeUtils.h"
#include <iostream>
//#include <vcg/complex/complex.h>
//#include <wrap/io_trimesh/export.h>

myVertex VertexInterp(float isolevel, Vector3f &p1, Vector3f &p2, ITMVoxel &v1, ITMVoxel &v2)
{
	myVertex r1; r1.pos = p1;
	myVertex r2; r2.pos = p2;

	if(abs(isolevel - ITMVoxel::SDF_valueToFloat(v1.sdf)) < 0.00001f) return r1;
	if(abs(isolevel - ITMVoxel::SDF_valueToFloat(v2.sdf)) < 0.00001f) return r2;
	if(abs(ITMVoxel::SDF_valueToFloat(v1.sdf) -ITMVoxel::SDF_valueToFloat(v2.sdf)) <0.00001f) return r1;

	float mu = (isolevel - ITMVoxel::SDF_valueToFloat(v1.sdf)) / (ITMVoxel::SDF_valueToFloat(v2.sdf) - ITMVoxel::SDF_valueToFloat(v1.sdf));

	myVertex res;
	res.pos.x = p1.x + mu * (p2.x -p1.x);
	res.pos.y = p1.y + mu * (p2.y -p1.y);
	res.pos.z = p1.z + mu * (p2.z -p1.z);

	return res;
}

void writeToFileOFF(std::string filename, vector<myTriangle> &triangles)
{
	std::fstream makefile;
	makefile.open(filename,ios::out);
	makefile<<"OFF"<<std::endl;
	makefile<<triangles.size()*3<<" "<<triangles.size()<<" 0"<<std::endl;
	for(int i=0 ; i<triangles.size() ; i++)
	{
		makefile<<triangles[i].v0.pos.x<<" " <<triangles[i].v0.pos.y<<" "<<triangles[i].v0.pos.z<<std::endl;
		makefile<<triangles[i].v1.pos.x<<" " <<triangles[i].v1.pos.y<<" "<<triangles[i].v1.pos.z<<std::endl;
		makefile<<triangles[i].v2.pos.x<<" " <<triangles[i].v2.pos.y<<" "<<triangles[i].v2.pos.z<<std::endl;
	}

	for(int i=0 ; i<triangles.size() ; i++)
	{
		makefile<<"3 "<<i*3<<" "<<i*3+1<<" "<<i*3+2<<std::endl;
	}

	makefile.close();
}

//hao modified it
void writeToFilePly(std::string filename, vector<myTriangle> &triangles)
{
  PointCloudPtr cloud(new PointCloud);
  pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh); 

  for(int i=0 ; i<triangles.size() ; i++)
  {
    Point p0(-triangles[i].v0.pos.x, -triangles[i].v0.pos.y, triangles[i].v0.pos.z);
    Point p1(-triangles[i].v1.pos.x, -triangles[i].v1.pos.y, triangles[i].v1.pos.z);
    Point p2(-triangles[i].v2.pos.x, -triangles[i].v2.pos.y, triangles[i].v2.pos.z);

    cloud->push_back(p0);
    cloud->push_back(p1);
    cloud->push_back(p2);

    pcl::Vertices vert;
    vert.vertices.push_back(i*3);
    vert.vertices.push_back(i*3+1);
    vert.vertices.push_back(i*3+2);
    mesh->polygons.push_back(vert);
  }

  pcl::toROSMsg(*cloud, mesh->cloud); 

  pcl::io::savePolygonFilePLY(filename, *mesh);
}

/*
void writeToFile(vector<Triangle> &triangles)
{
	class MyVertex; class MyEdge; class MyFace;

	struct MyUsedTypes : public vcg::UsedTypes<vcg::Use<MyVertex>   ::AsVertexType,

		vcg::Use<MyEdge>     ::AsEdgeType,

		vcg::Use<MyFace>     ::AsFaceType>{};

	class MyVertex  : public vcg::Vertex< MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags  >{};

	class MyFace    : public vcg::Face<   MyUsedTypes, vcg::face::FFAdj,  vcg::face::VertexRef, vcg::face::BitFlags > {};

	class MyEdge    : public vcg::Edge<   MyUsedTypes> {};

	class MyMesh    : public vcg::tri::TriMesh< std::vector<MyVertex>, std::vector<MyFace> , std::vector<MyEdge>  > {};

	MyMesh m;
	MyMesh::VertexIterator vi = vcg::tri::Allocator<MyMesh>::AddVertices(m,3);
	MyMesh::FaceIterator fi = vcg::tri::Allocator<MyMesh>::AddFaces(m,1);

	for(int i=0 ; i<triangles.size() ; i++)
	{
		MyMesh::VertexPointer ivp[3];
		ivp[0] = &*vi; vi->P() = MyMesh::CoordType(triangles[i].v0.pos.x, triangles[i].v0.pos.y, triangles[i].v0.pos.z); ++vi;
		ivp[1] = &*vi; vi->P() = MyMesh::CoordType(triangles[i].v1.pos.x, triangles[i].v1.pos.y, triangles[i].v1.pos.z); ++vi;
		ivp[2] = &*vi; vi->P() = MyMesh::CoordType(triangles[i].v2.pos.x, triangles[i].v2.pos.y, triangles[i].v2.pos.z); ++vi;

		fi->V(0) = ivp[0];
		fi->V(1) = ivp[1];
		fi->V(2) = ivp[2];
	}

	vcg::tri::io::ExporterPLY<MyMesh>::Save(m, "Mesh.ply");
}*/
