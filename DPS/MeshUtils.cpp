
#include "MeshUtils.h"

using namespace Ogre;

MeshData* MeshUtils::getMeshData(const MeshPtr& mesh)
{
	MeshData* md = new MeshData();

	std::pair<int, int> count = MeshUtils::getMeshCount(mesh);
	md->vertexCount = count.first;
	md->triangleCount = count.second / 3;

	unsigned long* indices = new unsigned long[count.second];
	
	md->vertices = new Vector3[md->vertexCount];
	md->triangles = new Vector3[md->triangleCount];

	meshBuffersToArrays(mesh, md->vertices, indices);
	for (size_t i = 0; i < md->triangleCount; i+=3) {
			md->triangles->x = indices[i];
			md->triangles->y = indices[i+1];
			md->triangles->z = indices[i+2];
		}

	return md;
}


std::pair<int, int> MeshUtils::getMeshCount(const MeshPtr& mesh)
{
	size_t vertex_count = 0;
	size_t index_count = 0;
	bool added_shared = false;

	for (int j = 0; j < mesh->getNumSubMeshes(); j++) {
			SubMesh* submesh = mesh->getSubMesh(j);

//			VertexData* vertex_data = submesh->vertexData;
//			IndexData* index_data = submesh->indexData;

			// We only need to add the shared vertices once
			if (submesh->useSharedVertices) {
					if (!added_shared) {
							vertex_count += mesh->sharedVertexData->vertexCount;
							added_shared = true;
						}
				} else {
					vertex_count += submesh->vertexData->vertexCount;
				}

			// Add the indices
			index_count += submesh->indexData->indexCount;
		}

	return std::make_pair(vertex_count, index_count);
}

void MeshUtils::meshBuffersToArrays(const MeshPtr& mesh, Vector3* vertices, unsigned long* indices)
{
	bool added_shared = false;
	size_t current_offset = 0;
	size_t shared_offset = 0;
	size_t next_offset = 0;
	size_t index_offset = 0;

	//const Ogre::Vector3 &position = ent->getParentNode()->_getDerivedPosition();
	//const Ogre::Quaternion &orient = ent->getParentNode()->_getDerivedOrientation();
	//const Ogre::Vector3 &scale = ent->getParentNode()->_getDerivedScale();

	const Ogre::Vector3 &position = Vector3::ZERO;
	const Ogre::Quaternion &orient = Quaternion::IDENTITY;
	const Ogre::Vector3 &scale = Vector3::UNIT_SCALE;

	Mesh::SubMeshIterator itr = mesh->getSubMeshIterator();
	while (itr.hasMoreElements()) {
			SubMesh* submesh = itr.getNext();

			Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

			if ((!submesh->useSharedVertices) || (submesh->useSharedVertices && !added_shared)) {
					if (submesh->useSharedVertices) {
							added_shared = true;
							shared_offset = current_offset;
						}

					const Ogre::VertexElement* posElem =
					    vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

					Ogre::HardwareVertexBufferSharedPtr vbuf =
					    vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

					unsigned char* vertex =
					    static_cast<unsigned char*> (vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

					// There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
					//  as second argument. So make it float, to avoid trouble when Ogre::Real will
					//  be comiled/typedefed as double:
					//      Ogre::Real* pReal;
					float* pReal;

					for (size_t k = 0; k < vertex_data->vertexCount; ++k, vertex += vbuf->getVertexSize()) {
							posElem->baseVertexPointerToElement(vertex, &pReal);

							Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);

							vertices[current_offset + k] = (orient * (pt * scale)) + position;
							//vertices[current_offset + k] = pt;
						}

					vbuf->unlock();
					next_offset += vertex_data->vertexCount;
				}


			Ogre::IndexData* index_data = submesh->indexData;
			size_t numTris = index_data->indexCount / 3;
			Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

			bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

			unsigned long* pLong = static_cast<unsigned long*> (ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
			unsigned short* pShort = reinterpret_cast<unsigned short*> (pLong);


			size_t offset = (submesh->useSharedVertices) ? shared_offset : current_offset;

			if (use32bitindexes) {
					for (size_t k = 0; k < numTris * 3; ++k) {
							indices[index_offset++] = pLong[k] + static_cast<unsigned long> (offset);
						}
				} else {
					for (size_t k = 0; k < numTris * 3; ++k) {
							indices[index_offset++] = static_cast<unsigned long> (pShort[k]) +
							                          static_cast<unsigned long> (offset);
						}
				}

			ibuf->unlock();
			current_offset = next_offset;
		}
}

ManualObject MeshUtils::createWire(const MeshPtr& mesh)
{
	ManualObject mo("ManualObject");	
	 
	MeshData* md = MeshUtils::getMeshData(mesh);

	mo.begin("BaseWhiteNoLighting", RenderOperation::OT_TRIANGLE_LIST);
 
	for(unsigned int i=0; i<md->triangleCount; i++)
	{
		Vector3 v = md->triangles[i]; 
		mo.position(v.x, v.y, v.z);
		std::cout << v << std::endl;
	}
	mo.end();
	
	return mo;
}
