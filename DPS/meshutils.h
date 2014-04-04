#ifndef MESHUTILS_H
#define MESHUTILS_H

#include "Ogre.h"

using namespace Ogre;

/**
 * Struct used to pass mesh data among libraries or to load from a file.
 * Such as loading a heightfield file into a physics library.
 */
struct MeshData {
		unsigned int vertexCount;
		unsigned int triangleCount;
		Vector3* vertices;
		Vector3* triangles;

		// todo xmin, xmax, ....

		MeshData() :
				vertexCount(0), triangleCount(0), vertices(0), triangles(0) {}
		~MeshData() {
			delete [] vertices;
			delete [] triangles;
		}
	};


class MeshUtils
	{
	public:
		static MeshData* getMeshData(const MeshPtr& mesh);
		static std::pair<int, int> getMeshCount(const MeshPtr& mesh);
		static void meshBuffersToArrays(const MeshPtr& mesh, Vector3* vertices, unsigned long* indices);
		static ManualObject createWire(const MeshPtr& mesh);
	};

#endif


