#include "Objloader.h"
#include <string>
#include <glm.hpp>
#include <vector>

void Objloader::LoadModel(string name, std::vector<float>* triangles,std::vector<int>* indicies) {
	// First check to see if it's already loaded and if so, just return it

	std::vector<unsigned int> vertexIndices, normalIndices, uvIndices;
	std::vector<glm::vec3> temp_vertices; 
	std::vector<glm::vec3> tempColours;
	std::vector<glm::vec2> tempUVs;
	std::vector<glm::vec3> temp_normals;
	bool hasUVs = false;

	string objFileName = "Content/" + name + ".objm";	
	std::ifstream is;

	is.open(objFileName, ios::in);

	if (is.is_open())
	{
		string s;
		string junk;
		string materialName;
		string materialPrefix = "usemtl";
		while(!is.eof())
		{
			getline(is,s);
			if (s.find("vn ") == 0)
			{
				stringstream ss(s);
				glm::vec3 vertex;
				ss >> junk >> vertex.x >> vertex.y >> vertex.z;
				temp_normals.push_back(vertex);
			}
			else if (s.find("v ") == 0)
			{
				stringstream ss(s);
				glm::vec3 vertex;
				ss >> junk >> vertex.x >> vertex.y >> vertex.z;
				temp_vertices.push_back(vertex);
			}			
			else if (s.find("vt ") == 0)
			{
				stringstream ss(s);
				glm::vec2 uv;
				ss >> junk >> uv.x >> uv.y;
				tempUVs.push_back(uv);
			}			

			else if (s.find(materialPrefix) == 0)
			{
				materialName = s.substr(materialPrefix.length() + 1);
			}
			else if (s.find("f ") == 0) {
				// Its a face in other words 3 vertices 
				unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];

				// Are there any texels? IF so parse them
				if (s.find("//") == string::npos)
				{
					hasUVs = true;
					// Much nicer to do this parse in C
					int matches = sscanf(s.substr(2).c_str()
						,"%d/%d/%d %d/%d/%d %d/%d/%d"
						,&vertexIndex[0]
					,&uvIndex[0]
					,&normalIndex[0]
					,&vertexIndex[1]
					,&uvIndex[1]
					,&normalIndex[1]
					,&vertexIndex[2]
					,&uvIndex[2]
					,&normalIndex[2] 
					);
					if (matches != 9){
						//throw ("Model file could not be loaded");

					}
					uvIndices.push_back(uvIndex[0]);
					uvIndices.push_back(uvIndex[1]);
					uvIndices.push_back(uvIndex[2]);		
				}
				else
				{
					hasUVs = false;
					// Much nicer to do this parse in C
					int matches = sscanf(s.substr(2).c_str()
						,"%d//%d %d//%d %d//%d"
						,&vertexIndex[0]
					,&normalIndex[0]
					,&vertexIndex[1]
					,&normalIndex[1]
					,&vertexIndex[2]
					,&normalIndex[2] 
					);
					if (matches != 6){
						//cout << "" << endl;
						//throw ("Model file could not be loaded");
					}
				}

				if(triangles)
				{
					triangles->push_back(vertexIndex[0]);
					triangles->push_back(vertexIndex[1]);
					triangles->push_back(vertexIndex[2]);
				}

				if(indicies)
				{
					indicies->push_back(normalIndex[0]);
					indicies->push_back(normalIndex[1]);
					indicies->push_back(normalIndex[2]);
				}

				vertexIndices.push_back(vertexIndex[0]);
				vertexIndices.push_back(vertexIndex[1]);
				vertexIndices.push_back(vertexIndex[2]);

				normalIndices.push_back(normalIndex[0]);
				normalIndices.push_back(normalIndex[1]);
				normalIndices.push_back(normalIndex[2]);
			}
		}
		is.close();
	}
}