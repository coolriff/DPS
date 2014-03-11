#ifndef __OBJLOADER_h_
#define __OBJLOADER_h_

#include <string>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

using namespace std;

class Objloader
{
public:
	Objloader(void);
	~Objloader(void);
	void LoadModel(string name, std::vector<float>* triangles,std::vector<int>* indicies, std::vector<float>* texCoord);
};

#endif
