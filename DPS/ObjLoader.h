#ifndef __OBJLOADER_h_
#define __OBJLOADER_h_

#include <string>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <vector>

using namespace std;

class Objloader
{
public:
	void LoadModel(string name, std::vector<float>* triangles,std::vector<int>* indicies);
};

#endif
