#ifndef OBJLOADER_H
#define OBJLOADER_H

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
	void LoadModel(string name, std::vector<float>* triangles=NULL,std::vector<int>* indicies=NULL);
};

#endif
