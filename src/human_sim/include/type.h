#ifndef DEF_TYPE
#define DEF_TYPE

#include <iostream>

struct Pose2D
{
	float x;
	float y;
	float theta;
};

struct Goal
{
	std::string type; // for now only position
	float x;
	float y;
	float theta;
};


#endif
