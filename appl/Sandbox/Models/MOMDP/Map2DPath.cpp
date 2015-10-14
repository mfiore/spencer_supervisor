#include "Map2DPath.h"

Map2DPath::Map2DPath(void)
{
}
Map2DPath::~Map2DPath(void)
{
}

void Map2DPath::addStep(SharedPointer<Map2DPosValue> step)
{
	path.push_back(step);
}
vector<SharedPointer<Map2DPosValue> > Map2DPath::getPath()
{
	return path;
}

