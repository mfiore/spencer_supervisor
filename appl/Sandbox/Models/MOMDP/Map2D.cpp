#include "Map2D.h"

Map2D::Map2D(void)
{
}
Map2D::Map2D(int width, int height)
{
	this->width = width;
	this->height = height;
}


Map2D::~Map2D(void)
{
}
void Map2D::setBounds(int width, int height)
{
	this->width = width;
	this->height = height;
}
SharedPointer<Map2DPosVar> Map2D::makePosVar(string name)
{
	SharedPointer<Map2DPosVar> result(new Map2DPosVar(name, width, height));
	return result;
}