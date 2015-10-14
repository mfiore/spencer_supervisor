#include "Map2DPosValue.h"
#include <sstream>

#include <cmath>

using namespace std;

Map2DPosValue::Map2DPosValue(string varname, int x, int y, int height, double prob)
{
	this->varName = varname;
	this->x = x;
	this->y = y;
	this->height = height;
	this->prob;
}

Map2DPosValue::~Map2DPosValue(void)
{
}

string Map2DPosValue::ToString()
{
	ostringstream ss;
	ss << "x" << x << "y"<< y;
	return ss.str();
}
int Map2DPosValue::getIndex()
{
	return y*height + x;
}

string Map2DPosValue::getVariableName()
{
	return varName;
}
string Map2DPosValue::getValueName()
{
	return this->ToString();
}

double Map2DPosValue::getProb()
{
	return prob;
}

SharedPointer<Map2DPosValue> Map2DPosValue::duplicate()
{
	SharedPointer<Map2DPosValue> result (new Map2DPosValue(this->getVariableName(), this->x, this->y, this->height));
	return result;
}
bool Map2DPosValue::isSamePos(SharedPointer<Map2DPosValue> dest)
{
	if(this->x != dest->x)
	{
		return false;
	}
	if(this->y != dest->y)
	{
		return false;
	}
	if(this->varName.compare(dest->varName) != 0)
	{
		return false;
	}
	return true;
}
double  Map2DPosValue::distanceTo(SharedPointer<Map2DPosValue> dest)
{
    int xDiff = this->x - dest->x;
    int yDiff = this->y - dest->y;

    double totalDiffSq = xDiff*xDiff + yDiff*yDiff;
    return sqrt(totalDiffSq);
}

bool Map2DPosValue::equals(SharedPointer<IVariableValue> obj)
{
	SharedPointer<Map2DPosValue> ptr = dynamic_pointer_cast<Map2DPosValue>(obj);
	bool result = true;
	result &= (this->x == ptr->x);
	result &= (this->y == ptr->y);

	return result;
}
