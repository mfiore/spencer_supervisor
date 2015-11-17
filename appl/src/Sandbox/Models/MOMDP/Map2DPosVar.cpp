#include "Map2DPosVar.h"
#include <exception>

Map2DPosVar::Map2DPosVar(string name, int width, int height) : IVariable()
{
	this->variableName = name;
	this->width = width;
	this->height = height;
}

Map2DPosVar::~Map2DPosVar(void)
{
}


string Map2DPosVar::getVariableName()
{
	return variableName;
}

vector<SharedPointer<IVariableValue> > Map2DPosVar::getValues()
{
	vector<SharedPointer<IVariableValue> > result;
	for(int y = 0; y < height; y++)
	{
		for(int x = 0; x < width; x++)
		{
			SharedPointer<Map2DPosValue> newEntry(new Map2DPosValue(variableName, x, y, height));
			result.push_back(newEntry);
		}
	}
	return result;
}

SharedPointer<IVariableValue> Map2DPosVar::getValueByName(string valName)
{
	vector<SharedPointer<IVariableValue> > values = this->getValues();
	for(int i = 0; i < values.size(); i++)
	{
		if(values[i]->getValueName().compare(valName) == 0)
		{
			return values[i];
		}
	}

	throw runtime_error("Cannot find value : " + valName + " in variable : " + this->getVariableName());
}

vector<SharedPointer<IVariableValue> > Map2DPosVar::getInitialValues()
{
	vector<SharedPointer<IVariableValue> > result;
	result.push_back(initPos);
	return result;
}

int Map2DPosVar::getNumValues()
{
	return height* width;
}
void Map2DPosVar::setInitPos(SharedPointer<Map2DPosValue> pos)
{
	pos->prob = 1.0;
	initPos = pos;
}
vector<SharedPointer<Map2DPosValue> > Map2DPosVar::getAdjPos(SharedPointer<Map2DPosValue>  src, double distance)
{
       vector<SharedPointer<Map2DPosValue> > result;
        int range = (int)ceil(distance);
        for(int xdiff = -range ; xdiff < range + 1; xdiff ++)
        {
            for(int ydiff = -range ; ydiff < range + 1; ydiff ++)
            {
                if(xdiff == 0 && ydiff == 0)
                {
                    continue;
                }

				SharedPointer<Map2DPosValue> temp = src->duplicate();
                temp->x += xdiff;
                temp->y += ydiff;
				if(temp->x >= this->width)
                {
                    continue;
                }
                if(temp->x < 0)
                {
                    //temp.x = 0;
                    continue;
                }
                if(temp->y >= this->height)
                {
                    //temp.y = temp.height -1;
                    continue;
                }
                if(temp->y < 0)
                {
                    //temp.y = 0;
                    continue;
                }
				result.push_back(temp);
            }
        }
        return result;
    }

vector<SharedPointer<Map2DPosValue> > Map2DPosVar::getDestSidePosPerpendicularToMovement(SharedPointer<Map2DPosValue>  src, SharedPointer<Map2DPosValue>  dest)
{
    vector<SharedPointer<Map2DPosValue> > sideCells;
        int dX = 0;
        int dY = 0;
        int incX = 0;
        int incY = 0;
        if(dest->x > src->x)
        {
            incX = -1;
        }
        else if(dest->x < src->x)
        {
            incX = 1;
        }

        if(dest->y > src->y)
        {
            incY = -1;
        }
        else if(dest->y < src->y)
        {
            incY = 1;
        }
        

        if(dest->x != src->x)
        {
            // movement in x
            dY = 1;
        }
        else if(dest->y != src->y)
        {
            // movement in y
            dX = 1;
        }
        else
        {
            // did not move...
            //System.out.println("Did not move!");
            dX = 1;
        }
        
        {
            SharedPointer<Map2DPosValue> sideCellVar = src->duplicate();
            sideCellVar->x += dX;
            sideCellVar->y += dY;
            if(sideCellVar->x < this->width && sideCellVar->y < this->height)
            {
				sideCells.push_back(sideCellVar);
            }
        }
        
        {
			SharedPointer<Map2DPosValue> sideCellVar = src->duplicate();
            sideCellVar->x -= dX;
            sideCellVar->y -= dY;
            if(sideCellVar->x >= 0 && sideCellVar->y >= 0)
            {
				sideCells.push_back(sideCellVar);
            }
        }

        {
            if(incX != 0 || incY != 0)
            {
                // only add this cell if it actually moves
                SharedPointer<Map2DPosValue> sideCellVar = src->duplicate();
                sideCellVar->x += incX;
                sideCellVar->y += incY;
                if(sideCellVar->x < this->width && sideCellVar->y < this->height && sideCellVar->x >= 0 && sideCellVar->y >= 0)
                {
					sideCells.push_back(sideCellVar);
                }
            }
        }
        


        return sideCells;
}