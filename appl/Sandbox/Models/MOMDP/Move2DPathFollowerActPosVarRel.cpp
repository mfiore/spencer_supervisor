#include "Move2DPathFollowerActPosVarRel.h"
#include <cfloat>

Move2DPathFollowerActPosVarRel::Move2DPathFollowerActPosVarRel(SharedPointer<Map2DPosVar> robotPosVar, SharedPointer<Map2DPath> path)
{
    this->path = path;
    this->posVar = robotPosVar;

	this->addSourceVar(this->posVar);
    this->setDestVariable(this->posVar);
}

Move2DPathFollowerActPosVarRel::~Move2DPathFollowerActPosVarRel(void)
{
}

vector<SharedPointer<RelEntry> > Move2DPathFollowerActPosVarRel::getProb(map<string, SharedPointer<IVariableValue> > sourceVals)
{
	//string action = sourceVals[destVar->getVariableName()];
	
	SharedPointer<Map2DPosValue> actPos = dynamic_pointer_cast<Map2DPosValue>(sourceVals[posVar->getVariableName()]);

	vector<SharedPointer<Map2DPosValue> >  steps = path->getPath();
    double minDistance = DBL_MAX;
    
	SharedPointer<Map2DPosValue> minPosVal;

    for (int i = 0; i < steps.size(); i++)
    {
        SharedPointer<Map2DPosValue>& curStep = steps[i];
        if (actPos->isSamePos(curStep))
        {
            // on path, should move to next step if possible
            SharedPointer<Map2DPosValue> nextStep = curStep;
            if (i + 1 < steps.size())
            {
                nextStep = steps[i + 1];
            }

            minPosVal = nextStep;
            break;
        }

        
        double dist = curStep->distanceTo(actPos);
        if (dist <= minDistance)
        {
            minDistance = dist;
            minPosVal = curStep;
        }
    }

    SharedPointer<Map2DPosValue> destPosVar = actPos->duplicate();
    if (minPosVal->x > destPosVar->x)
    {
        destPosVar->x++;
    }
	else if (minPosVal->x < destPosVar->x)
    {
        destPosVar->x--;
    }
    else if (minPosVal->y < destPosVar->y)
    {
        destPosVar->y--;
    }
    else if (minPosVal->y > destPosVar->y)
    {
        destPosVar->y++;
    }

	vector<SharedPointer<RelEntry> > result;

	SharedPointer<RelEntry> relEntry (new RelEntry());
	//relEntry->sourceValues[posVar->getVariableName()] = actPos;
	relEntry->destValues[this->getDestVariable()->getVariableName()] = destPosVar;
	relEntry->prob = 1.0;
	result.push_back(relEntry);


	return result;
}
