#include "Move2DVarRel.h"


Move2DVarRel::Move2DVarRel(SharedPointer<Variable> actionVar, SharedPointer<Map2DPosVar> robotPosVar)
{
	this->actionVar = actionVar;
	this->posVar = robotPosVar;
	this->addSourceVar(actionVar);
	this->addSourceVar(robotPosVar);
	this->setDestVariable(robotPosVar);
}

Move2DVarRel::~Move2DVarRel(void)
{
}

vector<SharedPointer<RelEntry> > Move2DVarRel::getProb(map<string, SharedPointer<IVariableValue> > sourceVals)
{
	string action = sourceVals[actionVar->getVariableName()]->getValueName();
	SharedPointer<Map2DPosValue> curPos = dynamic_pointer_cast<Map2DPosValue>(sourceVals[posVar->getVariableName()]);
	SharedPointer<Map2DPosValue> nextPos = curPos->duplicate();
	if(action == "left")
	{
		if(nextPos->x >0 )
		{
			nextPos->x --;
		}
	}
	else if(action == "right")
	{
		if(nextPos->x < posVar->width - 1)
		{
			nextPos->x ++;
		}
	}
	else if(action == "up")
	{
		if(nextPos->y >0 )
		{
			nextPos->y --;
		}
	}
	else if(action == "down")
	{
		if(nextPos->y < posVar->height - 1 )
		{
			nextPos->y ++;
		}
	}
	else if(action == "noop")
	{
	}
	else
	{
		cout << "Error: unkonwn action " << action << endl;
		throw runtime_error("Error: unkonwn action ");
	}

	vector<SharedPointer<RelEntry> > result;

	{
		SharedPointer<RelEntry> newEntry (new RelEntry());
		//newEntry->sourceValues[actionVar->getVariableName()] = sourceVals[actionVar->getVariableName()];
		//newEntry->sourceValues[posVar->getVariableName()] = sourceVals[posVar->getVariableName()];
		newEntry->destValues[this->getDestVariable()->getVariableName()] = nextPos;
		newEntry->prob = 1; // noisy value
		result.push_back(newEntry);
	}

	return result;
}
