#include "InRangeAllNto1VarRel.h"



InRangeAllNto1VarRel::InRangeAllNto1VarRel(SharedPointer<BooleanVariable> resultVar, SharedPointer<Map2DPosVar> mapDestVar, double range)
{
	this->resultVar = resultVar;
	this->mapDestVar = mapDestVar;
	this->range = range;
	this->addSourceVar(mapDestVar);
	this->setDestVariable(resultVar);
}

InRangeAllNto1VarRel::~InRangeAllNto1VarRel(void)
{
}
void InRangeAllNto1VarRel::addSrcVar(SharedPointer<Map2DPosVar> srcVar)
{
    mapSrcVars.push_back(srcVar);
    this->addSourceVar(srcVar);
}

vector<SharedPointer<RelEntry> > InRangeAllNto1VarRel::getProb(map<string, SharedPointer<IVariableValue> > sourceVals)
{
	//string action = sourceVals[destVar->getVariableName()];
	
	bool inRange = true;
	SharedPointer<Map2DPosValue> destVarPos = dynamic_pointer_cast<Map2DPosValue>(sourceVals[mapDestVar->getVariableName()]);
	for(int i = 0 ; i < mapSrcVars.size() ; i ++ )
	{
		SharedPointer<Map2DPosVar> srcVar = mapSrcVars[i];
		
		SharedPointer<Map2DPosValue> srcVarPos = dynamic_pointer_cast<Map2DPosValue>(sourceVals[srcVar->getVariableName()]);
		double dist = srcVarPos->distanceTo(destVarPos);
		if(dist > range)
		{
			inRange = false;
			break;
		}
	}

	

	vector<SharedPointer<RelEntry> > result;
	{
		SharedPointer<IVariableValue> resultValue;
		if(inRange)
		{
			 resultValue = resultVar->getTrueValue();
		}
		else
		{
			resultValue = resultVar->getFalseValue();
		}
		
		SharedPointer<RelEntry> newEntry (new RelEntry());
		/*for(int i = 0 ; i < srcVars.size(); i++)
		{
			SharedPointer<IVariable>& sourceVar = srcVars[i];
			newEntry->sourceValues[sourceVar->getVariableName()] = sourceVals[sourceVar->getVariableName()];
		}*/
		newEntry->destValues[this->getDestVariable()->getVariableName()] = resultValue;
		newEntry->prob = 1; // noisy value
		result.push_back(newEntry);
	}
	return result;
}
