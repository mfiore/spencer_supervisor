#include "Move2DPathFollowerActPosHasGuideNoisyVarRel.h"


Move2DPathFollowerActPosHasGuideNoisyVarRel::Move2DPathFollowerActPosHasGuideNoisyVarRel(SharedPointer<Map2DPosVar> robotPosVar, SharedPointer<BooleanVariable> hasGuideVar, SharedPointer<Map2DPath> path, SharedPointer<ProblemParams> params)
{
    this->path = path;
    this->hasGuideVar = hasGuideVar;
    this->posVar = robotPosVar;
	this->params = params;

    pathFollower = SharedPointer<Move2DPathFollowerActPosVarRel>( new Move2DPathFollowerActPosVarRel(robotPosVar, path));
    this->addSourceVar(this->hasGuideVar);
    this->addSourceVar(this->posVar);
    this->setDestVariable(this->posVar);
}

Move2DPathFollowerActPosHasGuideNoisyVarRel::~Move2DPathFollowerActPosHasGuideNoisyVarRel(void)
{
}

vector<SharedPointer<RelEntry> > Move2DPathFollowerActPosHasGuideNoisyVarRel::getProb(map<string, SharedPointer<IVariableValue> > sourceVals)
{
	//string action = sourceVals[destVar->getVariableName()];
	
	vector<SharedPointer<RelEntry> > result2 = pathFollower->getProb(sourceVals);
	SharedPointer<Map2DPosValue> robotCurPos = dynamic_pointer_cast<Map2DPosValue>(sourceVals[posVar->getVariableName()]);
	SharedPointer<VariableValue> hasGuideVal = dynamic_pointer_cast<VariableValue>(sourceVals[hasGuideVar->getVariableName()]);


	vector<SharedPointer<RelEntry> > result;
	for(int i = 0 ; i < result2.size(); i ++ )
	{
		SharedPointer<RelEntry> curRelEntry = result2[i];
		if(sourceVals[this->hasGuideVar->getVariableName()]->getIndex() == BooleanVariable::TrueValueIndex)
		{
			// the case that guide variable is true
			SharedPointer<RelEntry> relEntry (new RelEntry());
			relEntry->destValues.insert(curRelEntry->destValues.begin(), curRelEntry->destValues.end());
			relEntry->prob = 1.0; // move with certainty
			result.push_back(relEntry);

		}
		else
		{
			// the case that guide variable is false
			SharedPointer<RelEntry> newEntry (new RelEntry());
			newEntry->destValues.insert(curRelEntry->destValues.begin(), curRelEntry->destValues.end());
            newEntry->prob = params->AUVBlindMoveProb;
			result.push_back(newEntry);


            // move to dest's adj cells
            //SharedPointer<Map2DPosValue> srcPos = dynamic_pointer_cast<Map2DPosValue>(curRelEntry->sourceValues[posVar->getVariableName()]);
			SharedPointer<Map2DPosValue> srcPos = dynamic_pointer_cast<Map2DPosValue>(sourceVals[posVar->getVariableName()]);
            SharedPointer<Map2DPosValue> destPos = dynamic_pointer_cast<Map2DPosValue>(curRelEntry->destValues[posVar->getVariableName()]);

			vector<SharedPointer<Map2DPosValue> > destAdjVars = posVar->getDestSidePosPerpendicularToMovement(srcPos, destPos);
           
            double noisyProb = (1 - params->AUVBlindMoveProb) / destAdjVars.size();

			for(int i = 0 ; i < destAdjVars .size(); i++)
            {
				SharedPointer<Map2DPosValue>& noisyVar = destAdjVars[i];

                SharedPointer<RelEntry>  newEntry2 ( new RelEntry());
				//newEntry2->sourceValues[this->hasGuideVar->getVariableName()] = this->hasGuideVar->getValueByName("false");
                //newEntry2->sourceValues.insert(curRelEntry->sourceValues.begin(), curRelEntry->sourceValues.end());
				newEntry2->destValues[posVar->getVariableName()] = noisyVar;
                newEntry2->prob = noisyProb;
				result.push_back(newEntry2);
			}
		}

	}


	return result;
}
