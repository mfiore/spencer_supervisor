#include "RewardOnPath.h"
#include "VariableValue.h"
#include <set>

using namespace std;


RewardOnPath::RewardOnPath(SharedPointer<Variable> rewardVar, SharedPointer<Map2DPosVar> auvPos, SharedPointer<Map2DPath> path, SharedPointer<ProblemParams> params)
{
	this->auvPos = auvPos;
	this->rewardVar = rewardVar;
	this->params = params;
	this->path = path;

	this->addSourceVar(auvPos);
	this->setDestVariable(rewardVar);
}

RewardOnPath::~RewardOnPath(void)
{
}

vector<SharedPointer<RelEntry> > RewardOnPath::getProb(map<string, SharedPointer<IVariableValue> > sourceVals)
{
	vector<SharedPointer<Map2DPosValue> > pathSteps = path->getPath();
	
	SharedPointer<IVariableValue> curPos = sourceVals[auvPos->getVariableName()];

	vector<SharedPointer<RelEntry> > result;
	//set<string> visitedPos;

    FOREACH_NOCONST(SharedPointer<Map2DPosValue>,  pCurStep , pathSteps)
    {
		SharedPointer<Map2DPosValue> curStep = *pCurStep;
		if(curStep->equals(curPos))
		{
			SharedPointer<RelEntry> newEntry ( new RelEntry());
			//newEntry->sourceValues[curStep->getVariableName()] = curStep;
			newEntry->prob = params->OnPathReward;
			result.push_back(newEntry);
		}
		//visitedPos.insert(curStep->ToString());
	}
    //vector<SharedPointer<IVariableValue> > auvPosVals = auvPos->getValues();

	//FOREACH_NOCONST(SharedPointer<IVariableValue>,  pCurStep , auvPosVals)
	//{
	//	SharedPointer<Map2DPosValue> curStep = dynamic_pointer_cast<Map2DPosValue>(*pCurStep);
	//	if(visitedPos.find(curStep->ToString()) == visitedPos.end())
	//	{
	//		// not visited, therefore, is off path
	//		SharedPointer<RelEntry> newEntry ( new RelEntry());
	//		//newEntry->sourceValues[curStep->getVariableName()] = curStep;
	//		newEntry->prob = params->OffPathPenalty;
	//		result.push_back(newEntry);
	//		visitedPos.insert(curStep->ToString());
	//	}
	//}
    return result;
}
