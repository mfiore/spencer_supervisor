#include "RewardAtGoal.h"
#include "VariableValue.h"


RewardAtGoal::RewardAtGoal(SharedPointer<Variable> rewardVar, SharedPointer<Map2DPosVar> auvPos, SharedPointer<Map2DPath> path, SharedPointer<ProblemParams> params)
{
	this->auvPos = auvPos;
	this->rewardVar = rewardVar;
	this->params = params;
	this->path = path;

	this->addSourceVar(auvPos);
	this->setDestVariable(rewardVar);
}

RewardAtGoal::~RewardAtGoal(void)
{
}

vector<SharedPointer<RelEntry> > RewardAtGoal::getProb(map<string, SharedPointer<IVariableValue> > sourceVals)
{
	vector<SharedPointer<Map2DPosValue> > pathSteps = path->getPath();
	SharedPointer<Map2DPosValue> lastStep = pathSteps[pathSteps.size() - 1];
	SharedPointer<IVariableValue> curPos = sourceVals[auvPos->getVariableName()];

	vector<SharedPointer<RelEntry> > result;
	
	if(lastStep->equals(curPos))
	{
		SharedPointer<RelEntry> newEntry ( new RelEntry());
		//newEntry->sourceValues[lastStep->getVariableName()] = lastStep;
		newEntry->prob = params->OnGoalReward;
		result.push_back(newEntry);
	}
    return result;
}
