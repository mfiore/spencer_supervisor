#include "RewardMovementCostRel.h"
#include "VariableValue.h"


RewardMovementCostRel::RewardMovementCostRel(SharedPointer<Variable> rewardVar, SharedPointer<Variable> actionVar, SharedPointer<ProblemParams> params)
{
	this->actionVar = actionVar;
	this->rewardVar = rewardVar;
	this->params = params;
	this->addSourceVar(actionVar);
	this->setDestVariable(rewardVar);
}

RewardMovementCostRel::~RewardMovementCostRel(void)
{
}

vector<SharedPointer<RelEntry> > RewardMovementCostRel::getProb(map<string, SharedPointer<IVariableValue> > sourceVals)
{
	vector<SharedPointer<RelEntry> > result;
    vector<string> actions;
	actions.push_back("up");
    actions.push_back("down");
    actions.push_back("left");
    actions.push_back("right");
	

    FOREACH_NOCONST(string, pactionStr , actions)
    {
		if(sourceVals[actionVar->getVariableName()]->getValueName().compare(*pactionStr) == 0)
		{
			// if action variable takes a value in the list
			SharedPointer<RelEntry> newEntry ( new RelEntry());
			// Reward releations does not need to fill in destValues. They only need to fill in the correct reward (stored in prob)
			//newEntry->sourceValues[actionVar->getVariableName()] = actionVar->getValueByName(*pactionStr);
			//newEntry->destValues[rewardVar->getVariableName()] = actionVar->getValueByName(*pactionStr);
			newEntry->prob = params->MovementCost;
			result.push_back(newEntry);
			break;
			// then reward is movement cost
		}
		else
		{
			// if reward is zero, can omit the return value, RewardsByFunction should be able to handle the missing value
		}
	}
        
    return result;
}
