#include "RewardsByFunction.h"
#include "BeliefWithState.h"
#include "Actions.h"
#include "MOMDP.h"
#include <sstream>
RewardsByFunction::RewardsByFunction(void)
{
}

RewardsByFunction::~RewardsByFunction(void)
{
}

REAL_VALUE RewardsByFunction::getReward(BeliefWithState& b, int a)
{
	int Xc = b.sval; // currrent value for observed state variable
	SharedPointer<belief_vector> Bc = b.bvec; // current belief for unobserved state variable

	if (!(getMatrix(Xc)->isColumnEmpty(a)))
	{
		return inner_prod_column( *getMatrix(Xc), a, *Bc );
	}
	else
	{
		return 0;
	}
}

// (unobserved states, action)
SharedPointer<SparseMatrix> RewardsByFunction::getMatrix(int x)
{
	stringstream ss;
	ss << "reward x" << x;
	string key = ss.str();
	if(problem->cache.hasKey(key))
	{
		
	}
	else
	{
		problem->cache.put(key, getMatrixInner(x,false));
	}
	return problem->cache.get(key);
}
//SharedPointer<SparseMatrix> RewardsByFunction::getMatrixTr(int x)
//{
//	return getMatrixInner(x,true);
//}
SharedPointer<SparseMatrix> RewardsByFunction::getMatrixInner(int x, bool transpose)
{
	kmatrix tempMatrix;
	
	int numAction = problem->actions->size();
	int numUnobsState = problem->YStates->size();
	if(!transpose)
	{
		tempMatrix.resize(numUnobsState, numAction);
	}
	else
	{
		tempMatrix.resize(numAction, numUnobsState);
	}

	SharedPointer<SparseMatrix> result (new SparseMatrix());
	ValueSet xVals = problem->XStates->get(x);

	FOR(s, problem->YStates->size())
	{
		ValueSet yVals = problem->YStates->get(s);
		FOR(a, problem->actions->size())
		{
			ValueSet aVals = problem->actions->get(a);

			map<string, SharedPointer<IVariableValue> > sourceVals;
			sourceVals.insert(xVals.vals.begin(), xVals.vals.end());
			sourceVals.insert(yVals.vals.begin(), yVals.vals.end());
			sourceVals.insert(aVals.vals.begin(), aVals.vals.end());

			vector<vector<SharedPointer<RelEntry> > > RelEntries;
			double totalReward = 0.0;
			FOREACH(SharedPointer<VariableRelation> , curRel, relations)
			{
				vector<SharedPointer<RelEntry> > rewards = (*curRel)->getProb(sourceVals);
				if(rewards.size() > 1 )
				{
					throw runtime_error("Reward Relation should return at most one RelEntry");
				}
				if(rewards.size() > 0 )
				{
					totalReward += rewards[0]->prob;
				}

			}

			if(!transpose)
			{
				tempMatrix.push_back(s, a, totalReward);
			}
			else
			{
				tempMatrix.push_back(a, s, totalReward);
			}


		}
	}
		
	copy(*result, tempMatrix);
	return result;
}
