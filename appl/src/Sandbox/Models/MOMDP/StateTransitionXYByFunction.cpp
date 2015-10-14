#include "StateTransitionXYByFunction.h"
#include "MOMDP.h"

StateTransitionXYByFunction::StateTransitionXYByFunction(void)
{
}

StateTransitionXYByFunction::~StateTransitionXYByFunction(void)
{
}
// (unobserved states, unobserved states)
SharedPointer<SparseMatrix> StateTransitionXYByFunction::getMatrix(int a, int x)
{
	stringstream ss;
	ss << "StateTransitionXY a " << a << " x " << x;
	string key = ss.str();
	if(problem->cache.hasKey(key))
	{
		
	}
	else
	{
		problem->cache.put(key, getMatrixInner(a,x,false));
	}
	return problem->cache.get(key);
}
SharedPointer<SparseMatrix> StateTransitionXYByFunction::getMatrixTr(int a, int x)
{
	stringstream ss;
	ss << "StateTransitionXYTr a " << a << " x " << x;
	string key = ss.str();
	if(problem->cache.hasKey(key))
	{
		
	}
	else
	{
		problem->cache.put(key, getMatrixInner(a,x,true));
	}
	return problem->cache.get(key);
}
SharedPointer<SparseMatrix> StateTransitionXYByFunction::getMatrixInner(int a, int x, bool transpose)
{
	kmatrix tempMatrix;
	int numUnobsState = problem->YStates->size();
	tempMatrix.resize(numUnobsState, numUnobsState);

	SharedPointer<SparseMatrix> result (new SparseMatrix());
	ValueSet aVals = problem->actions->get(a);
	ValueSet xVals = problem->XStates->get(x);

	FOR(s, problem->YStates->size())
	{
		ValueSet yVals = problem->YStates->get(s);
		map<string, SharedPointer<IVariableValue> > sourceVals;
		sourceVals.insert(aVals.vals.begin(), aVals.vals.end());
		sourceVals.insert(xVals.vals.begin(), xVals.vals.end());
		sourceVals.insert(yVals.vals.begin(), yVals.vals.end());
		
		vector<vector<SharedPointer<RelEntry> > > RelEntries;
		FOREACH(SharedPointer<VariableRelation> , curRel, relations)
		{
			RelEntries.push_back((*curRel)->getProb(sourceVals));
		}

		vector<int> curProgress;
		FOR(index, RelEntries.size())
		{
			curProgress.push_back(0);
		}
		while(true)
		{
			map<string, SharedPointer<IVariableValue> > combinedDestProb;
			double combinedProb = 1.0;
			FOR(index, RelEntries.size())
			{
				vector<SharedPointer<RelEntry> > destProbs =  RelEntries[index];
				int progress = curProgress[index];
				SharedPointer<RelEntry> curRel = destProbs[progress];

				combinedDestProb.insert(curRel->destValues.begin(), curRel->destValues.end());
				combinedProb *= curRel->prob;

			}
			int destY = problem->YStates->indexOf(combinedDestProb);
			if(!transpose)
			{
				tempMatrix.push_back(s, destY, combinedProb);
			}
			else
			{
				tempMatrix.push_back(destY, s, combinedProb);
			}
			// set (s, destX) = combinedProb;

			// move to next combination
			curProgress[0] ++;
			bool done = false;
			FOR(index, RelEntries.size())
			{
				if(curProgress[index] >= RelEntries[index].size())
				{
					curProgress[index] = 0;
					if(index + 1 >= curProgress.size())
					{
						// carry in at most significant pos, should stop
						done = true;
					}
					else
					{
						curProgress[index + 1] ++;
					}
				}
			}

			if(done)
			{
				break;
			}

		}
	}

	copy(*result, tempMatrix);
	return result;
}
