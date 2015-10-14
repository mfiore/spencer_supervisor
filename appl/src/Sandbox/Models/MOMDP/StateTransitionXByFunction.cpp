#include "StateTransitionXByFunction.h"
#include "MOMDP.h"

StateTransitionXByFunction::StateTransitionXByFunction(void)
{
}

StateTransitionXByFunction::~StateTransitionXByFunction(void)
{
}

// (unobserved states, observed states)
SharedPointer<SparseMatrix> StateTransitionXByFunction::getMatrix(int a, int x)
{
	stringstream ss;
	ss << "StateTransX a " << a << " x " << x;
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
SharedPointer<SparseMatrix> StateTransitionXByFunction::getMatrixTr(int a, int x)
{
	stringstream ss;
	ss << "StateTransXTr a " << a << " x " << x;
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
SharedPointer<SparseMatrix> StateTransitionXByFunction::getMatrixInner(int a, int x, bool transpose)
{
	kmatrix tempMatrix;

	int numObsState = problem->XStates->size();
	int numUnobsState = problem->YStates->size();
	if(!transpose)
	{
		tempMatrix.resize(numUnobsState, numObsState);
	}
	else
	{
		tempMatrix.resize(numObsState, numUnobsState);
	}

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
			int destX = problem->XStates->indexOf(combinedDestProb);
			if(!transpose)
			{
				if(destX >= numObsState)
				{
					throw runtime_error("index exceeded");
				}
				tempMatrix.push_back(s, destX, combinedProb);
			}
			else
			{
				tempMatrix.push_back(destX, s, combinedProb);
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

