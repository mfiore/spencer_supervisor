#include "NoisyVariableRel.h"


NoisyVariableRel::NoisyVariableRel(SharedPointer<VariableRelation> origRel, SharedPointer<IVariableValue> noisyValue, double noisyProbability, SharedPointer<IVariableValue> otherValue)
{
	this->origRel = origRel;
	this->noisyValue = noisyValue;
	this->noisyProbability = noisyProbability;
	this->otherValue = otherValue;
    
	for(int i = 0 ; i < origRel->getSourceVars().size() ; i ++)
    {
		SharedPointer<IVariable> srcVar = origRel->getSourceVars()[i];
        this->addSourceVar(srcVar);
    }

    this->setDestVariable(origRel->getDestVariable());
}

NoisyVariableRel::~NoisyVariableRel(void)
{
}

vector<SharedPointer<RelEntry> > NoisyVariableRel::getProb(map<string, SharedPointer<IVariableValue> > sourceVals)
{
	vector<SharedPointer<RelEntry> > result = origRel->getProb(sourceVals);
	
	string destVarName = origRel->getDestVariable()->getVariableName();

	vector<SharedPointer<RelEntry> > result2;
	for(int j = 0 ; j < result.size(); j ++)
	{
		SharedPointer<RelEntry>& srcEntry = result[j];
		SharedPointer<IVariableValue>& val = srcEntry->destValues[destVarName];
		if(val->getIndex() == noisyValue->getIndex())
		{
			// match
			{
				SharedPointer<IVariableValue> resultValue = this->noisyValue;
				assert(otherValue->getVariableName() == this->getDestVariable()->getVariableName());
				SharedPointer<RelEntry> newEntry (new RelEntry());
				//newEntry->sourceValues.insert(srcEntry->sourceValues.begin(), srcEntry->sourceValues.end());
				newEntry->destValues[destVarName] = resultValue;
				newEntry->prob = this->noisyProbability; // noisy value
				result2.push_back(newEntry);
			}
			{
				SharedPointer<IVariableValue> resultValue = this->otherValue;
				assert(otherValue->getVariableName() == this->getDestVariable()->getVariableName());
				SharedPointer<RelEntry> newEntry (new RelEntry());
				//newEntry->sourceValues.insert(srcEntry->sourceValues.begin(), srcEntry->sourceValues.end());
				newEntry->destValues[destVarName] = resultValue;
				newEntry->prob = 1- this->noisyProbability; // other value
				result2.push_back(newEntry);
			}
		}
		else
		{
			// mismatch, just copy over
			result2.push_back(srcEntry);
		}
		


	}

	return result2;
}
