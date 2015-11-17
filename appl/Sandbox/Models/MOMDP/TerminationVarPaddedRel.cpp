#include "TerminationVarPaddedRel.h"


TerminationVarPaddedRel::TerminationVarPaddedRel(SharedPointer<VariableRelation> origRel, SharedPointer<BooleanVariable> termVar)
{
	this->origRel = origRel;
	this->termVar = termVar;
	this->addSourceVar(termVar);
	vector<SharedPointer<IVariable> > srcVars = origRel->getSourceVars();
	for(int i = 0 ; i < srcVars.size(); i++)
	{
		SharedPointer<IVariable> srcVar = srcVars[i];
		this->addSourceVar(srcVar);
	}
	this->setDestVariable(origRel->getDestVariable());
}

TerminationVarPaddedRel::~TerminationVarPaddedRel(void)
{
}

vector<SharedPointer<RelEntry> > TerminationVarPaddedRel::getProb(map<string, SharedPointer<IVariableValue> > sourceVals)
{
	vector<SharedPointer<RelEntry> > result;
	vector<SharedPointer<RelEntry> > srcEntries = origRel->getProb(sourceVals);

	for(int j = 0 ; j < srcEntries.size() ; j ++)
	{
		
		if(sourceVals[termVar->getVariableName()]->getIndex() == BooleanVariable::FalseValueIndex)
		{
			// if termination variable is false, copy the original relation's value
			SharedPointer<RelEntry> newEntry (new RelEntry());
		
			newEntry->destValues[this->getDestVariable()->getVariableName()] = srcEntries[j]->destValues[this->getDestVariable()->getVariableName()];
			newEntry->prob = srcEntries[j]->prob;
			result.push_back(newEntry);
		}
		else
		{
			// if termination variable is true, copy the original relation's value, but put the destination value to the first value of the variable
			SharedPointer<RelEntry> newEntry (new RelEntry());
			//vector<SharedPointer<IVariable> > srcVars = origRel->getSourceVars();
			//for(int i = 0 ; i < srcVars.size(); i++)
			//{
			//	SharedPointer<IVariable> srcVar = srcVars[i];
			//	newEntry->sourceValues[srcVar->getVariableName()] = srcEntries[j]->sourceValues[srcVar->getVariableName()];
			//}
			//newEntry->sourceValues[termVar->getVariableName()] = srcEntries[j]->sourceValues[termVar->getVariableName()];
			newEntry->destValues[this->getDestVariable()->getVariableName()] = this->getDestVariable()->getValues()[0];
			newEntry->prob = srcEntries[j]->prob; // noisy value
			result.push_back(newEntry);

		}
	}

	return result;
}
