#include "TriggerVarRel.h"


TriggerVarRel::TriggerVarRel(SharedPointer<IVariable> destVar, SharedPointer<IVariable> triggerVar, SharedPointer<IVariableValue> triggerValue, SharedPointer<IVariableValue> destTriggerValueValue, SharedPointer<IVariableValue> destNonTriggerValue)
{
	this->destVar = destVar;
	this->triggerVar = triggerVar;
	this->triggerValue = triggerValue;
	this->destTriggerValueValue = destTriggerValueValue;
	this->destNonTriggerValue = destNonTriggerValue;
	this->addSourceVar(triggerVar);
	this->setDestVariable(destVar);
}

TriggerVarRel::~TriggerVarRel(void)
{
}

vector<SharedPointer<RelEntry> > TriggerVarRel::getProb(map<string, SharedPointer<IVariableValue> > sourceVals)
{
	vector<SharedPointer<RelEntry> > result;

    SharedPointer<RelEntry> newEntry (new RelEntry());
	SharedPointer<IVariableValue> srcValue = sourceVals[triggerVar->getVariableName()];
	//newEntry->sourceValues[triggerVar->getVariableName()] = srcValue;

	if(srcValue->equals(triggerValue))
    {
		newEntry->destValues[destVar->getVariableName()] = destTriggerValueValue;
    }
    else
    {
       newEntry->destValues[destVar->getVariableName()] = destNonTriggerValue;
    }
    newEntry->prob = 1.0;
	result.push_back(newEntry);

    return result;

}
