#ifndef TriggerVarRel_H
#define TriggerVarRel_H


#include <vector>
#include <string>
#include "MObject.h"
#include "IVariableValue.h"
#include "IVariable.h"
#include "VariableValue.h"
#include "Variable.h"
#include "VariableRelation.h"
#include "Map2DPosValue.h"
#include "Map2DPosVar.h"
using namespace std;
using namespace momdp;

namespace momdp
{
	// generic class for variable, holds user defined variable value
	class TriggerVarRel: public VariableRelation
	{
	private:
		SharedPointer<IVariable> destVar;
		SharedPointer<IVariable> triggerVar;
		SharedPointer<IVariableValue> triggerValue;
		SharedPointer<IVariableValue> destTriggerValueValue;
		SharedPointer<IVariableValue> destNonTriggerValue;

	public:
		TriggerVarRel(SharedPointer<IVariable> destVar, SharedPointer<IVariable> triggerVar, SharedPointer<IVariableValue> triggerValue, SharedPointer<IVariableValue> destTriggerValueValue, SharedPointer<IVariableValue> destNonTriggerValue);
		virtual ~TriggerVarRel(void);
		
		virtual vector<SharedPointer<RelEntry> > getProb(map<string, SharedPointer<IVariableValue> > sourceVals);

	};

}

#endif

