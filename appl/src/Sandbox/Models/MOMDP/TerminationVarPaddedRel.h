#ifndef TerminationVarPaddedRel_H
#define TerminationVarPaddedRel_H


#include <vector>
#include <string>
#include "MObject.h"
#include "IVariableValue.h"
#include "IVariable.h"
#include "VariableValue.h"
#include "Variable.h"
#include "VariableRelation.h"
#include "Map2DPosValue.h"
#include "BooleanVariable.h"
#include "Map2DPosVar.h"
using namespace std;
using namespace momdp;

namespace momdp
{
	// generic class for variable, holds user defined variable value
	class TerminationVarPaddedRel : public VariableRelation
	{
	private:
		SharedPointer<VariableRelation> origRel;
		SharedPointer<BooleanVariable> termVar;

	public:
		TerminationVarPaddedRel(SharedPointer<VariableRelation> origRel, SharedPointer<BooleanVariable> termVar);
		virtual ~TerminationVarPaddedRel(void);
		
		virtual vector<SharedPointer<RelEntry> > getProb(map<string, SharedPointer<IVariableValue> > sourceVals);

	};

}

#endif

