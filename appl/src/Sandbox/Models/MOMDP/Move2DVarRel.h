#ifndef Move2DVarRel_H
#define Move2DVarRel_H


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
	class Move2DVarRel: public VariableRelation
	{
	private:
		SharedPointer<Variable> actionVar;
		SharedPointer<Map2DPosVar> posVar;

	public:
		Move2DVarRel(SharedPointer<Variable> actionVar, SharedPointer<Map2DPosVar> robotPosVar);
		virtual ~Move2DVarRel(void);
		
		virtual vector<SharedPointer<RelEntry> > getProb(map<string, SharedPointer<IVariableValue> > sourceVals);

	};

}

#endif

