#ifndef Move2DPathFollowerActPosVarRel_H
#define Move2DPathFollowerActPosVarRel_H


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
#include "Map2DPath.h"
using namespace std;
using namespace momdp;

namespace momdp
{
	// generic class for variable, holds user defined variable value
	class Move2DPathFollowerActPosVarRel: public VariableRelation
	{
	private:
		SharedPointer<Map2DPath> path;
		SharedPointer<Map2DPosVar> posVar;

	public:
		
		Move2DPathFollowerActPosVarRel(SharedPointer<Map2DPosVar> robotPosVar, SharedPointer<Map2DPath> path);
		virtual ~Move2DPathFollowerActPosVarRel(void);
		
		virtual vector<SharedPointer<RelEntry> > getProb(map<string, SharedPointer<IVariableValue> > sourceVals) ;

	};

}

#endif

