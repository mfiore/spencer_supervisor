#ifndef Move2DPathFollowerActPosHasGuideNoisyVarRel_H
#define Move2DPathFollowerActPosHasGuideNoisyVarRel_H


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
#include "Move2DPathFollowerActPosVarRel.h"
#include "ProblemParams.h"
#include "BooleanVariable.h"

using namespace std;
using namespace momdp;

namespace momdp
{
	// generic class for variable, holds user defined variable value
	class Move2DPathFollowerActPosHasGuideNoisyVarRel: public VariableRelation
	{
	private:
		SharedPointer<Map2DPath> path;
		SharedPointer<Map2DPosVar> posVar;
		SharedPointer<BooleanVariable> hasGuideVar;
		SharedPointer<Move2DPathFollowerActPosVarRel> pathFollower;
		SharedPointer<ProblemParams> params;
	public:
		
		Move2DPathFollowerActPosHasGuideNoisyVarRel(SharedPointer<Map2DPosVar> robotPosVar, SharedPointer<BooleanVariable> hasGuideVar, SharedPointer<Map2DPath> path, SharedPointer<ProblemParams> params);
		virtual ~Move2DPathFollowerActPosHasGuideNoisyVarRel(void);
		
		virtual vector<SharedPointer<RelEntry> > getProb(map<string, SharedPointer<IVariableValue> > sourceVals) ;

	};

}

#endif

